#!/usr/bin/env python3
"""End-to-end Nav2 navigation benchmark orchestrator (the CI/CD entrypoint).

Pipeline:
    1. Generate N TurtleBot3-tailored worlds (with deterministic start/goal).
    2. Augment each world SDF so Gazebo + Nav2 can simulate it.
    3. Build the Nav2 parameter profiles (planner x controller x costmap).
    4. For every (world x profile) pair, run one headless Gazebo + Nav2 trial via
       ``demo/launch/trial.launch.py`` and collect a result JSON.
    5. Aggregate everything into results.csv + summary.md (+ a chart if possible).
    6. Apply the CI gate: the baseline profile must succeed on every world,
       otherwise exit non-zero.

Run it from the repo root inside the uv project venv with ROS 2 on PATH, e.g.:
    uv run python demo/orchestrate.py --config demo/configs/turtlebot_nav.yaml \
        --worlds 3 --profiles curated --out demo/reports

A trial PASSES iff the goal is reached within tolerance before the timeout and
the robot never came within the collision threshold of an obstacle.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parent
_SCRIPTS_DIR = _THIS_DIR / "scripts"
_LAUNCH_FILE = _THIS_DIR / "launch" / "trial.launch.py"

sys.path.insert(0, str(_SCRIPTS_DIR))

import augment_world  # noqa: E402
import make_profiles  # noqa: E402

CSV_FIELDS = [
    "world_id", "seed", "profile", "planner", "controller", "inflation",
    "is_gate", "result", "passed", "collision", "timed_out", "nav_time_s",
    "wall_time_s", "planned_path_length_m", "distance_traveled_m",
    "min_clearance_m", "n_recoveries", "straight_line_distance_m", "error",
    "min_distance_to_goal_m", "position_reached",
]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=_THIS_DIR / "configs" / "turtlebot_nav.yaml")
    parser.add_argument("--worlds", type=int, default=3,
                        help="Number of worlds to generate (seeds from config base).")
    parser.add_argument("--seeds", default=None,
                        help="Explicit comma-separated seed list (overrides --worlds).")
    parser.add_argument("--profiles", default="curated", choices=["curated", "full"])
    parser.add_argument("--out", type=Path, default=_THIS_DIR / "reports")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="Max navigation time (sim seconds) per trial.")
    parser.add_argument("--wall-timeout", type=float, default=300.0,
                        help="Max wall-clock seconds the runner waits per trial.")
    parser.add_argument("--collision-threshold", type=float, default=0.16)
    parser.add_argument("--goal-tolerance", type=float, default=0.35,
                        help="Position tolerance (m) for benchmark success.")
    parser.add_argument("--launch-timeout", type=float, default=180.0,
                        help="Extra wall seconds (beyond --wall-timeout) before the "
                             "whole trial launch is force-killed.")
    parser.add_argument("--headless", default="True", choices=["True", "False"])
    parser.add_argument("--base-params", type=Path, default=None,
                        help="Nav2 base params (auto-detected by default).")
    parser.add_argument("--skip-generate", action="store_true",
                        help="Reuse existing generated worlds in <out>/worlds.")
    parser.add_argument("--no-kill-stragglers", action="store_true",
                        help="Do not pkill leftover 'gz sim' processes between trials.")
    parser.add_argument("--dds", default="auto",
                        choices=["auto", "server", "cyclone", "off"],
                        help="'auto' uses CycloneDDS when the installed Fast-CDR "
                             "library is ABI-incompatible, otherwise Fast DDS. "
                             "'server' starts a local loopback Fast DDS discovery "
                             "server and points all trial processes at it (fixes "
                             "hosts where default DDS discovery fails, e.g. WSL2). "
                             "'cyclone' selects rmw_cyclonedds_cpp. 'off' leaves "
                             "the host DDS config untouched.")
    return parser.parse_args(argv)


def setup_dds(args: argparse.Namespace) -> subprocess.Popen | None:
    """Configure loopback-only DDS discovery for all child trial processes.

    Returns the discovery-server process (if this call started one) so the caller
    can terminate it on shutdown. Child ``ros2 launch`` processes inherit the
    environment set here.
    """
    mode = resolve_dds_mode(args.dds)
    if mode == "off":
        print("[orchestrate] --dds off: using host DDS configuration as-is")
        return None
    if mode == "cyclone":
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        for key in (
            "ROS_DISCOVERY_SERVER",
            "ROS_SUPER_CLIENT",
            "ROS_DISCOVERY_INTERFACE",
            "ROS_DISCOVERY_PORT",
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            "FASTDDS_DEFAULT_PROFILES_FILE",
        ):
            os.environ.pop(key, None)
        print("[orchestrate] using CycloneDDS (rmw_cyclonedds_cpp)")
        return None

    profile = _THIS_DIR / "config" / "fastdds_localhost.xml"
    os.environ.pop("RMW_IMPLEMENTATION", None)
    os.environ.pop("ROS_DISCOVERY_INTERFACE", None)
    os.environ.pop("ROS_DISCOVERY_PORT", None)
    os.environ["ROS_DISCOVERY_SERVER"] = "127.0.0.1:11811"
    os.environ["ROS_SUPER_CLIENT"] = "TRUE"
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
    os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = str(profile)

    if _port_listening(11811):
        print("[orchestrate] reusing Fast DDS discovery server on 127.0.0.1:11811")
        return None

    try:
        proc = subprocess.Popen(
            ["fastdds", "discovery", "-i", "0", "-l", "127.0.0.1", "-p", "11811"],
            stdout=open("/tmp/demo_fastdds_discovery.log", "w"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except FileNotFoundError:
        print("[orchestrate] WARNING: 'fastdds' CLI not found; cannot start a local "
              "discovery server. Continuing with host DDS config.")
        return None
    time.sleep(3)
    print("[orchestrate] started local Fast DDS discovery server "
          f"on 127.0.0.1:11811 (pid {proc.pid})")
    return proc


def resolve_dds_mode(requested: str) -> str:
    if requested != "auto":
        return requested
    if not _fastcdr_exports_uint_serialize() and _ros_package_available(
        "rmw_cyclonedds_cpp"
    ):
        print(
            "[orchestrate] Fast-CDR ABI check failed; auto-selecting CycloneDDS"
        )
        return "cyclone"
    return "server"


def _fastcdr_exports_uint_serialize() -> bool:
    lib = Path("/opt/ros") / os.environ.get("ROS_DISTRO", "jazzy") / "lib" / "libfastcdr.so.2"
    if not lib.is_file():
        return True
    try:
        out = subprocess.run(
            ["nm", "-D", str(lib)],
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        ).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return True
    return "_ZN8eprosima7fastcdr3Cdr9serializeEj" in out


def _ros_package_available(package: str) -> bool:
    return subprocess.run(
        ["ros2", "pkg", "prefix", package],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode == 0


def _port_listening(port: int) -> bool:
    try:
        out = subprocess.run(
            ["ss", "-uln"], capture_output=True, text=True, check=False
        ).stdout
    except FileNotFoundError:
        return False
    return f":{port}" in out


def resolve_seeds(args: argparse.Namespace) -> list[int]:
    if args.seeds:
        return [int(s.strip()) for s in args.seeds.split(",") if s.strip()]
    import yaml

    with args.config.open(encoding="utf-8") as handle:
        base_seed = int(yaml.safe_load(handle).get("random_seed", 0))
    return [base_seed + i for i in range(args.worlds)]


def generate_worlds(args: argparse.Namespace, seeds: list[int]) -> dict[int, Path]:
    from random_gazebo_world.config import load_config
    from random_gazebo_world.pipeline import generate_valid_world, write_world_outputs

    worlds_dir = args.out / "worlds"
    fallback_worlds_dir = _THIS_DIR / "reports" / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    config = load_config(args.config)

    world_dirs: dict[int, Path] = {}
    for seed in seeds:
        wdir = worlds_dir / f"world_{seed}"
        world_sdf = wdir / f"{wdir.name}.sdf"
        if args.skip_generate and not world_sdf.is_file():
            fallback = fallback_worlds_dir / f"world_{seed}"
            fallback_sdf = fallback / f"{fallback.name}.sdf"
            if fallback != wdir and fallback_sdf.is_file():
                print(
                    "[orchestrate] reusing generated world "
                    f"seed={seed} from {fallback}"
                )
                wdir = fallback
        if not args.skip_generate:
            print(f"[orchestrate] generating world seed={seed} -> {wdir}")
            world = generate_valid_world(config.with_seed(seed))
            write_world_outputs(world, wdir)
            augment_world.augment_world_sdf(
                wdir / f"{wdir.name}.sdf", wdir / "world_nav.sdf"
            )
        world_sdf = wdir / f"{wdir.name}.sdf"
        if not world_sdf.is_file():
            raise FileNotFoundError(
                f"Missing generated world for seed {seed}: {world_sdf}"
            )
        if not (wdir / "world_nav.sdf").is_file():
            augment_world.augment_world_sdf(world_sdf, wdir / "world_nav.sdf")
        world_dirs[seed] = wdir
    return world_dirs


def kill_stragglers() -> None:
    # Kill every process type a trial can spawn. Nav2's component container and
    # the benchmark runner can survive a partial launch teardown (e.g. when the
    # Gazebo server dies), and if left running they squat on the ROS graph and
    # corrupt the next trial. Order matters: take down the sim first.
    for pattern in (
        "gz sim",
        "ros_gz_bridge",
        "parameter_bridge",
        "ruby",
        "component_container_isolated",
        "benchmark_runner.py",
        "robot_state_publisher",
    ):
        subprocess.run(
            ["pkill", "-9", "-f", pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )


def run_trial(
    args: argparse.Namespace,
    seed: int,
    world_dir: Path,
    profile: dict,
    result_path: Path,
) -> dict:
    result_path.parent.mkdir(parents=True, exist_ok=True)
    if result_path.exists():
        result_path.unlink()

    cmd = [
        "ros2", "launch", str(_LAUNCH_FILE),
        f"world:={(world_dir / 'world_nav.sdf').resolve()}",
        f"map:={(world_dir / 'map.yaml').resolve()}",
        f"params_file:={Path(profile['params_file']).resolve()}",
        f"nav_task:={(world_dir / 'nav_task.json').resolve()}",
        f"result_out:={result_path.resolve()}",
        f"timeout:={args.timeout}",
        f"wall_timeout:={args.wall_timeout}",
        f"collision_threshold:={args.collision_threshold}",
        f"goal_tolerance:={args.goal_tolerance}",
        f"headless:={args.headless}",
        "use_rviz:=False",
        f"world_id:={seed}",
        f"profile:={profile['name']}",
        f"planner:={profile['planner']}",
        f"controller:={profile['controller']}",
        f"inflation:={profile['inflation']}",
    ]

    hard_timeout = args.wall_timeout + args.launch_timeout
    print(f"[orchestrate] trial world={seed} profile={profile['name']} "
          f"(hard timeout {hard_timeout:.0f}s)")
    proc = subprocess.Popen(cmd, cwd=str(_REPO_ROOT), start_new_session=True)
    try:
        proc.wait(timeout=hard_timeout)
    except subprocess.TimeoutExpired:
        print(f"[orchestrate] trial timed out, killing launch group pid={proc.pid}")
        _terminate_group(proc)
    finally:
        if not args.no_kill_stragglers:
            kill_stragglers()
        time.sleep(2.0)

    return load_result(result_path, seed, profile)


def _terminate_group(proc: subprocess.Popen) -> None:
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=10)
            return
        except subprocess.TimeoutExpired:
            continue


def load_result(result_path: Path, seed: int, profile: dict) -> dict:
    if result_path.is_file():
        with result_path.open(encoding="utf-8") as handle:
            result = json.load(handle)
    else:
        result = {
            "result": "NO_RESULT",
            "passed": False,
            "error": "runner produced no result file (likely a launch/timeout failure)",
        }
    result.setdefault("collision", None)
    result.setdefault("timed_out", None)
    result.update(
        {
            "seed": seed,
            "world_id": result.get("world_id") or str(seed),
            "profile": profile["name"],
            "planner": profile["planner"],
            "controller": profile["controller"],
            "inflation": profile["inflation"],
            "is_gate": bool(profile.get("is_gate", False)),
        }
    )
    return result


def write_csv(rows: list[dict], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in CSV_FIELDS})


def _mean(values: list[float]) -> float | None:
    clean = [v for v in values if isinstance(v, (int, float))]
    return sum(clean) / len(clean) if clean else None


def summarize(rows: list[dict]) -> dict[str, dict]:
    summary: dict[str, dict] = {}
    for row in rows:
        name = row["profile"]
        bucket = summary.setdefault(
            name,
            {
                "trials": 0, "passes": 0, "is_gate": row.get("is_gate", False),
                "nav_times": [], "distances": [], "clearances": [],
            },
        )
        bucket["trials"] += 1
        bucket["passes"] += 1 if row.get("passed") else 0
        if row.get("passed"):
            bucket["nav_times"].append(row.get("nav_time_s"))
            bucket["distances"].append(row.get("distance_traveled_m"))
        bucket["clearances"].append(row.get("min_clearance_m"))
    return summary


def write_summary_md(rows: list[dict], summary: dict[str, dict], path: Path) -> None:
    lines = ["# Nav2 Benchmark Summary", ""]
    lines.append(f"Total trials: {len(rows)}")
    lines.append("")
    lines.append("## Per-profile results")
    lines.append("")
    lines.append("| Profile | Gate | Trials | Passes | Success % | "
                 "Mean nav time (s) | Mean dist (m) | Mean min clearance (m) |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for name, bucket in summary.items():
        rate = 100.0 * bucket["passes"] / bucket["trials"] if bucket["trials"] else 0.0
        nav = _mean(bucket["nav_times"])
        dist = _mean(bucket["distances"])
        clear = _mean(bucket["clearances"])
        lines.append(
            f"| {name} | {'yes' if bucket['is_gate'] else ''} | {bucket['trials']} "
            f"| {bucket['passes']} | {rate:.0f}% "
            f"| {nav:.1f} | {dist:.2f} | {clear:.3f} |".replace("None", "-")
            if nav is not None and dist is not None and clear is not None
            else f"| {name} | {'yes' if bucket['is_gate'] else ''} | {bucket['trials']} "
                 f"| {bucket['passes']} | {rate:.0f}% | - | - | "
                 f"{('%.3f' % clear) if clear is not None else '-'} |"
        )
    lines.append("")
    lines.append("## Per-trial detail")
    lines.append("")
    lines.append("| World | Profile | Result | Pass | Collision | Timeout | "
                 "Nav time (s) | Planned (m) | Traveled (m) | Min clear (m) | Recov |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for row in rows:
        def fmt(key, spec="%s"):
            value = row.get(key)
            return "-" if value is None else (spec % value)
        lines.append(
            f"| {row.get('seed')} | {row.get('profile')} | {row.get('result')} "
            f"| {'PASS' if row.get('passed') else 'FAIL'} | {fmt('collision')} "
            f"| {fmt('timed_out')} | {fmt('nav_time_s', '%.1f')} "
            f"| {fmt('planned_path_length_m', '%.2f')} "
            f"| {fmt('distance_traveled_m', '%.2f')} "
            f"| {fmt('min_clearance_m', '%.3f')} | {fmt('n_recoveries')} |"
        )
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_chart(summary: dict[str, dict], path: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:  # noqa: BLE001
        return
    names = list(summary.keys())
    rates = [
        100.0 * b["passes"] / b["trials"] if b["trials"] else 0.0
        for b in summary.values()
    ]
    fig, ax = plt.subplots(figsize=(max(6, len(names) * 1.5), 4))
    ax.bar(names, rates)
    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(0, 100)
    ax.set_title("Nav2 profile success rate")
    plt.xticks(rotation=30, ha="right")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    seeds = resolve_seeds(args)
    print(f"[orchestrate] seeds: {seeds}")

    dds_server = setup_dds(args)

    world_dirs = generate_worlds(args, seeds)

    base_params = args.base_params or make_profiles.default_base_params()
    if not Path(base_params).is_file():
        print(f"[orchestrate] ERROR: base params not found: {base_params}",
              file=sys.stderr)
        return 2
    profile_paths = make_profiles.write_profiles(
        Path(base_params), args.out / "profiles", args.profiles
    )
    with (args.out / "profiles" / "profiles.json").open(encoding="utf-8") as handle:
        profiles = json.load(handle)["profiles"]
    print(f"[orchestrate] {len(profiles)} profiles, {len(seeds)} worlds -> "
          f"{len(profiles) * len(seeds)} trials")

    trials_dir = args.out / "trials"
    rows: list[dict] = []
    try:
        for seed in seeds:
            for profile in profiles:
                result_path = trials_dir / f"world_{seed}__{profile['name']}.json"
                rows.append(
                    run_trial(args, seed, world_dirs[seed], profile, result_path)
                )
    finally:
        if dds_server is not None and dds_server.poll() is None:
            try:
                os.killpg(os.getpgid(dds_server.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    write_csv(rows, args.out / "results.csv")
    summary = summarize(rows)
    write_summary_md(rows, summary, args.out / "summary.md")
    write_chart(summary, args.out / "success_rate.png")

    gate_rows = [r for r in rows if r.get("is_gate")]
    gate_failures = [r for r in gate_rows if not r.get("passed")]
    print(f"\n[orchestrate] wrote {args.out / 'results.csv'} and "
          f"{args.out / 'summary.md'}")
    if gate_rows:
        print(f"[orchestrate] gate: {len(gate_rows) - len(gate_failures)}/"
              f"{len(gate_rows)} baseline trials passed")
    if gate_failures:
        for row in gate_failures:
            print(f"[orchestrate] GATE FAIL world={row['seed']} "
                  f"profile={row['profile']} result={row.get('result')}")
        return 1
    print("[orchestrate] gate PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
