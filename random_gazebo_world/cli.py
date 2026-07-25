from __future__ import annotations

import argparse
from pathlib import Path

from random_gazebo_world.config import load_config
from random_gazebo_world.feasibility import enforce_feasibility
from random_gazebo_world.pipeline import generate_valid_world, write_world_outputs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="random_gazebo_world",
        description="Generate procedural Gazebo worlds from rectangular room layouts.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="Generate a world layout.")
    generate.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Path to YAML config file.",
    )
    generate.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed override. Defaults to config random_seed.",
    )
    generate.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Output directory for generated world artifacts.",
    )
    generate.add_argument(
        "--strict-config",
        action="store_true",
        help="Treat feasibility warnings as errors.",
    )
    generate.add_argument(
        "--debug-retries",
        action="store_true",
        help="Print periodic retry diagnostics to stderr.",
    )
    generate.add_argument(
        "--debug-retries-summary-interval",
        type=int,
        default=25,
        help="How often to print retry diagnostics (in rejected attempts).",
    )
    return parser


def generate_world(
    config,
    out_dir: Path,
    debug_retries: bool = False,
    debug_retries_summary_interval: int = 25,
) -> None:
    world = generate_valid_world(
        config,
        debug_retries=debug_retries,
        debug_retry_summary_interval=debug_retries_summary_interval,
    )
    write_world_outputs(world, out_dir)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "generate":
        config = load_config(args.config)
        if args.seed is not None:
            config = config.with_seed(args.seed)
        enforce_feasibility(config, strict=args.strict_config)
        generate_world(
            config,
            args.out,
            debug_retries=args.debug_retries,
            debug_retries_summary_interval=args.debug_retries_summary_interval,
        )
        return 0

    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
