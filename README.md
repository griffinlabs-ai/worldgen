# random-gazebo-world

Procedural Gazebo world generator for flat indoor navigation experiments.

The generator builds one-level, indoor worlds from room-like cells connected by
door **gates** and corridor **passages**. It exports:

- a Gazebo / SDFormat world (`world.sdf`)
- a Nav2 occupancy map (`map.png` + `map.yaml`)
- deterministic start/goal task metadata for the Nav2 demo (`nav_task.json`)
- layout metadata (`layout.json`, `metadata.json`)
- staged debug images under `debug/`

The repository also contains an end-to-end Gazebo + Nav2 benchmark in
[`demo/`](demo/README.md). That demo generates TurtleBot3-scale worlds, runs
Nav2 profiles against them, records metrics, and applies a CI-style gate.

## Current Status

The generator and its unit tests are working in the current WSL2 / ROS 2 Jazzy
environment.

Verified locally:

```bash
.venv/bin/python -m pytest tests -q -s
# 124 passed
```

Also verified:

- fresh generated SDF validates with `gz sdf -k`
- a one-world curated demo benchmark reaches the baseline gate under
  `demo/orchestrate.py --dds auto`

Known runtime caveats are documented in [demo/README.md](demo/README.md),
especially around WSL2 DDS selection, slow software-rendered Gazebo, and Gazebo
server shutdown occasionally exiting with `-11` after results have already been
written.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `random_gazebo_world/` | Generator package: partitioning, topology, openings, walls, maps, SDF export. |
| `configs/default.yaml` | Default generator config. |
| `tests/` | Unit tests for geometry, topology, map/SDF export, and config behavior. |
| `demo/` | Automated Gazebo + Nav2 benchmark pipeline. |
| `outputs/` | Default local output area for generated worlds. |
| `pyproject.toml` / `uv.lock` | Python project metadata and locked dependencies. |

## Requirements

For generator-only work:

- Python `>=3.12`
- `uv` or an equivalent Python environment manager

Python dependencies are declared in `pyproject.toml`:

- `matplotlib`
- `networkx`
- `numpy`
- `pillow`
- `pyyaml`
- `pytest`
- `shapely`
- `scipy`

For Gazebo/SDF checks and the Nav2 demo:

- ROS 2 Jazzy
- Gazebo Sim 8 from the ROS Jazzy vendor packages
- Nav2 bringup and simple commander
- TurtleBot3 minimal simulation packages
- `colcon`, `ros2`, and `gz` available through the sourced ROS environment

On this WSL2 machine, `gz` is provided by:

```bash
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
```

Use this setup before running Gazebo commands:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
```

## Setup

From the repository root:

```bash
uv sync
```

If you are using the already-created local virtualenv:

```bash
.venv/bin/python --version
```

The project currently targets Python `3.12`.

## Generate A World

Generate with the default config seed:

```bash
uv run python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --out outputs/world_default
```

Generate with an explicit seed:

```bash
uv run python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42
```

Equivalent command using the venv directly:

```bash
.venv/bin/python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42
```

Generate a corridor-centric layout (straight central corridor with rooms on both
sides):

```bash
uv run python -m random_gazebo_world.cli generate \
  --config configs/corridor.yaml \
  --seed 4242 \
  --out outputs/corridor_4242
```

Corridor mode uses `layout_mode: corridor` and packs rooms along a fixed-length
corridor aligned with +X. Each side is tiled independently with rescaled room
widths and jagged per-room depths. Every room gets one fixed-width entrance
opening into the corridor; corridor ends are closed by exterior walls.

The CLI has one command today:

```bash
python -m random_gazebo_world.cli generate --config <yaml> --out <dir> [--seed <int>]
```

## Generated Outputs

A generated world directory contains:

```text
outputs/world_42/
  world.sdf          # Gazebo world: ground, walls, solid fills, lights
  map.png            # Nav2 occupancy image
  map.yaml           # Nav2 map metadata, origin [0, 0, 0]
  nav_task.json      # deterministic start/goal for the demo benchmark
  layout.json        # detailed generated layout geometry
  metadata.json      # seed, config, counts, selected start/goal
  debug/             # staged SVG/PNG debug views
  meshes/            # OBJ meshes for non-orthogonal solid fills, if needed
```

The main Gazebo model structure is:

- `ground`: static box slab with top at `z = 0`
- `walls`: static model containing wall boxes plus solid fill geometry

The exporter now writes static models using the SDF element form:

```xml
<model name="walls">
  <static>true</static>
  ...
</model>
```

This avoids Gazebo warnings from the older `static="true"` attribute form.

## Validate And View In Gazebo

Validate a generated SDF:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH

gz sdf -k outputs/world_42/world.sdf
```

Run it in Gazebo Sim:

```bash
gz sim outputs/world_42/world.sdf
```

The raw generator SDF is static geometry only. For a robot simulation with
physics, sensors, and scene broadcaster plugins, use the demo augmentation step:

```bash
uv run python demo/scripts/augment_world.py outputs/world_42/world.sdf
gz sim outputs/world_42/world_nav.sdf
```

## Generator Pipeline

The pipeline branches on `layout_mode`:

- **`partition`** (default): BSP or Voronoi partitioning, room selection, and
  topology-driven passages.
- **`corridor`**: builds a corridor-centric layout directly, then reuses the
  walls/map/SDF/metadata export stages below.

Shared downstream stages:

1. **Partition / corridor layout**: create cells and roles.
2. **Adjacency graph**: build edges where cells share boundaries.
3. **Room selection**: mark selected cells as rooms (corridor mode selects all
   side rooms).
4. **Candidate connections**: create gate candidates (partition mode also adds
   passage candidates through unused cells).
5. **Room graph selection**: choose a randomized spanning tree plus optional
   loop edges (partition mode only).
6. **Apply connections**: reclassify corridor cells and record logical openings.
7. **Passage constraints**: reject topologies that violate opening constraints
   (partition mode only).
8. **Openings**: place concrete doorway/gate widths on shared boundaries.
9. **Passage geometry**: build straight / L / Z corridor strips in passage cells
   (partition mode; corridor mode uses the full corridor cell polygon).
10. **Solid fills**: convert unused space and passage leftovers into SDF geometry.
11. **Walls**: emit thin wall segments around rooms and passage boundaries.
12. **Map export**: rasterize walkable geometry into a Nav2 map.
13. **Task sampling**: choose deterministic, reachable start/goal poses.
14. **SDF export**: write ground, walls, solids, lighting, and meshes.
15. **Metadata/debug**: write JSON outputs and staged debug images.
16. **Validation/retry**: retry with incremented seeds when generation fails.

## Cell Roles

| Role | Walkable On Map | SDF Geometry |
| --- | --- | --- |
| Room | Full cell interior | Perimeter walls with gate openings |
| Passage | Corridor strips only | Corridor walkable area plus solid leftover fills |
| Unused | No | Full-cell solid fill |

Gates connect two adjacent rooms through their shared boundary. Passages route
through cells that were originally unused. Each passage cell gets one or more
axis-aligned corridor strips between openings; leftover area becomes solid.

## Configuration

See [configs/default.yaml](configs/default.yaml) for partition mode and
[configs/corridor.yaml](configs/corridor.yaml) for corridor mode. Important keys:

| Key | Meaning |
| --- | --- |
| `layout_mode` | `partition` (default) or `corridor`. |
| `world_width`, `world_height` | World extent in metres (partition mode). |
| `corridor_length` | Exact corridor extent along +X (corridor mode). |
| `corridor_width` | Corridor width in metres (corridor mode). |
| `entrance_width` | Fixed room-to-corridor opening width (corridor mode). |
| `room_width_min`, `room_width_max` | Sampled room width range before tiling (corridor mode). |
| `room_depth_min`, `room_depth_max` | Per-room depth range (corridor mode). |
| `partition_method` | `voronoi` or `bsp`. |
| `min_cell_size`, `max_cell_size` | BSP cell size range. |
| `voronoi_seed_count` | Number of Voronoi sites. |
| `voronoi_lloyd_iterations` | Lloyd relaxation iterations. |
| `voronoi_min_cell_area`, `voronoi_max_cell_area` | Accepted Voronoi cell area bounds. |
| `min_room_count`, `max_room_count` | Room count range. |
| `gate_width_min`, `gate_width_max` | Room-to-room opening width range. |
| `passage_width_min`, `passage_width_max` | Passage opening width range. |
| `max_openings_per_passage_edge` | Max openings allowed on one passage edge. |
| `max_open_edges_per_passage` | Max sides of a passage cell that may contain openings. |
| `extra_loop_probability` | Probability of adding extra room-graph loop edges. |
| `map_resolution` | Occupancy map resolution in metres per pixel. |
| `random_seed` | Base seed. Current default is `10667`. |
| `max_attempts` | Top-level generation retry cap. |
| `max_selection_attempts` | Room graph selection retry cap per generation attempt. |
| `wall_height`, `wall_thickness` | Wall dimensions in metres. |
| `ground_thickness` | Ground slab thickness in metres. |
| `textures_enabled` | When `true`, export procedural floor tiles, wall paint, and skirting (default `false`). |
| `floor_tile_size` | Floor tile edge length in metres when textures are enabled (default `0.5`). |

When `textures_enabled` is `true`, SDF export also writes `floor_texture.png` next to
`world.sdf` (64 pixels per metre, seeded from `random_seed`).

## Debug Images

The generator writes staged debug images under `debug/`:

| Stage | Shows |
| --- | --- |
| `01_partition` | Generated cells |
| `02_selected_rooms` | Rooms vs unused cells |
| `03_cell_adjacency_graph` | Cell adjacency graph |
| `04_candidate_connections` | Candidate gates/passages |
| `05_selected_room_graph` | Final selected room connectivity |
| `06_passage_cells` | Cells reclassified as passages |
| `07_openings` | Doorway/gate placements |
| `08_wall_segments` | Generated wall segments |
| `09_occupancy_map_preview` | Nav2 map preview |
| `10_final_floorplan` | Composite floorplan |
| `11_passage_geometry` | Corridor strips and solid leftovers |

## Tests

Run all unit tests:

```bash
.venv/bin/python -m pytest tests -q -s
```

or:

```bash
uv run pytest tests -q -s
```

The `-s` flag avoids capture-related issues seen in this WSL2 desktop session.

Run the package test entrypoint:

```bash
uv run worldgen-test
```

Run focused SDF tests:

```bash
.venv/bin/python -m pytest tests/test_export_sdf.py -q -s
```

## Nav2 Benchmark Demo

The demo lives in [`demo/`](demo/README.md). Quick one-world gate check:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export LIBGL_ALWAYS_SOFTWARE=1

uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --seeds 10 \
  --profiles curated \
  --out /tmp/worldgen_nav_check \
  --timeout 60 \
  --wall-timeout 140 \
  --launch-timeout 60 \
  --headless True \
  --dds auto
```

In the current WSL2 environment, `--dds auto` detects a Fast-CDR ABI mismatch
and switches to CycloneDDS. The one-world gate was verified to pass with:

```text
gate: 1/1 baseline trials passed
```

See [demo/README.md](demo/README.md) for the full benchmark matrix, reports, CI
workflow, and runtime limitations.

## Known Limitations

- The generator targets flat, one-level indoor environments only.
- There is no furniture or semantic object placement yet.
- Corridors are axis-aligned strips inside generated cells.
- Non-orthogonal solid fills are exported as OBJ meshes; orthogonal solids are
  decomposed to boxes where possible.
- The raw `world.sdf` is static geometry. Use `demo/scripts/augment_world.py`
  for Gazebo + Nav2 simulation plugins.
- The Nav2 demo is sensitive to ROS/Gazebo host setup. DDS, software GL, and
  simulator teardown behavior are documented in [demo/README.md](demo/README.md).
