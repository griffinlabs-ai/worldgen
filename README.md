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
  floor_texture.png  # generated floor albedo when textures_enabled (same dir as world.sdf)
  meshes/            # OBJ meshes for non-orthogonal solid fills (when present)
```

Fixture mesh assets are **not** copied into the output directory. Mesh fixture visuals reference
`model://<subdir>/<file>.obj` URIs resolved from `fixture_models_dir` on the Gazebo resource path
(`GZ_SIM_RESOURCE_PATH` / `IGN_GAZEBO_RESOURCE_PATH`). The floor texture is referenced as the bare
filename `floor_texture.png` relative to `world.sdf`; Gazebo resolves it from the output directory
when that directory is on the resource path or when launching from it.

The main Gazebo model structure is:

- `ground`: static box slab with top at `z = 0`
- `walls`: static model containing wall boxes plus solid fill geometry
- one static model per restroom fixture instance and per counter/cabinet box (when `fixture_mode` is enabled)

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
12. **Fixtures** (optional): place restroom clusters and merge cubicle partitions into walls.
13. **Map export**: rasterize walkable geometry into a Nav2 map.
14. **Task sampling**: choose deterministic, reachable start/goal poses.
15. **SDF export**: write ground, walls, solids, fixtures, lighting, and meshes.
16. **Metadata/debug**: write JSON outputs and staged debug images.
17. **Validation/retry**: retry with incremented seeds when generation fails.

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
| `fixture_mode` | `none` (default) or `restroom_clusters` for wall-hugging restroom fixture clusters. |
| `fixture_models_dir` | Directory containing fixture mesh assets; required when `fixture_mode` is not `none`. |
| `fixture_toilet_offset_x/y/z/yaw` | Local mesh visual origin compensation for toilets (metres/radians in the fixture frame; default toilet x is `-0.458`). |
| `fixture_urinal_offset_x/y/z/yaw` | Local mesh visual origin compensation for urinals (default all zero). |
| `fixture_basin_offset_x/y/z/yaw` | Local mesh visual origin compensation for basins (default all zero). |
| `fixture_toilet_count_min`, `fixture_toilet_count_max` | Min/max toilets per toilet cluster (defaults `2` / `5`; pitch `1.5` m). |
| `fixture_urinal_count_min`, `fixture_urinal_count_max` | Min/max urinals per urinal cluster (defaults `2` / `5`; pitch `1.0` m). |
| `fixture_basin_count_min`, `fixture_basin_count_max` | Min/max basins per basin cluster (defaults `1` / `3`; pitch `1.2` m). |
| `cubicle_door_width` | Width of the front-door opening for each toilet cubicle in metres (default `0.65`). Must leave room for a front wall segment at least `wall_thickness` wide within the toilet pitch (`1.5` m). |
| `cubicle_wall_height` | Optional cubicle partition height in metres (default: use global `wall_height`). When set, must be positive and `<= wall_height`. |
| `cubicle_wall_color` | RGB laminate color for cubicle partition/front walls when textures are enabled (default `[0.36, 0.47, 0.55]`). |
| `lighting_mode` | `directional` (default) uses sun/fill lights; `point` places shadowed room lights and spaced corridor lights. |
| `light_height` | Z height in metres for point lights when `lighting_mode: point` (default `2.2`). |
| `corridor_light_spacing` | Spacing in metres for non-shadowed passage/corridor point lights (default `8.0`; at least one light per passage cell). |
| `scene_ambient` | World scene ambient RGB (default `[0.28, 0.28, 0.28]`). |
| `scene_background` | World scene background RGB (default `[0.7, 0.7, 0.7]`). |
| `physics_profile` | `ignored` (default) keeps lightweight ignored physics; `ode` exports Ignition/Gazebo ODE plugins and solver settings. |
| `counter_specular` | RGB specular for counter-top fixture visuals (default `[0.4, 0.4, 0.4]`). |
| `fixture_friction_mu` | ODE friction `mu`/`mu2` for counter and cabinet collisions (default `10000.2`). |

Per-type fixture offsets affect only mesh visual placement in SDF export. They do not
change fixture logical world poses or collision boxes. Map occupancy is stamped from
fixture/box collision footprints (not whole cluster footprints). Cubicle interiors stay
navigable except where partition walls and fixtures occupy space.

When `textures_enabled` is `true`, SDF export also writes `floor_texture.png` next to
`world.sdf` (64 pixels per metre, seeded from `random_seed`). Painted perimeter walls
receive skirting strips; laminate cubicle partitions do not.

When `fixture_mode: restroom_clusters`, each room receives three fixture clusters
(toilet cubicles, urinals, basin bank) placed along distinct walls when possible.
Per-kind cluster counts come from the `fixture_<kind>_count_min/max` keys above.
At config load, each kind's `count_min * pitch` must fit on the largest possible
room wall: in corridor mode `max(room_width_max, room_depth_max) - 2 * wall_thickness`;
in partition mode `max_cell_size - 2 * wall_thickness`. Otherwise validation raises
`ConfigError` with guidance to lower the count or increase room size bounds.
Each toilet cubicle is an enclosed mini-room with side/end partitions and a fixed-width
front door opening (`cubicle_door_width`). Partition walls merge into the wall layout for
map/SDF export; optional `cubicle_wall_height` lowers cubicle partitions in SDF while
leaving room walls at `wall_height`. Fixture mesh visuals use `model://` URIs (see Generated
Outputs above); `fixture_models_dir` must be on the Gazebo resource path. Each mesh fixture model
uses the logical world pose at the model origin; configured per-type offsets are applied only to
the mesh visual in the fixture's local frame. This mode is not supported with
`partition_method: voronoi`.

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
| `12_fixtures` | Restroom clusters with cubicle outlines, door spans, and fixture markers (empty overlay when fixtures disabled) |

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
