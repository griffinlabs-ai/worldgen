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
# 200 passed
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
| `configs/default.yaml` | Partition-mode reference config (BSP / Voronoi). |
| `configs/corridor.yaml` | Corridor-mode reference config (fixtures, textures, point lighting, ODE physics). |
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

## Quickstart: Generate A Corridor World

Install dependencies and generate a corridor-centric world from the reference
config:

```bash
uv sync

uv run python -m random_gazebo_world.cli generate \
  --config configs/corridor.yaml \
  --seed 4242 \
  --out outputs/corridor_4242
```

Equivalent command using the venv directly:

```bash
.venv/bin/python -m random_gazebo_world.cli generate \
  --config configs/corridor.yaml \
  --seed 4242 \
  --out outputs/corridor_4242
```

The command writes a fresh output directory (any existing `--out` path is
removed first). A successful run produces:

```text
outputs/corridor_4242/
  world.sdf
  map.png
  map.yaml
  nav_task.json
  layout.json
  metadata.json
  floor_texture.png       # when textures_enabled
  debug/                  # staged SVG/PNG debug views
  meshes/                 # OBJ meshes for non-orthogonal solids (when present)
```

Validate the SDF before opening it in Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH

gz sdf -k outputs/corridor_4242/world.sdf
```

Preview the world in Gazebo Sim (static geometry only):

```bash
gz sim outputs/corridor_4242/world.sdf
```

The CLI has one command today:

```bash
python -m random_gazebo_world.cli generate --config <yaml> --out <dir> [--seed <int>]
```

When `--seed` is omitted, the config file's `random_seed` is used. The seed
passed on the command line overrides the YAML value.

## Generate A Partition World

Generate with the default partition config:

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

Partition mode uses `layout_mode: partition` (the default) with BSP or Voronoi
cell partitioning, room selection, and topology-driven passages between rooms.

Corridor mode uses `layout_mode: corridor` and packs rooms along a fixed-length
corridor aligned with +X. Each side is tiled independently with rescaled room
widths and jagged per-room depths. Every room gets one fixed-width entrance
opening into the corridor; corridor ends are closed by exterior walls. See
[Tuning Corridor Worlds](#tuning-corridor-worlds) for the corridor-specific keys.

## Tuning Corridor Worlds

[`configs/corridor.yaml`](configs/corridor.yaml) is the reference config for
corridor mode. It enables restroom fixture clusters, procedural floor textures,
point lighting, and an ODE physics profile. Copy it as a starting point and
adjust keys in the groups below.

After each change, regenerate with a fixed `--seed` so layout differences come
from your edits rather than random variation. Use the debug images (see
[Debug Images](#debug-images)) to inspect intermediate stages before loading
the world in Gazebo or TCR.

### Layout and walls

| Key | Effect |
| --- | --- |
| `corridor_length` | Exact corridor extent along +X in metres. |
| `corridor_width` | Corridor width in metres. |
| `entrance_width` | Fixed width of each room-to-corridor opening. Must satisfy `room_width_min >= entrance_width + 2 * wall_thickness`. |
| `room_width_min`, `room_width_max` | Sampled room width range before tiling each corridor side. |
| `room_depth_min`, `room_depth_max` | Per-room depth range (jagged depths along the corridor). |
| `wall_height`, `wall_thickness` | Global wall dimensions in metres. |

Check `debug/01_partition` for room tiling, `debug/07_openings` for entrance
placements, and `debug/08_wall_segments` for the wall layout.

### Fixtures

| Key | Effect |
| --- | --- |
| `fixture_mode` | `none` (default) or `restroom_clusters`. |
| `fixture_models_dir` | Directory of fixture mesh assets; required when `fixture_mode` is not `none`. Meshes are referenced as `model://<subdir>/<file>.obj` URIs on the Gazebo resource path. |
| `fixture_toilet_count_min/max` | Toilets per toilet cluster (pitch `1.5` m). Defaults `2` / `5`. |
| `fixture_urinal_count_min/max` | Urinals per urinal cluster (pitch `1.0` m). Defaults `2` / `5`. |
| `fixture_basin_count_min/max` | Basins per basin cluster (pitch `1.2` m). Defaults `1` / `3`. |
| `fixture_<kind>_offset_x/y/z/yaw` | Local mesh visual origin compensation per fixture kind (metres/radians in the fixture frame). Defaults: toilet x `-0.458`, all others zero. |
| `cubicle_door_width` | Front-door opening width for each toilet cubicle (default `0.65` m). Must leave a front wall segment at least `wall_thickness` wide within the `1.5` m toilet pitch. |
| `cubicle_wall_height` | Optional cubicle partition height (default: use global `wall_height`). When set, must be positive and `<= wall_height`. |
| `counter_specular` | RGB specular for counter-top fixture visuals (default `[0.4, 0.4, 0.4]`). |
| `fixture_friction_mu` | ODE friction `mu`/`mu2` for counter and cabinet collisions (default `10000.2`). |

**Feasibility rule:** at config load, each kind's `count_min * pitch` must fit
on the largest possible room wall span: `max(room_width_max, room_depth_max) -
2 * wall_thickness`. If the range is infeasible, loading the config raises
`ConfigError` with guidance to lower `fixture_<kind>_count_min` or increase
`room_width_max` / `room_depth_max`. This fails fast before generation starts.

Each toilet cubicle is an enclosed mini-room with side/end partitions and a
fixed-width front door. Partition walls merge into the wall layout for map/SDF
export. Map occupancy is stamped from fixture/box collision footprints, not whole
cluster footprints; cubicle interiors stay navigable except where partitions and
fixtures occupy space.

Check `debug/12_fixtures` for cluster placement, cubicle outlines, door spans,
and fixture markers.

### Appearance and physics

| Key | Effect |
| --- | --- |
| `textures_enabled` | When `true`, export procedural floor tiles, wall paint, and skirting (default `false`). Writes `floor_texture.png` next to `world.sdf` (64 pixels per metre, seeded from `random_seed`). |
| `floor_tile_size` | Floor tile edge length in metres (default `0.5`). |
| `cubicle_wall_color` | RGB laminate color for cubicle partitions when textures are enabled (default `[0.36, 0.47, 0.55]`). |
| `lighting_mode` | `directional` (default) uses sun/fill lights; `point` places shadowed room lights and spaced corridor lights. |
| `light_height` | Z height in metres for point lights (default `2.2`). |
| `corridor_light_spacing` | Spacing in metres for non-shadowed passage/corridor point lights (default `8.0`; at least one light per passage cell). |
| `scene_ambient` | World scene ambient RGB (default `[0.28, 0.28, 0.28]`). |
| `scene_background` | World scene background RGB (default `[0.7, 0.7, 0.7]`). |
| `physics_profile` | `ignored` (default) keeps lightweight ignored physics; `ode` exports Ignition/Gazebo ODE plugins and solver settings. Use `ode` for robot simulation in TCR. |

Painted perimeter walls receive skirting strips when textures are enabled;
laminate cubicle partitions do not.

### Seeds and retries

Generation is deterministic for a given base seed and successful attempt index.
The pipeline retries with incremented seeds when a stage fails validation:
attempt `n` uses `random_seed + n`. The winning seed is recorded in
`metadata.json`.

| Key | Effect |
| --- | --- |
| `random_seed` | Base seed (default in [`configs/default.yaml`](configs/default.yaml): `10667`). |
| `max_attempts` | Top-level generation retry cap (default `100000`). |
| `max_selection_attempts` | Room-graph selection retry cap per generation attempt in partition mode only (default `64`). |

If generation exhausts `max_attempts`, the CLI exits with an error. Lower
`fixture_*_count_min` or relax layout bounds if retries cluster on fixture or
corridor layout failures.

## Generated Outputs

A generated world directory contains:

```text
outputs/world_42/
  world.sdf          # Gazebo world: ground, walls, solid fills, lights, fixtures
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
- point or directional lights depending on `lighting_mode`
- ODE physics plugins when `physics_profile: ode`

The exporter writes static models using the SDF element form:

```xml
<model name="walls">
  <static>true</static>
  ...
</model>
```

This avoids Gazebo warnings from the older `static="true"` attribute form. The
default world name in exported SDF is `generated_world`; rename it when
installing into another package (see [Using a Generated World in the TCR Simulation](#using-a-generated-world-in-the-tcr-simulation)).

## Ground-truth Free-space Contract

Stable occupancy-map semantics for AUT-145 and downstream Nav2/benchmark
consumers. Every successful run exports `map.png`, `map.yaml`, and (when the
map stage completes) a matching `metadata.json` `free_space` block through the
shared `export_occupancy_map` path in `random_gazebo_world/export_map.py`.

### `map.png`

- Pixel value **`0`** = occupied (walls, solid fills, fixture collision
  footprints).
- Pixel value **`254`** = free (walkable room and passage interior).
- **Row 0 is the top** of the image (maximum world +Y); columns increase with
  world +X. Matches Nav2 map-server image orientation.

### `map.yaml`

Nav2 map metadata written alongside `map.png`:

| Field | Value |
| --- | --- |
| `image` | `"map.png"` |
| `resolution` | Metres per pixel from config `map_resolution` |
| `origin` | `[0, 0, 0]` (world origin at the bottom-left corner of the map image) |
| `negate` | `0` |
| `occupied_thresh` | `0.65` |
| `free_thresh` | `0.196` |

Grid dimensions in pixels:

- `width = ceil(world_width / resolution)`
- `height = ceil(world_height / resolution)`

(`world_width` / `world_height` come from the generated partition bounds.)

### `metadata.json` → `free_space`

When occupancy is exported, `metadata.json` includes:

```json
"free_space": {
  "free_cells": <int>,
  "free_area_m2": <float>,
  "resolution": <float>,
  "map_image": "map.png"
}
```

`free_area_m2 = free_cells * resolution²`.

### Stability

This contract is the **M6 gate** for `partition_method: bsp`. It is designed
to generalize: `voronoi`, `corridor`, `two_room_gate`, and `two_room_corner`
layouts honor the same export path and should produce maps conforming to these
semantics.

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

Corridor configs with `physics_profile: ode` already include ODE physics and
Ignition/Gazebo system plugins and can be used directly in TCR without
augmentation.

## Using a Generated World in the TCR Simulation

This workflow installs a generated corridor world into the TCR Ignition stack so
the robot can be spawned and navigated in it.

### 1. Copy the world SDF

Copy the generated world file into the TCR worlds directory and rename it:

```bash
cp outputs/corridor_4242/world.sdf \
  ~/tcr/ros_ws/src/utils/tcr_ignition/worlds/my_world.sdf
```

Existing examples in that folder include `corridor_1.sdf` and `gen_42.sdf`.

### 2. Match the world name to the filename

Edit the copied SDF so the `<world>` name matches the filename without
`.sdf`:

```xml
<!-- before -->
<world name="generated_world">

<!-- after -->
<world name="my_world">
```

The launch file resolves worlds as `<world_name>.sdf` under the package
`worlds/` directory.

### 3. Copy the floor texture

When `textures_enabled` is `true`, copy the texture next to the SDF:

```bash
cp outputs/corridor_4242/floor_texture.png \
  ~/tcr/ros_ws/src/utils/tcr_ignition/worlds/
```

The SDF references it by relative filename:

```xml
<albedo_map>floor_texture.png</albedo_map>
```

**Caveat:** if multiple generated worlds with different textures coexist in
`worlds/`, rename each texture (for example `my_world_floor.png`) and update
the `<albedo_map>` in the corresponding SDF to match. A single shared
`floor_texture.png` will be overwritten by the last copy.

### 4. Fixture meshes on the resource path

Fixture mesh visuals use `model://` URIs. The TCR Ignition launch file adds the
package `models/` directory to `GZ_SIM_RESOURCE_PATH` (and
`IGN_GAZEBO_RESOURCE_PATH`). Set `fixture_models_dir` in your config to that
models directory, for example:

```yaml
fixture_models_dir: /home/griffinlabs/tcr/ros_ws/src/utils/tcr_ignition/models
```

No extra resource-path setup is needed when launching through TCR.

### 5. Point the tmuxp sim config at the new world

Edit `/home/griffinlabs/tcr/ros_ws/tmuxp/tcr_sim.yaml`. In
the **Robot Bringup** pane, change the `world:=` argument to the new world
name (filename without `.sdf`):

```yaml
ros2 launch tcr_ignition t1_v2_ignition_full.launch.py world:=my_world
x:=1.0 y:=5.0 z:=0.1 yaw:=1.57
headless:="${SIM_HEADLESS:-false}" gui:="${SIM_GUI:-true}" use_ground_truth_odom:=true
```

### 6. Choose a robot spawn pose

Use generator outputs to pick reachable coordinates:

- **`nav_task.json`** — contains a sampled reachable `start` pose
  (`x`, `y`, `yaw`) and a matching `goal`.
- **`debug/09_occupancy_map_preview.png`** — Nav2 occupancy grid preview; white
  regions are free space in map coordinates (origin `[0, 0, 0]`).
- **`debug/10_final_floorplan`** — composite floorplan with the same coordinate
  frame as the world.

Set `z:=0.1` for the robot spawn height. Adjust `x`, `y`, and `yaw` on the
launch line to a free corridor or room location away from fixtures and walls.

Example from `outputs/corridor_4242/nav_task.json`:

```json
"start": { "x": 3.725, "y": 4.825, "yaw": 0.535 }
```

### 7. Rebuild and launch

Recompile `tcr_ignition` so the new world is installed, re-source the
workspace, then start the sim:

```bash
cd ~/tcr/ros_ws
colcon build --packages-select tcr_ignition
source install/setup.bash

cd ~/tcr
ros_ws/bash/start_sim_tmuxp.sh
```

Use `--detached` to load the tmuxp session in the background.

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
13. **Map export**: rasterize walkable geometry into a Nav2 map and write
    `debug/09_occupancy_map_preview.png`.
14. **Task sampling**: choose deterministic, reachable start/goal poses.
15. **SDF export**: write ground, walls, solids, fixtures, lighting, and meshes.
16. **Metadata/debug**: write JSON outputs and remaining staged debug images.
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
[configs/corridor.yaml](configs/corridor.yaml) for corridor mode.

In corridor mode, partition-specific keys (`world_width`, `min_cell_size`, and
so on) are optional and receive built-in defaults if omitted. Corridor-specific
keys (`corridor_length`, `room_width_min`, and so on) are required.

All keys on `Config` with their defaults:

| Key | Default | Meaning |
| --- | --- | --- |
| `layout_mode` | `partition` | `partition` or `corridor`. |
| `world_width` | `1.0` in corridor mode; required in partition | World extent along X (partition mode). |
| `world_height` | `1.0` in corridor mode; required in partition | World extent along Y (partition mode). |
| `corridor_length` | — | Exact corridor extent along +X (corridor mode, required). |
| `corridor_width` | — | Corridor width in metres (corridor mode, required). |
| `entrance_width` | — | Fixed room-to-corridor opening width (corridor mode, required). |
| `room_width_min`, `room_width_max` | — | Sampled room width range before tiling (corridor mode, required). |
| `room_depth_min`, `room_depth_max` | — | Per-room depth range (corridor mode, required). |
| `partition_method` | `bsp` | `voronoi` or `bsp` (partition mode). |
| `passage_geometry_mode` | `curved` | `curved` or `legacy_orthogonal` (partition mode; `legacy_orthogonal` requires `partition_method: bsp`). |
| `min_cell_size`, `max_cell_size` | `1.0` / `10.0` in corridor; required in partition | BSP cell size range. |
| `voronoi_seed_count` | `16` | Number of Voronoi sites. |
| `voronoi_lloyd_iterations` | `8` | Lloyd relaxation iterations. |
| `voronoi_min_cell_area`, `voronoi_max_cell_area` | `1.0` / `64.0` | Accepted Voronoi cell area bounds. |
| `min_room_count`, `max_room_count` | `1` / `100` in corridor; required in partition | Room count range. |
| `gate_width_min`, `gate_width_max` | `entrance_width` in corridor; required in partition | Room-to-room opening width range. |
| `passage_width_min`, `passage_width_max` | `1.0` in corridor; required in partition | Passage opening width range. |
| `max_openings_per_passage_edge` | `1` | Max openings allowed on one passage edge. |
| `max_open_edges_per_passage` | `4` | Max sides of a passage cell that may contain openings. |
| `extra_loop_probability` | `0.0` in corridor; required in partition | Probability of adding extra room-graph loop edges. |
| `map_resolution` | — | Occupancy map resolution in metres per pixel (required). |
| `random_seed` | — | Base seed (required). |
| `max_attempts` | `100000` | Top-level generation retry cap. |
| `max_selection_attempts` | `64` | Room graph selection retry cap per attempt (partition mode). |
| `wall_height`, `wall_thickness` | — | Wall dimensions in metres (required). |
| `ground_thickness` | `0.1` | Ground slab thickness in metres. |
| `textures_enabled` | `false` | Procedural floor tiles, wall paint, and skirting. |
| `floor_tile_size` | `0.5` | Floor tile edge length in metres when textures are enabled. |
| `fixture_mode` | `none` | `none` or `restroom_clusters`. |
| `fixture_models_dir` | — | Directory containing fixture mesh assets; required when `fixture_mode` is not `none`. |
| `fixture_toilet_offset_x/y/z/yaw` | `-0.458` / `0` / `0` / `0` | Local mesh visual origin compensation for toilets. |
| `fixture_urinal_offset_x/y/z/yaw` | all `0` | Local mesh visual origin compensation for urinals. |
| `fixture_basin_offset_x/y/z/yaw` | all `0` | Local mesh visual origin compensation for basins. |
| `fixture_toilet_count_min`, `fixture_toilet_count_max` | `2` / `5` | Min/max toilets per toilet cluster (pitch `1.5` m). |
| `fixture_urinal_count_min`, `fixture_urinal_count_max` | `2` / `5` | Min/max urinals per urinal cluster (pitch `1.0` m). |
| `fixture_basin_count_min`, `fixture_basin_count_max` | `1` / `3` | Min/max basins per basin cluster (pitch `1.2` m). |
| `cubicle_door_width` | `0.65` | Width of the front-door opening for each toilet cubicle in metres. |
| `cubicle_wall_height` | — | Optional cubicle partition height (default: use global `wall_height`). |
| `cubicle_wall_color` | `[0.36, 0.47, 0.55]` | RGB laminate color for cubicle partitions when textures are enabled. |
| `lighting_mode` | `directional` | `directional` (sun/fill) or `point` (room and corridor point lights). |
| `light_height` | `2.2` | Z height in metres for point lights. |
| `corridor_light_spacing` | `8.0` | Spacing in metres for non-shadowed corridor point lights. |
| `scene_ambient` | `[0.28, 0.28, 0.28]` | World scene ambient RGB. |
| `scene_background` | `[0.7, 0.7, 0.7]` | World scene background RGB. |
| `physics_profile` | `ignored` | `ignored` or `ode` (ODE plugins and solver settings). |
| `counter_specular` | `[0.4, 0.4, 0.4]` | RGB specular for counter-top fixture visuals. |
| `fixture_friction_mu` | `10000.2` | ODE friction `mu`/`mu2` for counter and cabinet collisions. |

Per-type fixture offsets affect only mesh visual placement in SDF export. They do not
change fixture logical world poses or collision boxes.

When `fixture_mode: restroom_clusters`, each room receives three fixture clusters
(toilet cubicles, urinals, basin bank) placed along distinct walls when possible.
This mode is not supported with `partition_method: voronoi`.

## Debug Images

The generator writes staged debug images under `debug/`. All stages except
`09_occupancy_map_preview` produce both `.svg` and `.png`; stage 09 is PNG only.

| Stage | Shows | Tuning use |
| --- | --- | --- |
| `01_partition` | Generated cells | Corridor room tiling and cell boundaries. |
| `02_selected_rooms` | Rooms vs unused cells | Which cells become navigable rooms. |
| `03_cell_adjacency_graph` | Cell adjacency graph | Topology before connection selection. |
| `04_candidate_connections` | Candidate gates/passages | Available gate and passage options (partition mode). |
| `05_selected_room_graph` | Final selected room connectivity | Chosen room graph (partition mode). |
| `06_passage_cells` | Cells reclassified as passages | Passage routing after connection apply. |
| `07_openings` | Doorway/gate placements | Entrance and gate widths on shared boundaries. |
| `08_wall_segments` | Generated wall segments | Wall layout before fixtures and solids. |
| `09_occupancy_map_preview` | Nav2 map preview (PNG only) | Free space for Nav2 and robot spawn selection. |
| `10_final_floorplan` | Composite floorplan | Overall layout; pick spawn coordinates in world frame. |
| `11_passage_geometry` | Corridor strips and solid leftovers | Passage strip geometry and solid fills. |
| `12_fixtures` | Restroom clusters, cubicle outlines, fixture markers | Fixture cluster placement (empty overlay when fixtures disabled). |

Typical corridor tuning loop:

1. Adjust layout or fixture keys in your YAML.
2. Regenerate with a fixed `--seed`.
3. Check `12_fixtures` for cluster placement and cubicle doors.
4. Check `09_occupancy_map_preview` and `10_final_floorplan` for free-space
   coordinates, then set TCR spawn `x`, `y`, and `yaw` or use `nav_task.json`
   start values directly.

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
- General furniture or semantic object placement beyond restroom fixture clusters
  is not supported.
- Corridors are axis-aligned strips inside generated cells.
- Non-orthogonal solid fills are exported as OBJ meshes; orthogonal solids are
  decomposed to boxes where possible.
- The raw `world.sdf` with `physics_profile: ignored` is static geometry without
  simulation plugins. Use `physics_profile: ode` for TCR, or
  `demo/scripts/augment_world.py` for the Nav2 demo pipeline.
- The Nav2 demo is sensitive to ROS/Gazebo host setup. DDS, software GL, and
  simulator teardown behavior are documented in [demo/README.md](demo/README.md).
