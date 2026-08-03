from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from shapely import contains_xy
from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from random_gazebo_world.config import Config
from random_gazebo_world.fixtures import FixtureLayout, fixture_collision_footprint, box_collision_footprint
from random_gazebo_world.topology import CellRole
from random_gazebo_world.walls import WallLayout, WallSegment


class OccupancyMapError(RuntimeError):
    """Raised when occupancy map generation or validation fails."""


FREE_VALUE = 254
OCCUPIED_VALUE = 0


@dataclass(frozen=True)
class OccupancyMap:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    world_width: float
    world_height: float
    start_cell: tuple[int, int]
    goal_cell: tuple[int, int]
    free_cell_count: int
    free_area_m2: float
    start_yaw_offset: float = 0.0
    goal_yaw_offset: float = 0.0

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        return int(self.data.shape[0])


def export_occupancy_map(
    wall_layout: WallLayout,
    config: Config,
    output_dir: Path,
    rng,
) -> OccupancyMap:
    occupancy = generate_occupancy_map(wall_layout, config, rng)
    write_occupancy_map_files(occupancy, output_dir)
    return occupancy


def write_occupancy_map_files(occupancy: OccupancyMap, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    preview_path = output_dir / "debug" / "09_occupancy_map_preview.png"
    preview_path.parent.mkdir(parents=True, exist_ok=True)

    Image.fromarray(occupancy.data).save(output_dir / "map.png")
    _write_map_yaml(output_dir / "map.yaml", occupancy)
    _write_preview(preview_path, occupancy)
    validate_occupancy_map(occupancy)


def cell_to_world(
    cell: tuple[int, int], occupancy: OccupancyMap
) -> tuple[float, float]:
    """Convert an occupancy (row, col) cell to a world-frame (x, y) point in metres."""
    row, col = cell
    return _grid_cell_center(
        col,
        row,
        occupancy.resolution,
        occupancy.origin_x,
        occupancy.origin_y,
        occupancy.height,
    )


def build_nav_task(occupancy: OccupancyMap) -> dict:
    """Build a navigation task (start/goal world poses) from a generated map.

    The start and goal cells are guaranteed-free, mutually reachable cells sampled
    during occupancy map generation, so the returned poses are deterministic for a
    given seed and always lie in navigable space.
    """
    start_x, start_y = cell_to_world(occupancy.start_cell, occupancy)
    goal_x, goal_y = cell_to_world(occupancy.goal_cell, occupancy)
    heading = math.atan2(goal_y - start_y, goal_x - start_x)
    return {
        "frame_id": "map",
        "start": {
            "x": start_x,
            "y": start_y,
            "yaw": heading + occupancy.start_yaw_offset,
        },
        "goal": {
            "x": goal_x,
            "y": goal_y,
            "yaw": heading + occupancy.goal_yaw_offset,
        },
        "map": {
            "resolution": occupancy.resolution,
            "origin": [occupancy.origin_x, occupancy.origin_y, 0.0],
            "width": occupancy.width,
            "height": occupancy.height,
        },
    }


def export_nav_task_json(path: Path, occupancy: OccupancyMap) -> dict:
    """Write the navigation task (start/goal poses) to ``nav_task.json``."""
    task = build_nav_task(occupancy)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(task, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return task


def generate_occupancy_map(
    wall_layout: WallLayout,
    config: Config,
    rng,
    *,
    fixture_layout: FixtureLayout | None = None,
    pinned_start_goal_world: tuple[tuple[float, float], tuple[float, float]] | None = None,
    start_yaw_offset: float = 0.0,
    goal_yaw_offset: float = 0.0,
) -> OccupancyMap:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    resolution = config.map_resolution
    origin_x = 0.0
    origin_y = 0.0

    width = max(1, math.ceil(partition.world_width / resolution))
    height = max(1, math.ceil(partition.world_height / resolution))
    grid = np.full((height, width), OCCUPIED_VALUE, dtype=np.uint8)

    grid_x, grid_y = _pixel_center_grid(
        width, height, resolution, origin_x, origin_y
    )

    passage_geometry = wall_layout.passage_geometry
    free_geometries: list[BaseGeometry] = []
    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role is CellRole.ROOM:
            free_geometries.append(cell.polygon)
        elif role is CellRole.PASSAGE:
            corridor = (
                passage_geometry.corridor_for(cell.id)
                if passage_geometry is not None
                else None
            )
            if corridor is not None and not corridor.is_empty:
                free_geometries.append(corridor)
            else:
                free_geometries.append(cell.polygon)

    if free_geometries:
        free_geom = unary_union(free_geometries)
        grid[contains_xy(free_geom, grid_x, grid_y)] = FREE_VALUE

    # "Solid" fill polygons (unused Voronoi cells and passage solids) are real
    # obstacles. The demo's augment_world step rasterizes them into collision +
    # visual boxes so they actually collide AND are seen by the lidar in Gazebo,
    # keeping the occupancy map consistent with the simulator. Mark them occupied
    # here together with the wall-segment boxes.
    occupied_geometries: list[BaseGeometry] = list(wall_layout.unused_solids)
    if wall_layout.passage_geometry is not None:
        occupied_geometries.extend(wall_layout.passage_geometry.solids)
    half_thickness = config.wall_thickness / 2.0
    for segment in wall_layout.segments:
        occupied_geometries.append(_segment_polygon(segment, half_thickness))

    if fixture_layout is not None:
        for instance in fixture_layout.instances:
            occupied_geometries.append(fixture_collision_footprint(instance))
        for box in fixture_layout.boxes:
            occupied_geometries.append(box_collision_footprint(box))

    if occupied_geometries:
        occupied_geom = unary_union(occupied_geometries)
        grid[contains_xy(occupied_geom, grid_x, grid_y)] = OCCUPIED_VALUE

    if pinned_start_goal_world is not None:
        start_cell, goal_cell = _resolve_pinned_start_goal_cells(
            grid,
            layout,
            resolution,
            origin_x,
            origin_y,
            pinned_start_goal_world,
        )
    else:
        start_cell, goal_cell = _sample_start_goal_cells(
            grid,
            layout,
            resolution,
            origin_x,
            origin_y,
            rng,
        )
    free_cell_count = int(np.sum(grid == FREE_VALUE))
    free_area_m2 = free_cell_count * resolution * resolution
    occupancy = OccupancyMap(
        data=grid,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        world_width=partition.world_width,
        world_height=partition.world_height,
        start_cell=start_cell,
        goal_cell=goal_cell,
        free_cell_count=free_cell_count,
        free_area_m2=free_area_m2,
        start_yaw_offset=start_yaw_offset,
        goal_yaw_offset=goal_yaw_offset,
    )
    validate_occupancy_map(occupancy)
    return occupancy


def validate_occupancy_map(occupancy: OccupancyMap) -> None:
    free_cells = _free_cell_coordinates(occupancy.data)
    if not free_cells:
        raise OccupancyMapError("Occupancy map contains no free space")

    components = _connected_components(free_cells)
    if len(components) != 1:
        raise OccupancyMapError(
            f"Free space has {len(components)} connected components, expected 1"
        )

    if not _is_free(occupancy.data, occupancy.start_cell):
        raise OccupancyMapError("Sampled start lies outside free space")
    if not _is_free(occupancy.data, occupancy.goal_cell):
        raise OccupancyMapError("Sampled goal lies outside free space")
    if not _cells_reachable(occupancy.data, occupancy.start_cell, occupancy.goal_cell):
        raise OccupancyMapError("Sampled start and goal are not reachable")


def _write_map_yaml(path: Path, occupancy: OccupancyMap) -> None:
    payload = {
        "image": "map.png",
        "resolution": occupancy.resolution,
        "origin": [occupancy.origin_x, occupancy.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def _write_preview(path: Path, occupancy: OccupancyMap) -> None:
    preview = np.stack([occupancy.data, occupancy.data, occupancy.data], axis=-1)
    Image.fromarray(preview).save(path)


def _pixel_center_grid(
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[np.ndarray, np.ndarray]:
    cols = np.arange(width)
    rows = np.arange(height)
    world_x = origin_x + (cols + 0.5) * resolution
    world_y = origin_y + (height - 1 - rows + 0.5) * resolution
    return np.meshgrid(world_x, world_y)


def _segment_polygon(segment: WallSegment, half_thickness: float) -> Polygon:
    line = LineString([segment.p1, segment.p2])
    return line.buffer(half_thickness, cap_style="flat", join_style="mitre")


def _grid_cell_center(
    col: int,
    row: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    height: int,
) -> tuple[float, float]:
    world_x = origin_x + (col + 0.5) * resolution
    world_y = origin_y + (height - 1 - row + 0.5) * resolution
    return world_x, world_y


def _free_cell_coordinates(grid: np.ndarray) -> list[tuple[int, int]]:
    rows, cols = np.where(grid == FREE_VALUE)
    return list(zip(rows.tolist(), cols.tolist(), strict=True))


def _connected_components(cells: list[tuple[int, int]]) -> list[set[tuple[int, int]]]:
    remaining = set(cells)
    components: list[set[tuple[int, int]]] = []

    while remaining:
        start = remaining.pop()
        component = {start}
        queue: deque[tuple[int, int]] = deque([start])

        while queue:
            row, col = queue.popleft()
            for neighbor in (
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)

        components.append(component)

    return components


def _is_free(grid: np.ndarray, cell: tuple[int, int]) -> bool:
    row, col = cell
    if row < 0 or col < 0 or row >= grid.shape[0] or col >= grid.shape[1]:
        return False
    return int(grid[row, col]) == FREE_VALUE


def _cells_reachable(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> bool:
    if start == goal:
        return True

    visited = {start}
    queue: deque[tuple[int, int]] = deque([start])

    while queue:
        row, col = queue.popleft()
        for neighbor in (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ):
            if neighbor in visited:
                continue
            if not _is_free(grid, neighbor):
                continue
            if neighbor == goal:
                return True
            visited.add(neighbor)
            queue.append(neighbor)

    return False


def _world_to_cell(
    world_x: float,
    world_y: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    height: int,
) -> tuple[int, int]:
    col = int(round((world_x - origin_x) / resolution - 0.5))
    row = int(round(height - 1 - (world_y - origin_y) / resolution + 0.5))
    return row, col


def _nearest_free_cell(
    grid: np.ndarray,
    target: tuple[int, int],
    *,
    candidates: list[tuple[int, int]] | None = None,
) -> tuple[int, int]:
    if candidates is None:
        candidates = _free_cell_coordinates(grid)
    if not candidates:
        raise OccupancyMapError("No free cells available for pinned start/goal")

    best = min(
        candidates,
        key=lambda cell: (
            (cell[0] - target[0]) ** 2 + (cell[1] - target[1]) ** 2,
            cell[0],
            cell[1],
        ),
    )
    return best


def _resolve_pinned_start_goal_cells(
    grid: np.ndarray,
    layout,
    resolution: float,
    origin_x: float,
    origin_y: float,
    pinned_start_goal_world: tuple[tuple[float, float], tuple[float, float]],
) -> tuple[tuple[int, int], tuple[int, int]]:
    height = grid.shape[0]
    room_ids = sorted(layout.room_selection.room_cell_ids)
    if len(room_ids) < 2:
        raise OccupancyMapError(
            "Pinned start/goal requires at least two selected rooms"
        )

    resolved: list[tuple[int, int]] = []
    for index, (world_x, world_y) in enumerate(pinned_start_goal_world):
        target = _world_to_cell(
            world_x,
            world_y,
            resolution,
            origin_x,
            origin_y,
            height,
        )
        room_id = room_ids[index]
        room_cells = _free_cells_for_room(
            grid,
            layout,
            room_id,
            resolution,
            origin_x,
            origin_y,
        )
        if not room_cells:
            raise OccupancyMapError(
                f"No free cells found in room {room_id} for pinned pose"
            )
        cell = (
            target
            if _is_free(grid, target) and target in room_cells
            else _nearest_free_cell(grid, target, candidates=room_cells)
        )
        resolved.append(cell)

    start_cell, goal_cell = resolved[0], resolved[1]
    if not _is_free(grid, start_cell) or not _is_free(grid, goal_cell):
        raise OccupancyMapError("Pinned start/goal resolved outside free space")
    if not _cells_reachable(grid, start_cell, goal_cell):
        raise OccupancyMapError("Pinned start and goal are not reachable")
    return start_cell, goal_cell


def _sample_start_goal_cells(
    grid: np.ndarray,
    layout,
    resolution: float,
    origin_x: float,
    origin_y: float,
    rng,
) -> tuple[tuple[int, int], tuple[int, int]]:
    room_ids = sorted(layout.room_selection.room_cell_ids)
    if len(room_ids) < 2:
        free_cells = _free_cell_coordinates(grid)
        if len(free_cells) < 2:
            raise OccupancyMapError("Not enough free cells to sample start and goal")
        return free_cells[0], free_cells[-1]

    # Pick the most interior (highest obstacle clearance) cell of each room so the
    # spawn and goal sit well clear of walls and stay reachable once Nav2 inflates
    # the costmap. Selection is fully deterministic for reproducibility.
    clearance = _obstacle_clearance(grid)

    def _most_clear(cells: list[tuple[int, int]]) -> tuple[int, int]:
        return max(cells, key=lambda rc: (clearance[rc[0], rc[1]], -rc[0], -rc[1]))

    room_centers: list[tuple[int, int]] = []
    for room_id in room_ids:
        cells = _free_cells_for_room(
            grid, layout, room_id, resolution, origin_x, origin_y
        )
        if cells:
            room_centers.append(_most_clear(cells))

    if len(room_centers) < 2:
        raise OccupancyMapError("Could not find free cells in selected rooms")

    return _pick_start_goal_pair(grid, room_centers, resolution)


def _obstacle_clearance(grid: np.ndarray) -> np.ndarray:
    """Distance (in cells) from each free cell to the nearest non-free cell."""
    from scipy.ndimage import distance_transform_edt

    return distance_transform_edt(grid == FREE_VALUE)


def _pick_start_goal_pair(
    grid: np.ndarray,
    candidates: list[tuple[int, int]],
    resolution: float,
    *,
    max_detour_ratio: float = 2.2,
    clearance_cells: int = 10,
) -> tuple[tuple[int, int], tuple[int, int]]:
    """Choose a start/goal pair that is far apart yet connected by a direct route.

    The goal is a navigation task that is *meaningful* (large straight-line
    separation) but not a degenerate maze: among all candidate room centers we
    prefer the most-separated pair whose shortest navigable path stays within
    ``max_detour_ratio`` of the straight-line distance. If no pair satisfies the
    detour bound we fall back to the most direct reachable pair, and finally to
    the most-separated pair regardless of routing.
    """
    from scipy.ndimage import binary_erosion

    free = grid == FREE_VALUE
    # Erode by an approximate robot clearance so BFS distances reflect a path a
    # real footprint can follow rather than threading single-cell gaps.
    structure = np.ones((2 * clearance_cells + 1, 2 * clearance_cells + 1), bool)
    navigable = binary_erosion(free, structure=structure)
    # Keep candidates navigable; if a center eroded away, snap to nearest nav cell.
    nav_candidates = [_nearest_navigable(navigable, c) for c in candidates]

    best_feasible: tuple[float, tuple, tuple] | None = None
    best_direct: tuple[float, tuple, tuple] | None = None
    best_any: tuple[float, tuple, tuple] | None = None

    dist_cache: dict[int, np.ndarray] = {}
    for i in range(len(nav_candidates)):
        src = nav_candidates[i]
        if src is None:
            continue
        if i not in dist_cache:
            dist_cache[i] = _bfs_distance(navigable, src)
        dmap = dist_cache[i]
        for j in range(i + 1, len(nav_candidates)):
            dst = nav_candidates[j]
            if dst is None:
                continue
            path_cells = dmap[dst[0], dst[1]]
            straight = math.hypot(src[0] - dst[0], src[1] - dst[1])
            if straight <= 0:
                continue
            straight_m = straight * resolution
            if best_any is None or straight_m > best_any[0]:
                best_any = (straight_m, src, dst)
            if path_cells < 0:
                continue
            detour = (path_cells * resolution) / straight_m
            if detour <= max_detour_ratio:
                if best_feasible is None or straight_m > best_feasible[0]:
                    best_feasible = (straight_m, src, dst)
            if best_direct is None or detour < best_direct[0]:
                best_direct = (detour, src, dst)

    chosen = best_feasible or best_direct or best_any
    if chosen is None:
        raise OccupancyMapError("Could not find a connected start/goal pair")
    return chosen[1], chosen[2]


def _nearest_navigable(
    navigable: np.ndarray, cell: tuple[int, int]
) -> tuple[int, int] | None:
    """Return ``cell`` if navigable, else the closest navigable cell via BFS."""
    if navigable[cell[0], cell[1]]:
        return cell
    height, width = navigable.shape
    seen = np.zeros_like(navigable, dtype=bool)
    queue = deque([cell])
    seen[cell[0], cell[1]] = True
    while queue:
        row, col = queue.popleft()
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n_row, n_col = row + d_row, col + d_col
            if 0 <= n_row < height and 0 <= n_col < width and not seen[n_row, n_col]:
                if navigable[n_row, n_col]:
                    return (n_row, n_col)
                seen[n_row, n_col] = True
                queue.append((n_row, n_col))
    return None


def _bfs_distance(
    navigable: np.ndarray, source: tuple[int, int]
) -> np.ndarray:
    """8-connected BFS shortest-path distance (in cells) from ``source``."""
    height, width = navigable.shape
    dist = np.full((height, width), -1.0)
    dist[source[0], source[1]] = 0.0
    queue = deque([source])
    neighbors = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, 1.41421356),
        (1, -1, 1.41421356),
        (-1, 1, 1.41421356),
        (-1, -1, 1.41421356),
    )
    while queue:
        row, col = queue.popleft()
        base = dist[row, col]
        for d_row, d_col, step in neighbors:
            n_row, n_col = row + d_row, col + d_col
            if (
                0 <= n_row < height
                and 0 <= n_col < width
                and navigable[n_row, n_col]
                and dist[n_row, n_col] < 0
            ):
                dist[n_row, n_col] = base + step
                queue.append((n_row, n_col))
    return dist


def _free_cells_for_room(
    grid: np.ndarray,
    layout,
    room_id: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> list[tuple[int, int]]:
    cell = next(item for item in layout.partition.cells if item.id == room_id)
    polygon = cell.polygon
    cells: list[tuple[int, int]] = []
    height, width = grid.shape

    for row in range(height):
        for col in range(width):
            if int(grid[row, col]) != FREE_VALUE:
                continue
            world_x, world_y = _grid_cell_center(
                col, row, resolution, origin_x, origin_y, height
            )
            if contains_xy(polygon, world_x, world_y):
                cells.append((row, col))

    return cells
