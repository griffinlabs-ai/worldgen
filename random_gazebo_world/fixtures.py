from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

from shapely.geometry import Polygon

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, Edge, Vec2
from random_gazebo_world.openings import Opening
from random_gazebo_world.topology import CellRole
from random_gazebo_world.walls import WallLayout, WallSegment

FixtureKind = Literal["toilet", "urinal", "basin"]
BoxColorKey = Literal["counter", "cabinet"]

ENTRANCE_ALONG_WALL_CLEARANCE = 0.5
ENTRANCE_INTO_ROOM_CLEARANCE = 1.0
AISLE_GAP = 0.6
MAX_PLACEMENT_ATTEMPTS = 20
FIXTURE_YAW_OFFSET = math.pi / 2.0

COUNTER_TOP = (0.45, 0.45, 0.48)
CABINET_WOOD = (0.4, 0.3, 0.2)
LAMINATE = (0.36, 0.47, 0.55)


class FixtureError(RuntimeError):
    """Raised when fixture placement fails and should trigger a seed retry."""


@dataclass(frozen=True)
class FixtureSpec:
    kind: FixtureKind
    mesh_relpath: str
    collision_size: tuple[float, float, float]
    z_offset: float
    pitch: float
    cluster_depth: float
    count_min: int
    count_max: int


FIXTURE_SPECS: dict[FixtureKind, FixtureSpec] = {
    "toilet": FixtureSpec(
        kind="toilet",
        mesh_relpath="open_toilet/scale_down_toilet_bowl.obj",
        collision_size=(0.2, 0.4, 0.4),
        z_offset=0.5,
        pitch=1.5,
        cluster_depth=1.5,
        count_min=2,
        count_max=5,
    ),
    "urinal": FixtureSpec(
        kind="urinal",
        mesh_relpath=(
            "Sanitary_Urinals_DURAVIT-AG_ME-by-Starck-urinal-280930/"
            "basin_scale_down.obj"
        ),
        collision_size=(0.42, 0.30, 0.51),
        z_offset=0.436508,
        pitch=1.0,
        cluster_depth=0.28,
        count_min=2,
        count_max=5,
    ),
    "basin": FixtureSpec(
        kind="basin",
        mesh_relpath=(
            "Sanitary_Basins_Roca_DIVERTA-600-Over-countertop-basin/"
            "basin_scale_down.obj"
        ),
        collision_size=(0.4, 0.3, 0.075),
        z_offset=0.825,
        pitch=1.2,
        cluster_depth=0.6,
        count_min=1,
        count_max=3,
    ),
}


@dataclass(frozen=True)
class FixtureVisualOffset:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class FixtureInstance:
    name: str
    kind: FixtureKind
    room_id: int
    x: float
    y: float
    z: float
    yaw: float
    collision_size: tuple[float, float, float]
    mesh_relpath: str
    visual_offset: FixtureVisualOffset


def fixture_count_range_for_kind(config: Config, kind: FixtureKind) -> tuple[int, int]:
    if kind == "toilet":
        return config.fixture_toilet_count_min, config.fixture_toilet_count_max
    if kind == "urinal":
        return config.fixture_urinal_count_min, config.fixture_urinal_count_max
    return config.fixture_basin_count_min, config.fixture_basin_count_max


def fixture_visual_offset_for_kind(config: Config, kind: FixtureKind) -> FixtureVisualOffset:
    if kind == "toilet":
        return FixtureVisualOffset(
            x=config.fixture_toilet_offset_x,
            y=config.fixture_toilet_offset_y,
            z=config.fixture_toilet_offset_z,
            yaw=config.fixture_toilet_offset_yaw,
        )
    if kind == "urinal":
        return FixtureVisualOffset(
            x=config.fixture_urinal_offset_x,
            y=config.fixture_urinal_offset_y,
            z=config.fixture_urinal_offset_z,
            yaw=config.fixture_urinal_offset_yaw,
        )
    return FixtureVisualOffset(
        x=config.fixture_basin_offset_x,
        y=config.fixture_basin_offset_y,
        z=config.fixture_basin_offset_z,
        yaw=config.fixture_basin_offset_yaw,
    )


DEFAULT_FIXTURE_VISUAL_OFFSETS: dict[FixtureKind, FixtureVisualOffset] = {
    "toilet": FixtureVisualOffset(x=-0.458, y=0.0, z=0.0, yaw=0.0),
    "urinal": FixtureVisualOffset(x=0.0, y=0.0, z=0.0, yaw=0.0),
    "basin": FixtureVisualOffset(x=0.0, y=0.0, z=0.0, yaw=0.0),
}


@dataclass(frozen=True)
class CubicleDoorSpan:
    p1: Vec2
    p2: Vec2


@dataclass(frozen=True)
class CubicleLayout:
    index: int
    name: str
    room_id: int
    cluster_name: str
    polygon: Polygon
    door_span: CubicleDoorSpan
    toilet_instance_name: str


@dataclass(frozen=True)
class BoxFixture:
    name: str
    room_id: int
    x: float
    y: float
    z: float
    yaw: float
    size_x: float
    size_y: float
    size_z: float
    color_key: BoxColorKey


@dataclass(frozen=True)
class FixtureCluster:
    room_id: int
    kind: FixtureKind
    wall_edge: int
    span_start: float
    span_length: float
    footprint: Polygon
    instances: tuple[FixtureInstance, ...]
    boxes: tuple[BoxFixture, ...]
    wall_segments: tuple[WallSegment, ...]
    cubicles: tuple[CubicleLayout, ...] = ()


@dataclass(frozen=True)
class FixtureLayout:
    clusters: tuple[FixtureCluster, ...]

    @property
    def instances(self) -> tuple[FixtureInstance, ...]:
        return tuple(instance for cluster in self.clusters for instance in cluster.instances)

    @property
    def boxes(self) -> tuple[BoxFixture, ...]:
        return tuple(box for cluster in self.clusters for box in cluster.boxes)

    @property
    def cubicles(self) -> tuple[CubicleLayout, ...]:
        return tuple(cubicle for cluster in self.clusters for cubicle in cluster.cubicles)

    @property
    def extra_wall_segments(self) -> tuple[WallSegment, ...]:
        return tuple(
            segment for cluster in self.clusters for segment in cluster.wall_segments
        )

    @property
    def footprints(self) -> tuple[Polygon, ...]:
        return tuple(cluster.footprint for cluster in self.clusters)

    @property
    def collision_footprints(self) -> tuple[Polygon, ...]:
        footprints: list[Polygon] = []
        for instance in self.instances:
            footprints.append(fixture_collision_footprint(instance))
        for box in self.boxes:
            footprints.append(box_collision_footprint(box))
        return tuple(footprints)

    @property
    def mesh_relpaths(self) -> frozenset[str]:
        return frozenset(instance.mesh_relpath for instance in self.instances)


EMPTY_FIXTURE_LAYOUT = FixtureLayout(clusters=())


def fixture_collision_footprint(instance: FixtureInstance) -> Polygon:
    return _oriented_rectangle(
        instance.x,
        instance.y,
        instance.collision_size[0],
        instance.collision_size[1],
        instance.yaw,
    )


def box_collision_footprint(box: BoxFixture) -> Polygon:
    return _oriented_rectangle(box.x, box.y, box.size_x, box.size_y, box.yaw)


def _oriented_rectangle(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    yaw: float,
) -> Polygon:
    half_x = size_x / 2.0
    half_y = size_y / 2.0
    corners = (
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_corners = [
        (
            center_x + local_x * cos_yaw - local_y * sin_yaw,
            center_y + local_x * sin_yaw + local_y * cos_yaw,
        )
        for local_x, local_y in corners
    ]
    return Polygon(world_corners)


@dataclass(frozen=True)
class _RoomWallEdge:
    index: int
    edge: Edge

    @property
    def p1(self) -> Vec2:
        return self.edge.a

    @property
    def p2(self) -> Vec2:
        return self.edge.b

    @property
    def length(self) -> float:
        return self.edge.length

    @property
    def inward(self) -> Vec2:
        dx = self.p2[0] - self.p1[0]
        dy = self.p2[1] - self.p1[1]
        length = math.hypot(dx, dy)
        if length <= EPS:
            return (0.0, 1.0)
        return (-dy / length, dx / length)

    @property
    def yaw_into_room(self) -> float:
        inward = self.inward
        return math.atan2(inward[1], inward[0])

    def point_at(self, arc: float) -> Vec2:
        length = self.length
        if length <= EPS:
            return self.p1
        t = arc / length
        return (
            self.p1[0] + t * (self.p2[0] - self.p1[0]),
            self.p1[1] + t * (self.p2[1] - self.p1[1]),
        )


def generate_fixtures(
    wall_layout: WallLayout,
    config: Config,
    rng: random.Random,
) -> FixtureLayout:
    if config.fixture_mode == "none":
        return EMPTY_FIXTURE_LAYOUT
    if config.fixture_mode == "restroom_clusters":
        return _generate_restroom_clusters(wall_layout, config, rng)
    raise FixtureError(f"Unsupported fixture_mode: {config.fixture_mode!r}")


def _generate_restroom_clusters(
    wall_layout: WallLayout,
    config: Config,
    rng: random.Random,
) -> FixtureLayout:
    layout = wall_layout.opening_layout.applied_layout
    clusters: list[FixtureCluster] = []
    cluster_index = 0

    room_ids = sorted(
        cell.id
        for cell in layout.partition.cells
        if layout.role_for(cell.id) is CellRole.ROOM
    )

    for room_id in room_ids:
        cell = next(item for item in layout.partition.cells if item.id == room_id)
        if not cell.is_rectangle:
            raise FixtureError(f"Room {room_id} is not rectangular")

        room_openings = [
            opening
            for opening in wall_layout.opening_layout.openings
            if room_id in (opening.cell_a_id, opening.cell_b_id)
        ]
        room_rng = random.Random(config.random_seed + room_id * 9973)
        room_clusters = _place_room_clusters(
            cell,
            room_openings,
            config,
            room_rng,
            cluster_index,
        )
        clusters.extend(room_clusters)
        cluster_index += len(room_clusters)

    return FixtureLayout(clusters=tuple(clusters))


def _place_room_clusters(
    cell: Cell,
    openings: list[Opening],
    config: Config,
    rng: random.Random,
    cluster_index: int,
) -> list[FixtureCluster]:
    wall_edges = [_RoomWallEdge(index=edge.index, edge=edge) for edge in cell.edges]
    entrance_intervals = _entrance_intervals_by_wall(wall_edges, openings, cell.id)
    placed: list[FixtureCluster] = []
    occupied_spans: dict[int, list[tuple[float, float]]] = {}

    kinds: tuple[FixtureKind, ...] = ("toilet", "urinal", "basin")

    for kind in kinds:
        spec = FIXTURE_SPECS[kind]
        cluster = _place_cluster_kind(
            cell=cell,
            kind=kind,
            spec=spec,
            wall_edges=wall_edges,
            entrance_intervals=entrance_intervals,
            placed=placed,
            occupied_spans=occupied_spans,
            config=config,
            rng=rng,
            cluster_index=cluster_index,
            remaining_kinds=len(kinds) - kinds.index(kind) - 1,
        )
        placed.append(cluster)
        occupied_spans.setdefault(cluster.wall_edge, []).append(
            (cluster.span_start, cluster.span_start + cluster.span_length)
        )
        cluster_index += 1

    return placed


def _spans_overlap(
    left: tuple[float, float],
    right: tuple[float, float],
) -> bool:
    return min(left[1], right[1]) - max(left[0], right[0]) > EPS


def _place_cluster_kind(
    cell: Cell,
    kind: FixtureKind,
    spec: FixtureSpec,
    wall_edges: list[_RoomWallEdge],
    entrance_intervals: dict[int, list[tuple[float, float]]],
    placed: list[FixtureCluster],
    occupied_spans: dict[int, list[tuple[float, float]]],
    config: Config,
    rng: random.Random,
    cluster_index: int,
    *,
    remaining_kinds: int = 0,
) -> FixtureCluster:
    used_walls = set(occupied_spans)
    wall_order = _wall_candidate_order(
        wall_edges,
        used_walls,
        rng,
        spec=spec,
        entrance_intervals=entrance_intervals,
        placed=placed,
        wall_thickness=config.wall_thickness,
    )
    placed_footprints = [cluster.footprint for cluster in placed]
    entrance_zones = _entrance_clearance_zones(wall_edges, entrance_intervals)
    count_min, count_max = fixture_count_range_for_kind(config, kind)
    required_span = count_min * spec.pitch
    largest_free_interval = 0.0

    for _attempt in range(MAX_PLACEMENT_ATTEMPTS):
        for wall in wall_order:
            if wall.index in entrance_intervals and len(used_walls) < len(wall_edges) - 1:
                if any(
                    other.index not in entrance_intervals
                    for other in wall_edges
                    if other.index not in used_walls
                ):
                    continue
            free_intervals = _subtract_span_intervals(
                _free_intervals_on_wall(
                    wall,
                    entrance_intervals.get(wall.index, ()),
                    placed,
                    wall_edges,
                    config.wall_thickness,
                ),
                occupied_spans.get(wall.index, ()),
            )
            free_intervals.sort(key=lambda item: item[1] - item[0], reverse=True)
            for interval_start, interval_end in free_intervals:
                interval_len = interval_end - interval_start
                largest_free_interval = max(largest_free_interval, interval_len)
                max_fit = int(math.floor(interval_len / spec.pitch))
                count = min(count_max, max_fit)
                if count < count_min:
                    continue
                if remaining_kinds > 0:
                    count = count_min
                elif count > count_min:
                    count = rng.randint(count_min, count)
                else:
                    count = count_min

                span = count * spec.pitch
                if span > interval_len + EPS:
                    continue
                max_offset = interval_len - span
                if max_offset > EPS:
                    for trial_offset in (
                        max_offset / 2.0,
                        0.0,
                        max_offset,
                        rng.uniform(0.0, max_offset),
                    ):
                        span_start = interval_start + trial_offset
                        footprint = _cluster_footprint(
                            wall, span_start, span, spec.cluster_depth
                        )
                        if not cell.polygon.contains(footprint):
                            continue
                        if any(
                            _footprints_overlap(footprint, other)
                            for other in placed_footprints
                        ):
                            continue
                        if any(
                            _footprints_overlap(footprint, zone)
                            for zone in entrance_zones
                        ):
                            continue

                        cluster = _emit_cluster_geometry(
                            cell=cell,
                            kind=kind,
                            spec=spec,
                            wall=wall,
                            span_start=span_start,
                            span_length=span,
                            count=count,
                            footprint=footprint,
                            cluster_index=cluster_index,
                            config=config,
                        )
                        return cluster
                    continue

                span_start = interval_start
                footprint = _cluster_footprint(wall, span_start, span, spec.cluster_depth)
                if not cell.polygon.contains(footprint):
                    continue
                if any(_footprints_overlap(footprint, other) for other in placed_footprints):
                    continue
                if any(_footprints_overlap(footprint, zone) for zone in entrance_zones):
                    continue

                cluster = _emit_cluster_geometry(
                    cell=cell,
                    kind=kind,
                    spec=spec,
                    wall=wall,
                    span_start=span_start,
                    span_length=span,
                    count=count,
                    footprint=footprint,
                    cluster_index=cluster_index,
                    config=config,
                )
                return cluster

        rng.shuffle(wall_order)

    raise FixtureError(
        f"Could not place {kind} cluster in room {cell.id}: required span "
        f"{required_span} m (count_min={count_min} * pitch={spec.pitch}), "
        f"largest free wall interval {largest_free_interval} m"
    )


def _subtract_span_intervals(
    intervals: list[tuple[float, float]],
    occupied: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    result = intervals
    for span in occupied:
        result = _subtract_interval(result, span)
    return result


def _wall_candidate_order(
    wall_edges: list[_RoomWallEdge],
    used_walls: set[int],
    rng: random.Random,
    *,
    spec: FixtureSpec | None = None,
    entrance_intervals: dict[int, list[tuple[float, float]]] | None = None,
    placed: list[FixtureCluster] | None = None,
    wall_thickness: float = 0.15,
) -> list[_RoomWallEdge]:
    unused = [wall for wall in wall_edges if wall.index not in used_walls]
    shared = [wall for wall in wall_edges if wall.index in used_walls]

    def sort_key(wall: _RoomWallEdge) -> float:
        if spec is None or entrance_intervals is None or placed is None:
            return wall.length
        free = _free_intervals_on_wall(
            wall,
            entrance_intervals.get(wall.index, ()),
            placed,
            wall_edges,
            wall_thickness,
        )
        return max((end - start for start, end in free), default=0.0)

    unused.sort(key=sort_key, reverse=True)
    shared.sort(key=sort_key, reverse=True)
    rng.shuffle(unused)
    rng.shuffle(shared)
    unused.sort(key=sort_key, reverse=True)
    shared.sort(key=sort_key, reverse=True)
    return unused + shared


def _entrance_intervals_by_wall(
    wall_edges: list[_RoomWallEdge],
    openings: list[Opening],
    room_id: int,
) -> dict[int, list[tuple[float, float]]]:
    intervals: dict[int, list[tuple[float, float]]] = {}
    for opening in openings:
        wall_index = _opening_wall_index(opening, wall_edges, room_id)
        if wall_index is None:
            continue
        wall = wall_edges[wall_index]
        span = _opening_span_on_wall(opening, wall)
        if span is None:
            continue
        intervals.setdefault(wall_index, []).append(span)
    return intervals


def _opening_wall_index(
    opening: Opening,
    wall_edges: list[_RoomWallEdge],
    room_id: int,
) -> int | None:
    for wall in wall_edges:
        if _shared_wall_on_edge(opening.shared_wall.p1, opening.shared_wall.p2, wall):
            return wall.index
    for wall in wall_edges:
        span = _opening_span_on_wall(opening, wall)
        if span is not None:
            return wall.index
    del room_id
    return None


def _shared_wall_on_edge(p1: Vec2, p2: Vec2, wall: _RoomWallEdge) -> bool:
    for point in (p1, p2):
        if _point_on_wall_edge(point, wall) is None:
            return False
    return True


def _point_on_wall_edge(point: Vec2, wall: _RoomWallEdge) -> float | None:
    ax, ay = wall.p1
    bx, by = wall.p2
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length <= EPS:
        return None
    rel_x = point[0] - ax
    rel_y = point[1] - ay
    arc = (rel_x * dx + rel_y * dy) / length
    perp = abs(rel_x * (-dy) + rel_y * dx) / length
    if perp > 1e-6:
        return None
    if arc < -EPS or arc > length + EPS:
        return None
    return max(0.0, min(length, arc))


def _opening_span_on_wall(
    opening: Opening,
    wall: _RoomWallEdge,
) -> tuple[float, float] | None:
    starts: list[float] = []
    for point in opening.shared_wall.point_at_arc_length(opening.span_start), opening.shared_wall.point_at_arc_length(opening.span_end):
        arc = _point_on_wall_edge(point, wall)
        if arc is None:
            return None
        starts.append(arc)
    if len(starts) != 2:
        return None
    low, high = sorted(starts)
    if high - low <= EPS:
        return None
    return (max(0.0, low), min(wall.length, high))


def _free_intervals_on_wall(
    wall: _RoomWallEdge,
    entrance_intervals: list[tuple[float, float]],
    placed: list[FixtureCluster],
    wall_edges: list[_RoomWallEdge],
    wall_thickness: float,
) -> list[tuple[float, float]]:
    intervals = [(0.0, wall.length)]
    for start, end in entrance_intervals:
        pad_start = max(0.0, start - ENTRANCE_ALONG_WALL_CLEARANCE)
        pad_end = min(wall.length, end + ENTRANCE_ALONG_WALL_CLEARANCE)
        intervals = _subtract_interval(intervals, (pad_start, pad_end))

    corner_inset = _corner_inset(wall, placed, wall_edges, wall_thickness)
    start_inset, end_inset = corner_inset
    if start_inset + end_inset >= wall.length - EPS:
        return []
    if start_inset > EPS or end_inset > EPS:
        intervals = _subtract_interval(intervals, (0.0, start_inset))
        intervals = _subtract_interval(intervals, (wall.length - end_inset, wall.length))

    return [(start, end) for start, end in intervals if end - start > EPS]


def _corner_inset(
    wall: _RoomWallEdge,
    placed: list[FixtureCluster],
    wall_edges: list[_RoomWallEdge],
    wall_thickness: float,
) -> tuple[float, float]:
    start_inset = wall_thickness
    end_inset = wall_thickness
    edge_count = len(wall_edges)

    prev_index = (wall.index - 1) % edge_count
    next_index = (wall.index + 1) % edge_count
    for cluster in placed:
        if cluster.wall_edge == prev_index:
            start_inset = max(start_inset, FIXTURE_SPECS[cluster.kind].cluster_depth)
        if cluster.wall_edge == next_index:
            end_inset = max(end_inset, FIXTURE_SPECS[cluster.kind].cluster_depth)
    return start_inset, end_inset


def _entrance_clearance_zones(
    wall_edges: list[_RoomWallEdge],
    entrance_intervals: dict[int, list[tuple[float, float]]],
) -> list[Polygon]:
    zones: list[Polygon] = []
    for wall_index, intervals in entrance_intervals.items():
        wall = wall_edges[wall_index]
        for start, end in intervals:
            zones.append(
                _cluster_footprint(
                    wall,
                    start,
                    end - start,
                    ENTRANCE_INTO_ROOM_CLEARANCE,
                )
            )
    return zones


def _cluster_footprint(
    wall: _RoomWallEdge,
    span_start: float,
    span_length: float,
    depth: float,
) -> Polygon:
    back_left = wall.point_at(span_start)
    back_right = wall.point_at(span_start + span_length)
    inward = wall.inward
    front_left = (
        back_left[0] + inward[0] * depth,
        back_left[1] + inward[1] * depth,
    )
    front_right = (
        back_right[0] + inward[0] * depth,
        back_right[1] + inward[1] * depth,
    )
    return Polygon([back_left, back_right, front_right, front_left])


def _footprints_overlap(left: Polygon, right: Polygon) -> bool:
    if not left.intersects(right):
        return False
    return left.intersection(right).area > 0.05


def _emit_cluster_geometry(
    cell: Cell,
    kind: FixtureKind,
    spec: FixtureSpec,
    wall: _RoomWallEdge,
    span_start: float,
    span_length: float,
    count: int,
    footprint: Polygon,
    cluster_index: int,
    config: Config,
) -> FixtureCluster:
    prefix = f"room{cell.id}_{kind}_{cluster_index}"
    yaw = wall.yaw_into_room + FIXTURE_YAW_OFFSET
    inward = wall.inward
    visual_offset = fixture_visual_offset_for_kind(config, kind)

    if kind == "toilet":
        return _emit_toilet_cluster(
            prefix,
            cell.id,
            spec,
            wall,
            span_start,
            count,
            span_length,
            footprint,
            yaw,
            inward,
            visual_offset,
            config,
        )
    if kind == "urinal":
        return _emit_urinal_cluster(
            prefix,
            cell.id,
            spec,
            wall,
            span_start,
            count,
            span_length,
            footprint,
            yaw,
            inward,
            visual_offset,
        )
    return _emit_basin_cluster(
        prefix,
        cell.id,
        spec,
        wall,
        span_start,
        count,
        span_length,
        footprint,
        yaw,
        inward,
        visual_offset,
    )


def _emit_toilet_cluster(
    prefix: str,
    room_id: int,
    spec: FixtureSpec,
    wall: _RoomWallEdge,
    span_start: float,
    count: int,
    span_length: float,
    footprint: Polygon,
    yaw: float,
    inward: Vec2,
    visual_offset: FixtureVisualOffset,
    config: Config,
) -> FixtureCluster:
    instances: list[FixtureInstance] = []
    wall_segments: list[WallSegment] = []
    cubicles: list[CubicleLayout] = []
    door_width = config.cubicle_door_width
    segment_height = config.cubicle_wall_height
    front_wall_length = spec.pitch - door_width
    min_segment_length = config.wall_thickness

    for bay in range(count):
        bay_start = span_start + bay * spec.pitch
        bay_end = bay_start + spec.pitch
        bay_center = bay_start + spec.pitch / 2.0
        back = wall.point_at(bay_center)
        toilet_pos = (
            back[0] + inward[0] * (spec.cluster_depth - 0.5),
            back[1] + inward[1] * (spec.cluster_depth - 0.5),
        )
        instance_name = f"{prefix}_toilet_{bay + 1}"
        instances.append(
            FixtureInstance(
                name=instance_name,
                kind="toilet",
                room_id=room_id,
                x=toilet_pos[0],
                y=toilet_pos[1],
                z=spec.z_offset,
                yaw=yaw,
                collision_size=spec.collision_size,
                mesh_relpath=spec.mesh_relpath,
                visual_offset=visual_offset,
            )
        )

        back_left = wall.point_at(bay_start)
        back_right = wall.point_at(bay_end)
        front_left = (
            back_left[0] + inward[0] * spec.cluster_depth,
            back_left[1] + inward[1] * spec.cluster_depth,
        )
        front_right = (
            back_right[0] + inward[0] * spec.cluster_depth,
            back_right[1] + inward[1] * spec.cluster_depth,
        )
        cubicle_polygon = Polygon([back_left, back_right, front_right, front_left])
        door_start = _point_at_depth(wall, bay_start + front_wall_length, spec.cluster_depth)
        door_end = front_right
        cubicles.append(
            CubicleLayout(
                index=bay,
                name=f"{prefix}_cubicle_{bay + 1}",
                room_id=room_id,
                cluster_name=prefix,
                polygon=cubicle_polygon,
                door_span=CubicleDoorSpan(p1=door_start, p2=door_end),
                toilet_instance_name=instance_name,
            )
        )

        if front_wall_length + EPS >= min_segment_length:
            front_segment = _segment_at_depth(
                wall,
                bay_start,
                bay_start + front_wall_length,
                spec.cluster_depth,
                height=segment_height,
            )
            if front_segment is not None:
                wall_segments.append(front_segment)

    partition_arcs = [span_start + index * spec.pitch for index in range(count + 1)]
    for partition_arc in partition_arcs:
        partition = _partition_segment(
            wall,
            partition_arc,
            spec.cluster_depth,
            height=segment_height,
        )
        if partition.length + EPS >= min_segment_length:
            wall_segments.append(partition)

    return FixtureCluster(
        room_id=room_id,
        kind="toilet",
        wall_edge=wall.index,
        span_start=span_start,
        span_length=span_length,
        footprint=footprint,
        instances=tuple(instances),
        boxes=(),
        wall_segments=tuple(wall_segments),
        cubicles=tuple(cubicles),
    )


def _point_at_depth(wall: _RoomWallEdge, arc: float, depth: float) -> Vec2:
    back = wall.point_at(arc)
    inward = wall.inward
    return (back[0] + inward[0] * depth, back[1] + inward[1] * depth)


def _segment_at_depth(
    wall: _RoomWallEdge,
    arc_start: float,
    arc_end: float,
    depth: float,
    *,
    height: float | None = None,
) -> WallSegment | None:
    if arc_end - arc_start <= EPS:
        return None
    return WallSegment(
        p1=_point_at_depth(wall, arc_start, depth),
        p2=_point_at_depth(wall, arc_end, depth),
        height=height,
        material_key="laminate",
    )


def _partition_segment(
    wall: _RoomWallEdge,
    arc: float,
    depth: float,
    *,
    height: float | None = None,
) -> WallSegment:
    back = wall.point_at(arc)
    inward = wall.inward
    front = (back[0] + inward[0] * depth, back[1] + inward[1] * depth)
    return WallSegment(p1=back, p2=front, height=height, material_key="laminate")


def _emit_urinal_cluster(
    prefix: str,
    room_id: int,
    spec: FixtureSpec,
    wall: _RoomWallEdge,
    span_start: float,
    count: int,
    span_length: float,
    footprint: Polygon,
    yaw: float,
    inward: Vec2,
    visual_offset: FixtureVisualOffset,
) -> FixtureCluster:
    instances: list[FixtureInstance] = []
    for index in range(count):
        center_arc = span_start + (index + 0.5) * spec.pitch
        back = wall.point_at(center_arc)
        pos = (
            back[0] + inward[0] * spec.cluster_depth,
            back[1] + inward[1] * spec.cluster_depth,
        )
        instances.append(
            FixtureInstance(
                name=f"{prefix}_urinal_{index + 1}",
                kind="urinal",
                room_id=room_id,
                x=pos[0],
                y=pos[1],
                z=spec.z_offset,
                yaw=yaw,
                collision_size=spec.collision_size,
                mesh_relpath=spec.mesh_relpath,
                visual_offset=visual_offset,
            )
        )
    return FixtureCluster(
        room_id=room_id,
        kind="urinal",
        wall_edge=wall.index,
        span_start=span_start,
        span_length=span_length,
        footprint=footprint,
        instances=tuple(instances),
        boxes=(),
        wall_segments=(),
    )


def _emit_basin_cluster(
    prefix: str,
    room_id: int,
    spec: FixtureSpec,
    wall: _RoomWallEdge,
    span_start: float,
    count: int,
    span_length: float,
    footprint: Polygon,
    yaw: float,
    inward: Vec2,
    visual_offset: FixtureVisualOffset,
) -> FixtureCluster:
    span = span_length
    center_arc = span_start + span / 2.0
    back_center = wall.point_at(center_arc)
    center = (
        back_center[0] + inward[0] * (spec.cluster_depth / 2.0),
        back_center[1] + inward[1] * (spec.cluster_depth / 2.0),
    )

    size_counter = (span, spec.cluster_depth, 0.05)
    size_cabinet = (span, spec.cluster_depth, 0.7)

    boxes = (
        BoxFixture(
            name=f"{prefix}_counter",
            room_id=room_id,
            x=center[0],
            y=center[1],
            z=0.725,
            yaw=yaw,
            size_x=size_counter[0],
            size_y=size_counter[1],
            size_z=size_counter[2],
            color_key="counter",
        ),
        BoxFixture(
            name=f"{prefix}_cabinet",
            room_id=room_id,
            x=center[0],
            y=center[1],
            z=0.35,
            yaw=yaw,
            size_x=size_cabinet[0],
            size_y=size_cabinet[1],
            size_z=size_cabinet[2],
            color_key="cabinet",
        ),
    )

    instances: list[FixtureInstance] = []
    for index in range(count):
        center_arc = span_start + (index + 0.5) * spec.pitch
        back = wall.point_at(center_arc)
        pos = (
            back[0] + inward[0] * spec.cluster_depth,
            back[1] + inward[1] * spec.cluster_depth,
        )
        instances.append(
            FixtureInstance(
                name=f"{prefix}_basin_{index + 1}",
                kind="basin",
                room_id=room_id,
                x=pos[0],
                y=pos[1],
                z=spec.z_offset,
                yaw=yaw,
                collision_size=spec.collision_size,
                mesh_relpath=spec.mesh_relpath,
                visual_offset=visual_offset,
            )
        )

    return FixtureCluster(
        room_id=room_id,
        kind="basin",
        wall_edge=wall.index,
        span_start=span_start,
        span_length=span_length,
        footprint=footprint,
        instances=tuple(instances),
        boxes=boxes,
        wall_segments=(),
    )


def _subtract_interval(
    intervals: list[tuple[float, float]],
    hole: tuple[float, float],
) -> list[tuple[float, float]]:
    hole_start, hole_end = hole
    result: list[tuple[float, float]] = []
    for start, end in intervals:
        if hole_end <= start + EPS or hole_start >= end - EPS:
            result.append((start, end))
            continue
        if start + EPS < hole_start:
            result.append((start, hole_start))
        if hole_end + EPS < end:
            result.append((hole_end, end))
    return result


def box_fixture_color(box: BoxFixture, textures_enabled: bool) -> tuple[float, float, float]:
    if not textures_enabled:
        return (0.8, 0.8, 0.8)
    if box.color_key == "counter":
        return COUNTER_TOP
    return CABINET_WOOD
