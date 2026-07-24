from __future__ import annotations

import math
from typing import Literal

from shapely.geometry import Polygon

from random_gazebo_world.adjacency import AdjacencyGraph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, Edge, SharedWall, Vec2
from random_gazebo_world.openings import Opening, OpeningLayout
from random_gazebo_world.passage_geometry import PassageGeometryLayout
from random_gazebo_world.topology import CellRole


class WallGenerationError(RuntimeError):
    """Raised when wall segments are invalid."""


class WallSegment:
    """A straight wall segment defined by its two endpoints.

    Axis-aligned convenience accessors are provided so existing rectangular
    callers and tests keep working.
    """

    __slots__ = ("p1", "p2", "height", "material_key")

    def __init__(
        self,
        orientation: Literal["vertical", "horizontal"] | None = None,
        fixed_coord: float | None = None,
        span_start: float | None = None,
        span_end: float | None = None,
        *,
        p1: Vec2 | None = None,
        p2: Vec2 | None = None,
        height: float | None = None,
        material_key: str | None = None,
    ) -> None:
        if p1 is not None and p2 is not None:
            point_a, point_b = p1, p2
        elif orientation is not None:
            if fixed_coord is None or span_start is None or span_end is None:
                raise ValueError("Axis-aligned WallSegment requires span bounds")
            if orientation == "vertical":
                point_a = (fixed_coord, span_start)
                point_b = (fixed_coord, span_end)
            else:
                point_a = (span_start, fixed_coord)
                point_b = (span_end, fixed_coord)
        else:
            raise ValueError("WallSegment requires endpoints or axis description")

        if point_b < point_a:
            point_a, point_b = point_b, point_a
        object.__setattr__(self, "p1", point_a)
        object.__setattr__(self, "p2", point_b)
        if height is not None and height <= 0:
            raise ValueError("WallSegment height must be positive when provided")
        object.__setattr__(self, "height", height)
        object.__setattr__(self, "material_key", material_key)

    @property
    def length(self) -> float:
        return math.hypot(self.p2[0] - self.p1[0], self.p2[1] - self.p1[1])

    @property
    def orientation(self) -> str:
        if abs(self.p1[0] - self.p2[0]) <= EPS:
            return "vertical"
        if abs(self.p1[1] - self.p2[1]) <= EPS:
            return "horizontal"
        return "diagonal"

    @property
    def fixed_coord(self) -> float:
        orientation = self.orientation
        if orientation == "vertical":
            return self.p1[0]
        if orientation == "horizontal":
            return self.p1[1]
        raise ValueError("fixed_coord undefined for diagonal wall")

    @property
    def span_start(self) -> float:
        if self.orientation == "vertical":
            return min(self.p1[1], self.p2[1])
        if self.orientation == "horizontal":
            return min(self.p1[0], self.p2[0])
        raise ValueError("span_start undefined for diagonal wall")

    @property
    def span_end(self) -> float:
        if self.orientation == "vertical":
            return max(self.p1[1], self.p2[1])
        if self.orientation == "horizontal":
            return max(self.p1[0], self.p2[0])
        raise ValueError("span_end undefined for diagonal wall")

    @property
    def midpoint(self) -> Vec2:
        return (
            (self.p1[0] + self.p2[0]) / 2.0,
            (self.p1[1] + self.p2[1]) / 2.0,
        )

    @property
    def yaw(self) -> float:
        return math.atan2(self.p2[1] - self.p1[1], self.p2[0] - self.p1[0])

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, WallSegment):
            return NotImplemented
        return (
            self.p1 == other.p1
            and self.p2 == other.p2
            and self.height == other.height
            and self.material_key == other.material_key
        )

    def __hash__(self) -> int:
        return hash((self.p1, self.p2, self.height, self.material_key))

    def __repr__(self) -> str:
        if self.height is None and self.material_key is None:
            return f"WallSegment(p1={self.p1}, p2={self.p2})"
        extras = []
        if self.height is not None:
            extras.append(f"height={self.height}")
        if self.material_key is not None:
            extras.append(f"material_key={self.material_key!r}")
        return f"WallSegment(p1={self.p1}, p2={self.p2}, {', '.join(extras)})"


class WallLayout:
    """Walls plus solid fills for a generated world.

    ``unused_solids`` are shapely polygons for cells that were not selected as
    rooms or passages.
    """

    __slots__ = ("opening_layout", "segments", "passage_geometry", "unused_solids")

    def __init__(
        self,
        opening_layout: OpeningLayout,
        segments: tuple[WallSegment, ...],
        passage_geometry: PassageGeometryLayout | None = None,
        unused_solids: tuple[Polygon, ...] = (),
    ) -> None:
        object.__setattr__(self, "opening_layout", opening_layout)
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "passage_geometry", passage_geometry)
        object.__setattr__(self, "unused_solids", unused_solids)


def generate_walls(
    opening_layout: OpeningLayout,
    adjacency: AdjacencyGraph,
    config: Config,
    passage_geometry: PassageGeometryLayout | None = None,
) -> WallLayout:
    layout = opening_layout.applied_layout
    openings_by_pair = _group_openings_by_cell_pair(opening_layout.openings)
    segments: list[WallSegment] = []

    for edge in adjacency.edges:
        left_id = edge.cell_a_id
        right_id = edge.cell_b_id
        role_left = layout.role_for(left_id)
        role_right = layout.role_for(right_id)

        if not _should_generate_interior_wall(role_left, role_right):
            continue

        pair = (min(left_id, right_id), max(left_id, right_id))
        openings = openings_by_pair.get(pair, ())
        segments.extend(
            _wall_segments_from_shared_wall(
                edge.shared_wall,
                openings,
                config.wall_thickness,
            )
        )

    for cell in layout.partition.cells:
        if layout.role_for(cell.id) == CellRole.UNUSED:
            continue
        segments.extend(
            _exterior_wall_segments(cell, adjacency, config.wall_thickness)
        )

    unused_solids = _unused_cell_solids(layout)

    wall_layout = WallLayout(
        opening_layout=opening_layout,
        segments=tuple(segments),
        passage_geometry=passage_geometry,
        unused_solids=unused_solids,
    )
    validate_wall_layout(wall_layout, config)
    return wall_layout


def validate_wall_layout(wall_layout: WallLayout, config: Config) -> None:
    min_length = config.wall_thickness

    for segment in wall_layout.segments:
        if segment.length + EPS < min_length:
            raise WallGenerationError(
                f"Wall segment length {segment.length} below threshold {min_length}"
            )

    for opening in wall_layout.opening_layout.openings:
        for segment in wall_layout.segments:
            if _segment_overlaps_opening(segment, opening):
                raise WallGenerationError(
                    f"Wall segment overlaps opening "
                    f"{opening.cell_a_id}-{opening.cell_b_id}"
                )


def wall_segment_line(
    segment: WallSegment,
) -> tuple[tuple[float, float], tuple[float, float]]:
    return (segment.p1, segment.p2)


def _should_generate_interior_wall(role_left: CellRole, role_right: CellRole) -> bool:
    if role_left == CellRole.UNUSED or role_right == CellRole.UNUSED:
        return False
    if role_left == CellRole.PASSAGE and role_right == CellRole.PASSAGE:
        return False
    return True


def _unused_cell_solids(layout) -> tuple[Polygon, ...]:
    return tuple(
        cell.polygon
        for cell in layout.partition.cells
        if layout.role_for(cell.id) == CellRole.UNUSED
    )


def _group_openings_by_cell_pair(
    openings: tuple[Opening, ...],
) -> dict[tuple[int, int], tuple[Opening, ...]]:
    grouped: dict[tuple[int, int], list[Opening]] = {}
    for opening in openings:
        pair = (opening.cell_a_id, opening.cell_b_id)
        grouped.setdefault(pair, []).append(opening)
    return {pair: tuple(items) for pair, items in grouped.items()}


def _wall_segments_from_shared_wall(
    shared_wall: SharedWall,
    openings: tuple[Opening, ...],
    min_length: float,
) -> list[WallSegment]:
    relevant = [opening for opening in openings if opening.shared_wall == shared_wall]
    intervals = [(0.0, shared_wall.length)]

    for opening in sorted(relevant, key=lambda item: item.span_start):
        intervals = _subtract_interval(
            intervals, (opening.span_start, opening.span_end)
        )

    segments: list[WallSegment] = []
    for start, end in intervals:
        if end - start + EPS < min_length:
            continue
        segments.append(
            WallSegment(
                p1=shared_wall.point_at_arc_length(start),
                p2=shared_wall.point_at_arc_length(end),
            )
        )
    return segments


def _exterior_wall_segments(
    cell: Cell,
    adjacency: AdjacencyGraph,
    min_length: float,
) -> list[WallSegment]:
    incident_walls = [
        edge.shared_wall
        for edge in adjacency.edges
        if cell.id in (edge.cell_a_id, edge.cell_b_id)
    ]

    segments: list[WallSegment] = []
    for edge in cell.edges:
        edge_length = edge.length
        if edge_length <= EPS:
            continue
        intervals = [(0.0, edge_length)]
        for wall in incident_walls:
            covered = _wall_interval_on_edge(wall, edge)
            if covered is not None:
                intervals = _subtract_interval(intervals, covered)

        for start, end in intervals:
            if end - start + EPS < min_length:
                continue
            segments.append(
                WallSegment(
                    p1=_edge_point(edge, start),
                    p2=_edge_point(edge, end),
                )
            )
    return segments


def _segment_overlaps_opening(segment: WallSegment, opening: Opening) -> bool:
    wall = opening.shared_wall
    start = _project_point_on_wall(segment.p1, wall)
    end = _project_point_on_wall(segment.p2, wall)
    if start is None or end is None:
        return False

    seg_start, seg_end = sorted((start, end))
    overlap = min(seg_end, opening.span_end) - max(seg_start, opening.span_start)
    return overlap > EPS


def _project_point_on_wall(point: Vec2, wall: SharedWall) -> float | None:
    length = wall.length
    if length <= EPS:
        return None
    dx = wall.p2[0] - wall.p1[0]
    dy = wall.p2[1] - wall.p1[1]
    rel_x = point[0] - wall.p1[0]
    rel_y = point[1] - wall.p1[1]
    arc = (rel_x * dx + rel_y * dy) / length
    perp = abs(rel_x * (-dy) + rel_y * dx) / length
    if perp > 1e-6:
        return None
    if arc < -EPS or arc > length + EPS:
        return None
    return arc


def _wall_interval_on_edge(
    wall: SharedWall,
    edge: Edge,
) -> tuple[float, float] | None:
    edge_length = edge.length
    if edge_length <= EPS:
        return None
    s1 = _project_point_on_edge(wall.p1, edge)
    s2 = _project_point_on_edge(wall.p2, edge)
    if s1 is None or s2 is None:
        return None
    low, high = sorted((s1, s2))
    if high - low <= EPS:
        return None
    return (max(0.0, low), min(edge_length, high))


def _project_point_on_edge(point: Vec2, edge: Edge) -> float | None:
    ax, ay = edge.a
    bx, by = edge.b
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
    return arc


def _edge_point(edge: Edge, arc: float) -> Vec2:
    length = edge.length
    if length <= EPS:
        return edge.a
    t = arc / length
    return (
        edge.a[0] + t * (edge.b[0] - edge.a[0]),
        edge.a[1] + t * (edge.b[1] - edge.a[1]),
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
