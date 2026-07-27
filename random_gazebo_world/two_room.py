from __future__ import annotations

from dataclasses import dataclass, replace

from shapely.geometry import Polygon
from shapely.ops import unary_union

from random_gazebo_world.adjacency import AdjacencyGraph, build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, get_shared_wall
from random_gazebo_world.openings import LogicalOpening, Opening, OpeningLayout
from random_gazebo_world.partition import Partition
from random_gazebo_world.passage_geometry import (
    PassageCellGeometry,
    PassageGeometryLayout,
    _iter_polygons,
    validate_passage_geometry,
)
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnection,
    CandidateConnections,
    ConnectionType,
    RoomSelection,
    SelectedRoomGraph,
)

PASSAGE_CELL_ID = 0
ROOM_A_CELL_ID = 1
ROOM_B_CELL_ID = 2


class TwoRoomError(RuntimeError):
    """Raised when a two-room test layout cannot be generated under constraints."""


@dataclass(frozen=True)
class RoomCenter:
    cell_id: int
    x: float
    y: float


@dataclass(frozen=True)
class TwoRoomLayout:
    config: Config
    partition: Partition
    adjacency: AdjacencyGraph
    room_selection: RoomSelection
    candidates: CandidateConnections
    selected_graph: SelectedRoomGraph
    applied_layout: AppliedLayout
    opening_layout: OpeningLayout
    room_centers: tuple[RoomCenter, RoomCenter]
    passage_geometry: PassageGeometryLayout | None = None


def generate_two_room_gate_layout(config: Config) -> TwoRoomLayout:
    assert config.layout_mode == "two_room_gate"
    assert config.room_size is not None
    assert config.gate_width is not None
    assert config.divider_thickness is not None

    room_size = config.room_size
    gate_width = config.gate_width
    divider_thickness = config.divider_thickness

    room_a = Cell.from_origin_size(ROOM_A_CELL_ID, 0.0, 0.0, room_size, room_size)
    divider = Cell.from_origin_size(
        PASSAGE_CELL_ID,
        room_size,
        0.0,
        divider_thickness,
        room_size,
    )
    room_b = Cell.from_origin_size(
        ROOM_B_CELL_ID,
        room_size + divider_thickness,
        0.0,
        room_size,
        room_size,
    )

    world_width = 2.0 * room_size + divider_thickness
    world_height = room_size
    partition = Partition(
        cells=(room_a, divider, room_b),
        world_width=world_width,
        world_height=world_height,
    )

    layout = _build_two_room_topology(
        config=config,
        partition=partition,
        room_ids=frozenset({ROOM_A_CELL_ID, ROOM_B_CELL_ID}),
        passage_ids=frozenset({PASSAGE_CELL_ID}),
        openings=_gate_openings_for_row(config, room_a, divider, room_b, gate_width),
    )

    updated_config = replace(
        config,
        world_width=world_width,
        world_height=world_height,
        gate_width_min=gate_width,
        gate_width_max=gate_width,
        passage_width_min=gate_width,
        passage_width_max=gate_width,
        passage_geometry_mode="legacy_orthogonal",
    )

    return replace(
        layout,
        config=updated_config,
        passage_geometry=None,
    )


def generate_two_room_corner_layout(config: Config) -> TwoRoomLayout:
    assert config.layout_mode == "two_room_corner"
    assert config.room_size is not None
    assert config.leg_a_width is not None
    assert config.leg_a_length is not None
    assert config.leg_b_width is not None
    assert config.leg_b_length is not None

    room_size = config.room_size
    leg_a_width = config.leg_a_width
    leg_a_length = config.leg_a_length
    leg_b_width = config.leg_b_width
    leg_b_length = config.leg_b_length
    inset = config.wall_thickness / 2.0

    room_a = Cell.from_origin_size(ROOM_A_CELL_ID, 0.0, 0.0, room_size, room_size)

    passage_width = leg_a_length + inset
    passage_height = inset + leg_a_width + leg_b_length
    passage = Cell.from_origin_size(
        PASSAGE_CELL_ID,
        room_size,
        0.0,
        passage_width,
        passage_height,
    )

    vertical_leg_center_x = room_size + leg_a_length - leg_b_width / 2.0
    room_b_x = vertical_leg_center_x - room_size / 2.0
    room_b = Cell.from_origin_size(
        ROOM_B_CELL_ID,
        room_b_x,
        passage_height,
        room_size,
        room_size,
    )

    world_width = max(room_b.x_max, passage.x_max)
    world_height = room_b.y_max
    partition = Partition(
        cells=(room_a, passage, room_b),
        world_width=world_width,
        world_height=world_height,
    )

    horizontal_leg = Polygon(
        [
            (room_size, inset),
            (room_size + leg_a_length, inset),
            (room_size + leg_a_length, inset + leg_a_width),
            (room_size, inset + leg_a_width),
        ]
    )
    vertical_leg = Polygon(
        [
            (room_size + leg_a_length - leg_b_width, inset + leg_a_width),
            (room_size + leg_a_length, inset + leg_a_width),
            (room_size + leg_a_length, passage_height),
            (room_size + leg_a_length - leg_b_width, passage_height),
        ]
    )
    corridor = unary_union([horizontal_leg, vertical_leg])

    room_a_opening_y = inset + leg_a_width / 2.0
    room_b_opening_x = vertical_leg_center_x

    openings = (
        _centered_opening(
            room_a,
            passage,
            opening_width=leg_a_width,
            axis="vertical",
            center=room_a_opening_y,
        ),
        _centered_opening(
            room_b,
            passage,
            opening_width=leg_b_width,
            axis="horizontal",
            center=room_b_opening_x,
        ),
    )

    layout = _build_two_room_topology(
        config=config,
        partition=partition,
        room_ids=frozenset({ROOM_A_CELL_ID, ROOM_B_CELL_ID}),
        passage_ids=frozenset({PASSAGE_CELL_ID}),
        openings=openings,
    )

    leftover = passage.polygon.difference(corridor)
    solids = tuple(_iter_polygons(leftover))
    passage_geometry = PassageGeometryLayout(
        opening_layout=layout.opening_layout,
        cells=(
            PassageCellGeometry(
                cell_id=PASSAGE_CELL_ID,
                corridor=corridor,
                solids=solids,
            ),
        ),
    )
    validate_passage_geometry(passage_geometry, config)

    updated_config = replace(
        config,
        world_width=world_width,
        world_height=world_height,
        gate_width_min=leg_a_width,
        gate_width_max=leg_a_width,
        passage_width_min=min(leg_a_width, leg_b_width),
        passage_width_max=max(leg_a_width, leg_b_width),
    )

    return replace(
        layout,
        config=updated_config,
        passage_geometry=passage_geometry,
    )


def _build_two_room_topology(
    config: Config,
    partition: Partition,
    room_ids: frozenset[int],
    passage_ids: frozenset[int],
    openings: tuple[Opening, ...],
) -> TwoRoomLayout:
    adjacency = build_adjacency_graph(partition)
    room_selection = RoomSelection(partition=partition, room_cell_ids=room_ids)
    cells_by_id = {cell.id: cell for cell in partition.cells}

    logical_openings: list[LogicalOpening] = []
    connections: list[CandidateConnection] = []
    for opening in openings:
        logical_openings.append(
            LogicalOpening(
                cell_a_id=opening.cell_a_id,
                cell_b_id=opening.cell_b_id,
                shared_wall=opening.shared_wall,
                kind="gate",
            )
        )
        connections.append(
            CandidateConnection(
                room_a_id=opening.cell_a_id,
                room_b_id=opening.cell_b_id,
                connection_type=ConnectionType.GATE,
                shared_wall=opening.shared_wall,
                path_cell_ids=(opening.cell_a_id, opening.cell_b_id),
            )
        )

    candidates = CandidateConnections(
        room_selection=room_selection,
        connections=tuple(connections),
    )
    selected_graph = SelectedRoomGraph(
        candidates=candidates,
        connections=tuple(connections),
        spanning_tree_connections=tuple(connections),
        loop_connections=(),
    )
    applied_layout = AppliedLayout(
        partition=partition,
        room_selection=room_selection,
        selected_graph=selected_graph,
        passage_cell_ids=passage_ids,
        logical_openings=tuple(logical_openings),
    )
    opening_layout = OpeningLayout(
        applied_layout=applied_layout,
        openings=openings,
    )

    room_centers = tuple(
        RoomCenter(
            cell_id=room_id,
            x=cells_by_id[room_id].centroid[0],
            y=cells_by_id[room_id].centroid[1],
        )
        for room_id in (ROOM_A_CELL_ID, ROOM_B_CELL_ID)
    )

    return TwoRoomLayout(
        config=config,
        partition=partition,
        adjacency=adjacency,
        room_selection=room_selection,
        candidates=candidates,
        selected_graph=selected_graph,
        applied_layout=applied_layout,
        opening_layout=opening_layout,
        room_centers=room_centers,
    )


def _gate_openings_for_row(
    config: Config,
    room_a: Cell,
    divider: Cell,
    room_b: Cell,
    gate_width: float,
) -> tuple[Opening, ...]:
    center_y = room_a.height / 2.0
    return (
        _centered_opening(
            room_a,
            divider,
            opening_width=gate_width,
            axis="vertical",
            center=center_y,
        ),
        _centered_opening(
            divider,
            room_b,
            opening_width=gate_width,
            axis="vertical",
            center=center_y,
        ),
    )


def _centered_opening(
    cell_a: Cell,
    cell_b: Cell,
    *,
    opening_width: float,
    axis: str,
    center: float,
) -> Opening:
    del axis
    shared_wall = get_shared_wall(cell_a, cell_b)
    if shared_wall is None:
        raise TwoRoomError(
            f"Cells {cell_a.id} and {cell_b.id} do not share a wall"
        )

    if shared_wall.length + EPS < opening_width:
        raise TwoRoomError(
            f"Opening width {opening_width} exceeds shared wall length "
            f"{shared_wall.length} between cells {cell_a.id} and {cell_b.id}"
        )

    orientation = shared_wall.orientation
    if orientation == "vertical":
        arc_center = center - shared_wall.p1[1]
    elif orientation == "horizontal":
        arc_center = center - shared_wall.p1[0]
    else:
        raise TwoRoomError(
            f"Unsupported shared wall orientation {orientation!r} between "
            f"cells {cell_a.id} and {cell_b.id}"
        )

    span_start = arc_center - opening_width / 2.0
    span_end = arc_center + opening_width / 2.0
    if span_start < -EPS or span_end > shared_wall.length + EPS:
        raise TwoRoomError(
            f"Opening centered at {center} with width {opening_width} does not "
            f"fit on shared wall of length {shared_wall.length}"
        )

    cell_a_id = min(cell_a.id, cell_b.id)
    cell_b_id = max(cell_a.id, cell_b.id)
    return Opening(
        cell_a_id=cell_a_id,
        cell_b_id=cell_b_id,
        shared_wall=shared_wall,
        kind="gate",
        width=opening_width,
        span_start=max(0.0, span_start),
        span_end=min(shared_wall.length, span_end),
    )
