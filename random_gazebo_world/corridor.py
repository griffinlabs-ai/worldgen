from __future__ import annotations

import random
from dataclasses import dataclass, replace

from random_gazebo_world.adjacency import AdjacencyGraph, build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, SharedWall, get_shared_wall
from random_gazebo_world.openings import LogicalOpening, Opening, OpeningLayout
from random_gazebo_world.partition import Partition
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnection,
    CandidateConnections,
    ConnectionType,
    RoomSelection,
    SelectedRoomGraph,
)

CORRIDOR_CELL_ID = 0


class CorridorError(RuntimeError):
    """Raised when a corridor layout cannot be generated under constraints."""


@dataclass(frozen=True)
class CorridorLayout:
    config: Config
    partition: Partition
    adjacency: AdjacencyGraph
    room_selection: RoomSelection
    candidates: CandidateConnections
    selected_graph: SelectedRoomGraph
    applied_layout: AppliedLayout
    opening_layout: OpeningLayout


@dataclass(frozen=True)
class _SideRooms:
    widths: tuple[float, ...]
    depths: tuple[float, ...]
    x_starts: tuple[float, ...]


def generate_corridor_layout(config: Config, rng: random.Random) -> CorridorLayout:
    assert config.layout_mode == "corridor"
    assert config.corridor_length is not None
    assert config.corridor_width is not None
    assert config.entrance_width is not None
    assert config.room_width_min is not None
    assert config.room_width_max is not None
    assert config.room_depth_min is not None
    assert config.room_depth_max is not None

    corridor_length = config.corridor_length
    bottom = _sample_side_rooms(
        rng,
        corridor_length,
        config.room_width_min,
        config.room_width_max,
        config.room_depth_min,
        config.room_depth_max,
    )
    top = _sample_side_rooms(
        rng,
        corridor_length,
        config.room_width_min,
        config.room_width_max,
        config.room_depth_min,
        config.room_depth_max,
    )

    _validate_side_tiling(bottom.widths, corridor_length)
    _validate_side_tiling(top.widths, corridor_length)
    _validate_entrance_fits(
        bottom.widths,
        top.widths,
        config.entrance_width,
        config.wall_thickness,
    )

    d_bot = max(bottom.depths) if bottom.depths else 0.0
    d_top = max(top.depths) if top.depths else 0.0
    corridor_y_min = d_bot
    corridor_y_max = d_bot + config.corridor_width
    world_width = corridor_length
    world_height = d_bot + config.corridor_width + d_top

    cells: list[Cell] = [
        Cell.from_origin_size(
            CORRIDOR_CELL_ID,
            0.0,
            corridor_y_min,
            corridor_length,
            config.corridor_width,
        )
    ]

    next_id = 1
    bottom_ids: list[int] = []
    for x_start, width, depth in zip(
        bottom.x_starts, bottom.widths, bottom.depths, strict=True
    ):
        cells.append(
            Cell.from_origin_size(
                next_id,
                x_start,
                corridor_y_min - depth,
                width,
                depth,
            )
        )
        bottom_ids.append(next_id)
        next_id += 1

    top_ids: list[int] = []
    for x_start, width, depth in zip(top.x_starts, top.widths, top.depths, strict=True):
        cells.append(
            Cell.from_origin_size(
                next_id,
                x_start,
                corridor_y_max,
                width,
                depth,
            )
        )
        top_ids.append(next_id)
        next_id += 1

    partition = Partition(
        cells=tuple(cells),
        world_width=world_width,
        world_height=world_height,
    )
    adjacency = build_adjacency_graph(partition)
    room_ids = frozenset(bottom_ids + top_ids)
    room_selection = RoomSelection(partition=partition, room_cell_ids=room_ids)

    corridor_cell = partition.cells[0]
    cells_by_id = {cell.id: cell for cell in partition.cells}
    logical_openings: list[LogicalOpening] = []
    openings: list[Opening] = []
    connections: list[CandidateConnection] = []
    margin = config.wall_thickness
    entrance_width = config.entrance_width

    for room_id in sorted(room_ids):
        room_cell = cells_by_id[room_id]
        shared_wall = get_shared_wall(room_cell, corridor_cell)
        if shared_wall is None:
            raise CorridorError(
                f"Room {room_id} is not adjacent to corridor cell {CORRIDOR_CELL_ID}"
            )

        opening = _place_entrance(
            room_id=room_id,
            corridor_id=CORRIDOR_CELL_ID,
            shared_wall=shared_wall,
            entrance_width=entrance_width,
            margin=margin,
            rng=rng,
        )
        openings.append(opening)
        logical_openings.append(
            LogicalOpening(
                cell_a_id=opening.cell_a_id,
                cell_b_id=opening.cell_b_id,
                shared_wall=shared_wall,
                kind="gate",
            )
        )
        connections.append(
            CandidateConnection(
                room_a_id=opening.cell_a_id,
                room_b_id=opening.cell_b_id,
                connection_type=ConnectionType.GATE,
                shared_wall=shared_wall,
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
        passage_cell_ids=frozenset({CORRIDOR_CELL_ID}),
        logical_openings=tuple(logical_openings),
    )
    opening_layout = OpeningLayout(
        applied_layout=applied_layout,
        openings=tuple(openings),
    )

    updated_config = replace(
        config,
        world_width=world_width,
        world_height=world_height,
        gate_width_min=entrance_width,
        gate_width_max=entrance_width,
    )

    return CorridorLayout(
        config=updated_config,
        partition=partition,
        adjacency=adjacency,
        room_selection=room_selection,
        candidates=candidates,
        selected_graph=selected_graph,
        applied_layout=applied_layout,
        opening_layout=opening_layout,
    )


def _sample_side_rooms(
    rng: random.Random,
    corridor_length: float,
    width_min: float,
    width_max: float,
    depth_min: float,
    depth_max: float,
) -> _SideRooms:
    if corridor_length + EPS < width_min:
        widths = (corridor_length,)
    else:
        widths_list: list[float] = []
        total = 0.0
        while total < corridor_length - EPS:
            widths_list.append(rng.uniform(width_min, width_max))
            total += widths_list[-1]
        if not widths_list:
            widths_list = [corridor_length]
        scale = corridor_length / sum(widths_list)
        widths = tuple(width * scale for width in widths_list)

    depths = tuple(rng.uniform(depth_min, depth_max) for _ in widths)
    x_starts: list[float] = []
    cursor = 0.0
    for width in widths:
        x_starts.append(cursor)
        cursor += width
    return _SideRooms(widths=widths, depths=depths, x_starts=tuple(x_starts))


def _validate_side_tiling(widths: tuple[float, ...], corridor_length: float) -> None:
    total = sum(widths)
    if abs(total - corridor_length) > 1e-6:
        raise CorridorError(
            f"Room widths sum to {total}, expected corridor_length {corridor_length}"
        )


def _validate_entrance_fits(
    bottom_widths: tuple[float, ...],
    top_widths: tuple[float, ...],
    entrance_width: float,
    wall_thickness: float,
) -> None:
    min_wall = entrance_width + 2.0 * wall_thickness
    for width in (*bottom_widths, *top_widths):
        if width + EPS < min_wall:
            raise CorridorError(
                f"Room width {width} too narrow for entrance "
                f"{entrance_width} with margin {wall_thickness}"
            )


def _place_entrance(
    room_id: int,
    corridor_id: int,
    shared_wall: SharedWall,
    entrance_width: float,
    margin: float,
    rng: random.Random,
) -> Opening:
    wall_length = shared_wall.length
    min_center = margin + entrance_width / 2.0
    max_center = wall_length - margin - entrance_width / 2.0
    if min_center > max_center + EPS:
        raise CorridorError(
            f"Cannot fit entrance on room {room_id}-corridor wall of length {wall_length}"
        )

    center = rng.uniform(min_center, max_center)
    span_start = center - entrance_width / 2.0
    span_end = center + entrance_width / 2.0
    cell_a_id = min(room_id, corridor_id)
    cell_b_id = max(room_id, corridor_id)
    return Opening(
        cell_a_id=cell_a_id,
        cell_b_id=cell_b_id,
        shared_wall=shared_wall,
        kind="gate",
        width=entrance_width,
        span_start=span_start,
        span_end=span_end,
    )
