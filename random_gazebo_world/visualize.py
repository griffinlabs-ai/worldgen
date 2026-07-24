from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch
from shapely.geometry.base import BaseGeometry

from random_gazebo_world.fixtures import FixtureLayout
from random_gazebo_world.geometry import Cell, SharedWall
from random_gazebo_world.openings import OpeningLayout, opening_line
from random_gazebo_world.partition import Partition
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    ConnectionType,
    RoomSelection,
    SelectedRoomGraph,
)
from random_gazebo_world.walls import WallLayout, wall_segment_line


def _setup_axes(
    ax: plt.Axes,
    world_width: float,
    world_height: float,
    title: str,
) -> None:
    ax.set_xlim(0.0, world_width)
    ax.set_ylim(0.0, world_height)
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def _save_figure(fig: plt.Figure, output_base: Path) -> None:
    output_base.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_base.with_suffix(".png"), dpi=150)
    fig.savefig(output_base.with_suffix(".svg"))
    plt.close(fig)


def _add_cell_patch(ax: plt.Axes, cell: Cell, **kwargs) -> None:
    ax.add_patch(PolygonPatch(list(cell.polygon_vertices), closed=True, **kwargs))


def _add_shapely_patch(ax: plt.Axes, geometry: BaseGeometry, **kwargs) -> None:
    for polygon in _iter_polygons(geometry):
        coords = list(polygon.exterior.coords)[:-1]
        ax.add_patch(PolygonPatch(coords, closed=True, **kwargs))


def _iter_polygons(geometry: BaseGeometry):
    if geometry.is_empty:
        return
    if geometry.geom_type == "Polygon":
        yield geometry
    elif geometry.geom_type in {"MultiPolygon", "GeometryCollection"}:
        for part in geometry.geoms:
            if part.geom_type == "Polygon" and not part.is_empty:
                yield part


def _centroid(cell: Cell) -> tuple[float, float]:
    return cell.centroid


def render_partition(
    partition: Partition,
    output_base: Path,
    title: str = "Partition",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    colors = plt.get_cmap("tab20")

    for index, cell in enumerate(partition.cells):
        color = colors(index % 20)
        _add_cell_patch(
            ax,
            cell,
            facecolor=color,
            edgecolor="black",
            linewidth=1.0,
            alpha=0.65,
        )
        cx, cy = _centroid(cell)
        ax.text(cx, cy, str(cell.id), ha="center", va="center", fontsize=9,
                color="black", weight="bold")

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def _shared_wall_line(
    shared_wall: SharedWall,
) -> tuple[tuple[float, float], tuple[float, float]]:
    return (shared_wall.p1, shared_wall.p2)


def render_adjacency_graph(
    partition: Partition,
    adjacency: AdjacencyGraph,
    output_base: Path,
    title: str = "Cell Adjacency Graph",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    colors = plt.get_cmap("Pastel1")

    for index, cell in enumerate(partition.cells):
        color = colors(index % 9)
        _add_cell_patch(
            ax, cell, facecolor=color, edgecolor="black", linewidth=0.8, alpha=0.8
        )
        cx, cy = _centroid(cell)
        ax.text(cx, cy, str(cell.id), ha="center", va="center", fontsize=9,
                color="black", weight="bold")

    for edge in adjacency.edges:
        start, end = _shared_wall_line(edge.shared_wall)
        ax.plot([start[0], end[0]], [start[1], end[1]], color="crimson",
                linewidth=3.0, solid_capstyle="round", zorder=3)

        cell_a = adjacency.cell_by_id(edge.cell_a_id)
        cell_b = adjacency.cell_by_id(edge.cell_b_id)
        ca = _centroid(cell_a)
        cb = _centroid(cell_b)
        ax.plot([ca[0], cb[0]], [ca[1], cb[1]], color="navy", linewidth=1.0,
                linestyle="--", alpha=0.7, zorder=2)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_selected_rooms(
    partition: Partition,
    selection: RoomSelection,
    output_base: Path,
    title: str = "Selected Rooms",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in selection.unused_cells():
        _add_cell_patch(ax, cell, facecolor="#d9d9d9", edgecolor="#666666",
                        linewidth=1.0, alpha=0.9)
        cx, cy = _centroid(cell)
        ax.text(cx, cy, f"{cell.id}\nunused", ha="center", va="center",
                fontsize=8, color="#444444")

    for cell in selection.room_cells():
        _add_cell_patch(ax, cell, facecolor="#7bd389", edgecolor="#1f7a3a",
                        linewidth=1.5, alpha=0.9)
        cx, cy = _centroid(cell)
        ax.text(cx, cy, f"{cell.id}\nroom", ha="center", va="center",
                fontsize=9, color="#12351f", weight="bold")

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_candidate_connections(
    partition: Partition,
    room_selection: RoomSelection,
    candidates: CandidateConnections,
    output_base: Path,
    title: str = "Candidate Connections",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    cells_by_id = {cell.id: cell for cell in partition.cells}

    for cell in room_selection.unused_cells():
        _add_cell_patch(ax, cell, facecolor="#d9d9d9", edgecolor="#666666",
                        linewidth=1.0, alpha=0.9)

    for cell in room_selection.room_cells():
        _add_cell_patch(ax, cell, facecolor="#7bd389", edgecolor="#1f7a3a",
                        linewidth=1.5, alpha=0.9)
        cx, cy = _centroid(cell)
        ax.text(cx, cy, str(cell.id), ha="center", va="center", fontsize=9,
                color="#12351f", weight="bold")

    for connection in candidates.connections:
        if connection.connection_type == ConnectionType.GATE:
            assert connection.shared_wall is not None
            start, end = _shared_wall_line(connection.shared_wall)
            ax.plot([start[0], end[0]], [start[1], end[1]], color="#d4a017",
                    linewidth=4.0, solid_capstyle="round", zorder=4)
            continue

        path_cells = [cells_by_id[cell_id] for cell_id in connection.path_cell_ids]
        xs, ys = zip(*(_centroid(cell) for cell in path_cells))
        ax.plot(xs, ys, color="#7b2cbf", linewidth=2.5, linestyle="-",
                marker="o", markersize=5, zorder=4)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def _draw_room_layout_base(ax: plt.Axes, room_selection: RoomSelection) -> None:
    for cell in room_selection.unused_cells():
        _add_cell_patch(ax, cell, facecolor="#d9d9d9", edgecolor="#666666",
                        linewidth=1.0, alpha=0.9)

    for cell in room_selection.room_cells():
        _add_cell_patch(ax, cell, facecolor="#7bd389", edgecolor="#1f7a3a",
                        linewidth=1.5, alpha=0.9)
        cx, cy = _centroid(cell)
        ax.text(cx, cy, str(cell.id), ha="center", va="center", fontsize=9,
                color="#12351f", weight="bold")


def _draw_connection(
    ax: plt.Axes,
    partition: Partition,
    connection,
    *,
    color: str,
    linewidth: float,
    linestyle: str,
) -> None:
    cells_by_id = {cell.id: cell for cell in partition.cells}
    if connection.connection_type == ConnectionType.GATE:
        assert connection.shared_wall is not None
        start, end = _shared_wall_line(connection.shared_wall)
        ax.plot([start[0], end[0]], [start[1], end[1]], color=color,
                linewidth=linewidth, linestyle=linestyle,
                solid_capstyle="round", zorder=4)
        return

    path_cells = [cells_by_id[cell_id] for cell_id in connection.path_cell_ids]
    xs, ys = zip(*(_centroid(cell) for cell in path_cells))
    ax.plot(xs, ys, color=color, linewidth=linewidth, linestyle=linestyle,
            marker="o", markersize=5, zorder=4)


def render_selected_room_graph(
    partition: Partition,
    selected: SelectedRoomGraph,
    output_base: Path,
    title: str = "Selected Room Graph",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    _draw_room_layout_base(ax, selected.room_selection)

    loop_pairs = selected.loop_connection_pairs
    for connection in selected.connections:
        pair = (connection.room_a_id, connection.room_b_id)
        is_loop = pair in loop_pairs
        _draw_connection(
            ax,
            partition,
            connection,
            color="#ff7f0e" if is_loop else "#1f4e79",
            linewidth=3.5 if is_loop else 3.0,
            linestyle="--" if is_loop else "-",
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def _role_style(role_value: str) -> tuple[str, str, str, str]:
    if role_value == "room":
        return "#7bd389", "#1f7a3a", "#12351f", "bold"
    if role_value == "passage":
        return "#8ecae6", "#219ebc", "#023047", "bold"
    return "#d9d9d9", "#666666", "#444444", "normal"


def render_passage_cells(
    layout: AppliedLayout,
    output_base: Path,
    title: str = "Passage Cells",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    partition = layout.partition

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        facecolor, edgecolor, text_color, weight = _role_style(role.value)
        _add_cell_patch(ax, cell, facecolor=facecolor, edgecolor=edgecolor,
                        linewidth=1.2, alpha=0.9)
        cx, cy = _centroid(cell)
        ax.text(cx, cy, f"{cell.id}\n{role.value}", ha="center", va="center",
                fontsize=8, color=text_color, weight=weight)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_openings(
    opening_layout: OpeningLayout,
    output_base: Path,
    title: str = "Openings",
) -> None:
    layout = opening_layout.applied_layout
    partition = layout.partition
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        facecolor, edgecolor, _, _ = _role_style(role.value)
        _add_cell_patch(ax, cell, facecolor=facecolor, edgecolor=edgecolor,
                        linewidth=1.0, alpha=0.85)

    for logical_opening in layout.logical_openings:
        start, end = _shared_wall_line(logical_opening.shared_wall)
        ax.plot([start[0], end[0]], [start[1], end[1]], color="#bbbbbb",
                linewidth=2.0, solid_capstyle="butt", zorder=2)

    for opening in opening_layout.openings:
        start, end = opening_line(opening)
        color = "#d4a017" if opening.kind == "gate" else "#7b2cbf"
        ax.plot([start[0], end[0]], [start[1], end[1]], color=color,
                linewidth=5.0, solid_capstyle="round", zorder=4)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_passage_geometry(
    wall_layout: WallLayout,
    output_base: Path,
    title: str = "Passage Geometry",
) -> None:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    passage_geometry = wall_layout.passage_geometry
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#eef8f0"
        elif role.value == "passage":
            facecolor = "#fbeeee"
        else:
            facecolor = "#f5f5f5"
        _add_cell_patch(ax, cell, facecolor=facecolor, edgecolor="#cccccc",
                        linewidth=0.5, alpha=0.9)

    if passage_geometry is not None:
        for cell_geometry in passage_geometry.cells:
            for solid in cell_geometry.solids:
                _add_shapely_patch(ax, solid, facecolor="#444444",
                                   edgecolor="#222222", linewidth=0.5,
                                   alpha=0.85, zorder=3)
            _add_shapely_patch(ax, cell_geometry.corridor, facecolor="#8ecae6",
                               edgecolor="#219ebc", linewidth=0.5, alpha=0.95,
                               zorder=4)

    for solid in wall_layout.unused_solids:
        _add_shapely_patch(ax, solid, facecolor="#666666", edgecolor="#333333",
                           linewidth=0.5, alpha=0.9, zorder=3)

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot([start[0], end[0]], [start[1], end[1]], color="#111111",
                linewidth=2.0, solid_capstyle="butt", zorder=5)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_fixtures(
    wall_layout: WallLayout,
    fixture_layout: FixtureLayout,
    output_base: Path,
    title: str = "Fixtures",
) -> None:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#eef8f0"
        elif role.value == "passage":
            facecolor = "#eef7fb"
        else:
            facecolor = "#f5f5f5"
        _add_cell_patch(ax, cell, facecolor=facecolor, edgecolor="#cccccc",
                        linewidth=0.5, alpha=0.9)

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot([start[0], end[0]], [start[1], end[1]], color="#111111",
                linewidth=1.5, solid_capstyle="butt", zorder=4)

    kind_colors = {
        "toilet": "#6a4c93",
        "urinal": "#1982c4",
        "basin": "#8ac926",
    }
    for cluster in fixture_layout.clusters:
        _add_shapely_patch(
            ax,
            cluster.footprint,
            facecolor=kind_colors.get(cluster.kind, "#888888"),
            edgecolor="#333333",
            linewidth=1.0,
            alpha=0.45,
            zorder=5,
        )

    for instance in fixture_layout.instances:
        color = kind_colors.get(instance.kind, "#888888")
        dx = 0.15 * math.cos(instance.yaw)
        dy = 0.15 * math.sin(instance.yaw)
        ax.plot(
            [instance.x, instance.x + dx],
            [instance.y, instance.y + dy],
            color=color,
            linewidth=2.0,
            zorder=6,
        )
        ax.scatter([instance.x], [instance.y], color=color, s=30, zorder=7)

    for box in fixture_layout.boxes:
        half_x = box.size_x / 2.0
        half_y = box.size_y / 2.0
        corners = [
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        ]
        cos_yaw = math.cos(box.yaw)
        sin_yaw = math.sin(box.yaw)
        world_corners = [
            (
                box.x + local_x * cos_yaw - local_y * sin_yaw,
                box.y + local_x * sin_yaw + local_y * cos_yaw,
            )
            for local_x, local_y in corners
        ]
        ax.add_patch(
            PolygonPatch(
                world_corners,
                closed=True,
                facecolor="#b5651d" if box.color_key == "cabinet" else "#888888",
                edgecolor="#333333",
                linewidth=0.8,
                alpha=0.6,
                zorder=6,
            )
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_wall_segments(
    wall_layout: WallLayout,
    output_base: Path,
    title: str = "Wall Segments",
) -> None:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#eef8f0"
        elif role.value == "passage":
            facecolor = "#eef7fb"
        else:
            facecolor = "#f5f5f5"
        _add_cell_patch(ax, cell, facecolor=facecolor, edgecolor="#cccccc",
                        linewidth=0.5, alpha=0.9)

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot([start[0], end[0]], [start[1], end[1]], color="#111111",
                linewidth=3.0, solid_capstyle="butt", zorder=5)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_final_floorplan(
    wall_layout: WallLayout,
    output_base: Path,
    title: str = "Final Floor Plan",
) -> None:
    opening_layout = wall_layout.opening_layout
    layout = opening_layout.applied_layout
    partition = layout.partition
    passage_geometry = wall_layout.passage_geometry
    cells_by_id = {cell.id: cell for cell in partition.cells}
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#7bd389"
            edgecolor = "#1f7a3a"
            label = str(cell.id)
        elif role.value == "passage":
            facecolor = "#ececec" if passage_geometry is not None else "#8ecae6"
            edgecolor = "#aaaaaa" if passage_geometry is not None else "#219ebc"
            label = str(cell.id)
        else:
            facecolor = "#ececec"
            edgecolor = "#aaaaaa"
            label = ""

        _add_cell_patch(ax, cell, facecolor=facecolor, edgecolor=edgecolor,
                        linewidth=1.0, alpha=0.95)
        if label:
            cx, cy = _centroid(cell)
            ax.text(cx, cy, label, ha="center", va="center", fontsize=9,
                    color="#12351f", weight="bold")

    if passage_geometry is not None:
        for cell_geometry in passage_geometry.cells:
            _add_shapely_patch(ax, cell_geometry.corridor, facecolor="#8ecae6",
                               edgecolor="none", alpha=0.95, zorder=2)

    for solid in wall_layout.unused_solids:
        _add_shapely_patch(ax, solid, facecolor="#666666", edgecolor="#333333",
                           linewidth=0.5, alpha=0.95, zorder=2)

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot([start[0], end[0]], [start[1], end[1]], color="#111111",
                linewidth=3.0, solid_capstyle="butt", zorder=4)

    for opening in opening_layout.openings:
        start, end = opening_line(opening)
        color = "#d4a017" if opening.kind == "gate" else "#7b2cbf"
        ax.plot([start[0], end[0]], [start[1], end[1]], color=color,
                linewidth=4.5, solid_capstyle="round", zorder=5)

    for connection in layout.selected_graph.connections:
        if connection.connection_type != ConnectionType.PASSAGE:
            continue
        path_cells = [cells_by_id[cell_id] for cell_id in connection.path_cell_ids]
        xs, ys = zip(*(_centroid(cell) for cell in path_cells))
        ax.plot(xs, ys, color="#5a189a", linewidth=1.5, linestyle="--",
                alpha=0.8, zorder=3)

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)
