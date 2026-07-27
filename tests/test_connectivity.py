from __future__ import annotations

import json
from pathlib import Path

import networkx as nx
import numpy as np
import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.export_map import OccupancyMap
from random_gazebo_world.fixtures import EMPTY_FIXTURE_LAYOUT
from random_gazebo_world.geometry import Cell
from random_gazebo_world.metadata import build_layout_document, load_layout_json
from random_gazebo_world.openings import OpeningLayout
from random_gazebo_world.partition import Partition
from random_gazebo_world.passage_geometry import PassageGeometryLayout
from random_gazebo_world.pipeline import (
    REQUIRED_DEBUG_STAGES,
    GeneratedWorld,
    WorldValidationError,
    generate_valid_world,
    required_outputs,
    validate_world_connectivity,
    write_world_outputs,
)
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    RoomSelection,
    SelectedRoomGraph,
)
from random_gazebo_world.walls import WallLayout


def _sample_config(**overrides: float | int) -> Config:
    values = {
        "world_width": 20.0,
        "world_height": 20.0,
        "min_cell_size": 2.0,
        "max_cell_size": 6.0,
        "min_room_count": 3,
        "max_room_count": 8,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.8,
        "gate_width_max": 1.2,
        "passage_width_min": 0.8,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.2,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _broken_world() -> GeneratedWorld:
    cells = (
        Cell.from_origin_size(0, 0.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(1, 5.0, 0.0, 5.0, 5.0),
    )
    partition = Partition(cells=cells, world_width=10.0, world_height=5.0)
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset({0, 1}))
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    applied = AppliedLayout(
        partition=partition,
        room_selection=selection,
        selected_graph=selected,
        passage_cell_ids=frozenset(),
        logical_openings=(),
    )
    opening_layout = OpeningLayout(applied_layout=applied, openings=())
    passage_geometry = PassageGeometryLayout(opening_layout=opening_layout, cells=())
    wall_layout = WallLayout(
        opening_layout=opening_layout,
        segments=(),
        passage_geometry=passage_geometry,
    )
    occupancy = OccupancyMap(
        data=np.full((1, 1), 254, dtype=np.uint8),
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        world_width=10.0,
        world_height=5.0,
        start_cell=(0, 0),
        goal_cell=(0, 0),
    )
    return GeneratedWorld(
        config=_sample_config(min_room_count=2, max_room_count=2),
        partition=partition,
        adjacency=adjacency,
        room_selection=selection,
        candidates=candidates,
        selected_graph=selected,
        applied_layout=applied,
        opening_layout=opening_layout,
        passage_geometry=passage_geometry,
        wall_layout=wall_layout,
        fixture_layout=EMPTY_FIXTURE_LAYOUT,
        occupancy=occupancy,
        layout_document=build_layout_document(applied, opening_layout, wall_layout),
        attempt=0,
    )


def test_generate_valid_world_connects_all_rooms() -> None:
    world = generate_valid_world(_sample_config(random_seed=42))
    validate_world_connectivity(world)

    room_graph = nx.Graph()
    room_graph.add_nodes_from(world.room_selection.room_cell_ids)
    for connection in world.selected_graph.connections:
        room_graph.add_edge(connection.room_a_id, connection.room_b_id)
    assert nx.is_connected(room_graph)


def test_cli_output_set_is_complete(tmp_path: Path) -> None:
    world = generate_valid_world(_sample_config(random_seed=7))
    out_dir = tmp_path / "world"
    write_world_outputs(world, out_dir)

    for relative_path in required_outputs(out_dir):
        assert (out_dir / relative_path).is_file()

    debug_dir = out_dir / "debug"
    for stage in REQUIRED_DEBUG_STAGES:
        assert (debug_dir / f"{stage}.png").is_file()
        if stage != "09_occupancy_map_preview":
            assert (debug_dir / f"{stage}.svg").is_file()


def test_same_seed_produces_structurally_identical_layout() -> None:
    config = _sample_config(random_seed=123)
    first = generate_valid_world(config)
    second = generate_valid_world(config)
    assert first.layout_document == second.layout_document
    assert first.attempt == second.attempt


def test_write_world_outputs_round_trips_layout_json(tmp_path: Path) -> None:
    world = generate_valid_world(_sample_config(random_seed=55))
    out_dir = tmp_path / "world_55"
    write_world_outputs(world, out_dir)
    loaded = load_layout_json(out_dir / "layout.json")
    assert loaded == world.layout_document


def test_validate_world_connectivity_rejects_disconnected_candidates() -> None:
    with pytest.raises(WorldValidationError, match="cannot be connected"):
        validate_world_connectivity(_broken_world())


def test_metadata_records_effective_generation_seed(tmp_path: Path) -> None:
    world = generate_valid_world(_sample_config(random_seed=42))
    write_world_outputs(world, tmp_path / "world")
    metadata = json.loads((tmp_path / "world" / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["seed"] == 42 + world.attempt
