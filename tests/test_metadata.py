from __future__ import annotations

from pathlib import Path

import numpy as np

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.export_map import OccupancyMap
from random_gazebo_world.geometry import Cell
from random_gazebo_world.metadata import (
    build_layout_document,
    config_from_dict,
    export_layout_json,
    export_metadata_json,
    load_layout_json,
    load_metadata_json,
)
from random_gazebo_world.openings import generate_openings
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    RoomSelection,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
)
from random_gazebo_world.visualize import render_final_floorplan
from random_gazebo_world.walls import generate_walls


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
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _grid_partition() -> Partition:
    cells = (
        Cell.from_origin_size(0, 0.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(1, 5.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(2, 0.0, 5.0, 5.0, 5.0),
        Cell.from_origin_size(3, 5.0, 5.0, 5.0, 5.0),
    )
    return Partition(cells=cells, world_width=10.0, world_height=10.0)


def _build_pipeline(room_ids: set[int], config: Config, seed: int):
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset(room_ids))
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(seed))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(seed + 1000))
    wall_layout = generate_walls(opening_layout, adjacency, config)
    document = build_layout_document(applied, opening_layout, wall_layout)
    return config, selected, wall_layout, document


def test_layout_json_round_trips(tmp_path: Path) -> None:
    config, _, _, document = _build_pipeline({0, 1, 3}, _sample_config(), 42)
    layout_path = tmp_path / "layout.json"
    export_layout_json(layout_path, document)
    loaded = load_layout_json(layout_path)
    assert loaded == document


def test_metadata_json_includes_free_space_when_occupancy_provided(
    tmp_path: Path,
) -> None:
    config, selected, _, document = _build_pipeline({0, 1, 3}, _sample_config(), 7)
    occupancy = OccupancyMap(
        data=np.full((4, 4), 254, dtype=np.uint8),
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        world_width=10.0,
        world_height=10.0,
        start_cell=(0, 0),
        goal_cell=(3, 3),
        free_cell_count=16,
        free_area_m2=16 * 0.05 * 0.05,
    )
    metadata_path = tmp_path / "metadata.json"
    export_metadata_json(
        metadata_path,
        config,
        document,
        selected,
        occupancy=occupancy,
    )

    payload = load_metadata_json(metadata_path)
    assert payload["free_space"] == {
        "free_cells": 16,
        "free_area_m2": 16 * 0.05 * 0.05,
        "resolution": 0.05,
        "map_image": "map.png",
    }


def test_metadata_json_records_seed_and_config(tmp_path: Path) -> None:
    config, selected, _, document = _build_pipeline({0, 1, 3}, _sample_config(random_seed=99), 7)
    metadata_path = tmp_path / "metadata.json"
    export_metadata_json(metadata_path, config, document, selected)

    payload = load_metadata_json(metadata_path)
    assert payload["seed"] == 99
    assert payload["config"]["world_width"] == 20.0
    assert payload["counts"]["rooms"] == 3
    assert config_from_dict(payload["config"]) == config


def test_render_final_floorplan_writes_svg_and_png(tmp_path: Path) -> None:
    _, _, wall_layout, _ = _build_pipeline({0, 1, 3}, _sample_config(), 42)
    output_base = tmp_path / "10_final_floorplan"
    render_final_floorplan(wall_layout, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0


def test_generated_world_metadata_exports(tmp_path: Path) -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(99))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(1001))
    wall_layout = generate_walls(opening_layout, adjacency, config)
    document = build_layout_document(applied, opening_layout, wall_layout)

    export_layout_json(tmp_path / "layout.json", document)
    export_metadata_json(tmp_path / "metadata.json", config, document, selected)
    assert load_layout_json(tmp_path / "layout.json") == document
