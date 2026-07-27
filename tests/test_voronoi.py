from __future__ import annotations

import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.partition import PartitionError, generate_partition
from random_gazebo_world.pipeline import generate_valid_world, write_world_outputs
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.voronoi import generate_voronoi_partition


def _voronoi_config(**overrides) -> Config:
    values = {
        "world_width": 20.0,
        "world_height": 20.0,
        "min_cell_size": 2.0,
        "max_cell_size": 6.0,
        "min_room_count": 5,
        "max_room_count": 8,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.5,
        "gate_width_max": 0.8,
        "passage_width_min": 0.5,
        "passage_width_max": 0.8,
        "extra_loop_probability": 0.1,
        "map_resolution": 0.1,
        "random_seed": 7,
        "partition_method": "voronoi",
        "voronoi_seed_count": 18,
        "voronoi_lloyd_iterations": 6,
        "voronoi_min_cell_area": 1.0,
        "voronoi_max_cell_area": 150.0,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def test_voronoi_partition_tiles_world() -> None:
    config = _voronoi_config()
    partition = generate_voronoi_partition(config, create_seeded_rng(config.random_seed))

    assert len(partition.cells) == config.voronoi_seed_count
    total_area = sum(cell.area for cell in partition.cells)
    assert total_area == pytest.approx(config.world_width * config.world_height, abs=1e-3)

    for cell in partition.cells:
        assert not cell.is_rectangle
        assert len(cell.polygon_vertices) >= 3
        assert cell.x_min >= -1e-9
        assert cell.y_min >= -1e-9
        assert cell.x_max <= config.world_width + 1e-9
        assert cell.y_max <= config.world_height + 1e-9


def test_voronoi_cells_do_not_overlap() -> None:
    config = _voronoi_config()
    partition = generate_voronoi_partition(config, create_seeded_rng(config.random_seed))
    cells = partition.cells
    for i, left in enumerate(cells):
        for right in cells[i + 1 :]:
            overlap = left.polygon.intersection(right.polygon).area
            assert overlap <= 1e-4


def test_voronoi_partition_is_deterministic_for_a_seed() -> None:
    config = _voronoi_config()
    first = generate_voronoi_partition(config, create_seeded_rng(123))
    second = generate_voronoi_partition(config, create_seeded_rng(123))
    first_vertices = [cell.polygon_vertices for cell in first.cells]
    second_vertices = [cell.polygon_vertices for cell in second.cells]
    assert first_vertices == second_vertices


def test_voronoi_partition_connected_adjacency() -> None:
    import networkx as nx

    config = _voronoi_config()
    partition = generate_voronoi_partition(config, create_seeded_rng(config.random_seed))
    adjacency = build_adjacency_graph(partition)
    assert nx.is_connected(adjacency.graph)


def test_lloyd_relaxation_improves_area_uniformity() -> None:
    raw = generate_voronoi_partition(
        _voronoi_config(voronoi_lloyd_iterations=0),
        create_seeded_rng(99),
    )
    relaxed = generate_voronoi_partition(
        _voronoi_config(voronoi_lloyd_iterations=12),
        create_seeded_rng(99),
    )
    raw_std = statistics.pstdev([cell.area for cell in raw.cells])
    relaxed_std = statistics.pstdev([cell.area for cell in relaxed.cells])
    assert relaxed_std < raw_std


def test_voronoi_partition_error_on_impossible_area_floor() -> None:
    config = _voronoi_config(voronoi_min_cell_area=500.0, voronoi_max_cell_area=1000.0)
    with pytest.raises(PartitionError):
        generate_voronoi_partition(config, create_seeded_rng(config.random_seed))


def test_generate_partition_dispatches_to_voronoi() -> None:
    config = _voronoi_config()
    partition = generate_partition(config, create_seeded_rng(config.random_seed))
    assert all(not cell.is_rectangle for cell in partition.cells)


def test_end_to_end_voronoi_world(tmp_path: Path) -> None:
    config = _voronoi_config()
    world = generate_valid_world(config, max_attempts=200)
    assert len(world.room_selection.room_cell_ids) >= config.min_room_count

    write_world_outputs(world, tmp_path)
    sdf_path = tmp_path / f"{tmp_path.name}.sdf"
    assert sdf_path.is_file()
    assert (tmp_path / "map.png").is_file()

    tree = ET.parse(sdf_path)
    world_el = tree.getroot().find("world")
    assert world_el is not None
    assert world_el.get("name") == tmp_path.name
    model_names = {model.get("name") for model in world_el.findall("model")}
    assert {"walls", "ground"}.issubset(model_names)
