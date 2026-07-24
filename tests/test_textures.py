from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.partition import Partition
from random_gazebo_world.textures import (
    FLOOR_TEXTURE_NAME,
    PIXELS_PER_METRE,
    generate_floor_texture,
)
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    RoomSelection,
    SelectedRoomGraph,
)


def _sample_config(**overrides: float | int | bool) -> Config:
    values = {
        "world_width": 10.0,
        "world_height": 8.0,
        "min_cell_size": 2.0,
        "max_cell_size": 6.0,
        "min_room_count": 1,
        "max_room_count": 4,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.8,
        "gate_width_max": 1.2,
        "passage_width_min": 0.8,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
        "textures_enabled": True,
        "floor_tile_size": 0.5,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _applied_layout(*, room_ids: frozenset[int], passage_ids: frozenset[int]) -> AppliedLayout:
    cells = (
        Cell.from_origin_size(0, 0.0, 0.0, 5.0, 8.0),
        Cell.from_origin_size(1, 5.0, 0.0, 5.0, 8.0),
    )
    partition = Partition(cells=cells, world_width=10.0, world_height=8.0)
    selection = RoomSelection(partition=partition, room_cell_ids=room_ids)
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    return AppliedLayout(
        partition=partition,
        room_selection=selection,
        selected_graph=selected,
        passage_cell_ids=passage_ids,
        logical_openings=(),
    )


def test_generate_floor_texture_size_and_filename(tmp_path: Path) -> None:
    config = _sample_config()
    layout = _applied_layout(room_ids=frozenset({0}), passage_ids=frozenset({1}))
    output_path = tmp_path / FLOOR_TEXTURE_NAME
    generate_floor_texture(output_path, config, layout)

    assert output_path.is_file()
    with Image.open(output_path) as image:
        assert image.size == (
            int(round(config.world_width * PIXELS_PER_METRE)),
            int(round(config.world_height * PIXELS_PER_METRE)),
        )


def test_generate_floor_texture_is_deterministic(tmp_path: Path) -> None:
    config = _sample_config(random_seed=99)
    layout = _applied_layout(room_ids=frozenset({0}), passage_ids=frozenset({1}))
    first_path = tmp_path / "first.png"
    second_path = tmp_path / "second.png"
    generate_floor_texture(first_path, config, layout)
    generate_floor_texture(second_path, config, layout)
    assert first_path.read_bytes() == second_path.read_bytes()


def test_generate_floor_texture_tints_regions_differently(tmp_path: Path) -> None:
    config = _sample_config(random_seed=7)
    layout = _applied_layout(room_ids=frozenset({0}), passage_ids=frozenset({1}))
    output_path = tmp_path / FLOOR_TEXTURE_NAME
    generate_floor_texture(output_path, config, layout)

    image = np.asarray(Image.open(output_path))
    room_pixel = image[400, 100]
    passage_pixel = image[400, 480]
    assert not np.array_equal(room_pixel, passage_pixel)


def test_floor_tile_size_must_be_positive(tmp_path: Path) -> None:
    config_path = tmp_path / "bad.yaml"
    config_path.write_text(
        "\n".join(
            [
                "layout_mode: corridor",
                "corridor_length: 10.0",
                "corridor_width: 2.0",
                "entrance_width: 0.8",
                "room_width_min: 2.0",
                "room_width_max: 3.0",
                "room_depth_min: 2.0",
                "room_depth_max: 3.0",
                "wall_height: 2.5",
                "wall_thickness: 0.15",
                "map_resolution: 0.05",
                "random_seed: 1",
                "floor_tile_size: 0",
            ]
        ),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="floor_tile_size"):
        load_config(config_path)


def test_corridor_config_loads_texture_settings() -> None:
    config = load_config(Path("configs/corridor.yaml"))
    assert config.textures_enabled is True
    assert config.floor_tile_size == pytest.approx(0.5)
