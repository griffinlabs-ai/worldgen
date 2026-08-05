from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.export_map import OccupancyMap
from random_gazebo_world.pipeline import (
    WorldValidationError,
    _validate_free_area_band,
    generate_valid_world,
)


def _sample_config(**overrides: float | int | dict[int, float] | None) -> Config:
    values: dict[str, float | int | dict[int, float] | None] = {
        "world_width": 8.5,
        "world_height": 8.5,
        "min_cell_size": 2.0,
        "max_cell_size": 4.0,
        "min_room_count": 6,
        "max_room_count": 6,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 1.0,
        "gate_width_max": 1.0,
        "passage_width_min": 1.2,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.2,
        "map_resolution": 0.05,
        "random_seed": 0,
        "partition_method": "bsp",
        "passage_geometry_mode": "legacy_orthogonal",
        "min_free_area_m2": 47.0,
        "max_free_area_m2": 53.0,
        "room_count_world_size": {4: 15.0, 6: 8.5, 8: 7.5},
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _occupancy(area_m2: float) -> OccupancyMap:
    resolution = 0.05
    cells = int(round(area_m2 / (resolution * resolution)))
    side = max(1, int(round(cells**0.5)))
    return OccupancyMap(
        data=np.full((side, side), 254, dtype=np.uint8),
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        world_width=side * resolution,
        world_height=side * resolution,
        start_cell=(0, 0),
        goal_cell=(side - 1, side - 1),
        free_cell_count=cells,
        free_area_m2=area_m2,
    )


_BASE_CONFIG_LINES = [
    "world_width: 8.5",
    "world_height: 8.5",
    "min_cell_size: 2.0",
    "max_cell_size: 4.0",
    "min_room_count: 6",
    "max_room_count: 6",
    "wall_height: 2.5",
    "wall_thickness: 0.15",
    "gate_width_min: 1.0",
    "gate_width_max: 1.0",
    "passage_width_min: 1.2",
    "passage_width_max: 1.2",
    "extra_loop_probability: 0.2",
    "map_resolution: 0.05",
    "random_seed: 42",
    "partition_method: bsp",
    "passage_geometry_mode: legacy_orthogonal",
]


def _write_config(tmp_path: Path, extra_lines: list[str]) -> Path:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        "\n".join(_BASE_CONFIG_LINES + extra_lines),
        encoding="utf-8",
    )
    return config_path


def test_free_area_band_requires_both_bounds() -> None:
    with pytest.raises(ConfigError, match="both be set or both omitted"):
        _sample_config(min_free_area_m2=47.0, max_free_area_m2=None)


def test_free_area_band_min_must_not_exceed_max() -> None:
    with pytest.raises(ConfigError, match="free area"):
        _sample_config(min_free_area_m2=53.0, max_free_area_m2=47.0)


def test_room_count_world_size_validation(tmp_path: Path) -> None:
    config = load_config(
        _write_config(
            tmp_path,
            [
                "min_free_area_m2: 47.0",
                "max_free_area_m2: 53.0",
                "room_count_world_size:",
                "  4: 15.0",
                "  6: 8.5",
            ],
        )
    )
    assert config.room_count_world_size == {4: 15.0, 6: 8.5}


def test_validate_free_area_band_accepts_in_band() -> None:
    config = _sample_config()
    _validate_free_area_band(config, _occupancy(50.0))


def test_validate_free_area_band_rejects_out_of_band() -> None:
    config = _sample_config()
    with pytest.raises(WorldValidationError, match=r"free_area_m2 40\.00 outside band \[47\.0, 53\.0\]"):
        _validate_free_area_band(config, _occupancy(40.0))


@pytest.mark.parametrize(
    ("room_count", "seed"),
    [(4, 0), (6, 1), (8, 2)],
)
def test_fixed_area_generation_holds_band(room_count: int, seed: int) -> None:
    config = _sample_config(
        min_room_count=room_count,
        max_room_count=room_count,
        random_seed=seed,
    )
    world = generate_valid_world(config)
    assert 47.0 <= world.occupancy.free_area_m2 <= 53.0
    assert world.config.world_width == world.config.world_height
    assert world.config.min_room_count == room_count
