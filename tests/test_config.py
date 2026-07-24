from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.rng import create_seeded_rng


def test_load_default_config() -> None:
    config = load_config(Path("configs/default.yaml"))
    assert config.world_width == 20.0
    assert config.random_seed == 10667


def test_invalid_config_raises_clear_error(tmp_path: Path) -> None:
    bad_config = tmp_path / "bad.yaml"
    bad_config.write_text(
        "\n".join(
            [
                "world_width: 20.0",
                "world_height: 20.0",
                "min_cell_size: 8.0",
                "max_cell_size: 4.0",
                "min_room_count: 3",
                "max_room_count: 8",
                "wall_height: 2.5",
                "wall_thickness: 0.15",
                "gate_width_min: 0.8",
                "gate_width_max: 1.2",
                "passage_width_min: 0.8",
                "passage_width_max: 1.2",
                "extra_loop_probability: 0.2",
                "map_resolution: 0.05",
                "random_seed: 42",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="min cell size"):
        load_config(bad_config)


_BASE_CONFIG_LINES = [
    "world_width: 20.0",
    "world_height: 20.0",
    "min_cell_size: 2.0",
    "max_cell_size: 6.0",
    "min_room_count: 3",
    "max_room_count: 8",
    "wall_height: 2.5",
    "wall_thickness: 0.15",
    "gate_width_min: 0.8",
    "gate_width_max: 1.2",
    "passage_width_min: 0.8",
    "passage_width_max: 1.2",
    "extra_loop_probability: 0.2",
    "map_resolution: 0.05",
    "random_seed: 42",
]


def _write_config(tmp_path: Path, extra_lines: list[str]) -> Path:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        "\n".join(_BASE_CONFIG_LINES + extra_lines), encoding="utf-8"
    )
    return config_path


def test_passage_constraint_fields_default_when_omitted(tmp_path: Path) -> None:
    config = load_config(_write_config(tmp_path, []))
    assert config.max_openings_per_passage_edge == 1
    assert config.max_open_edges_per_passage == 4
    assert config.max_attempts == 100000
    assert config.max_selection_attempts == 64
    assert config.ground_thickness == 0.1
    assert config.passage_geometry_mode == "curved"


def test_legacy_orthogonal_with_voronoi_raises(tmp_path: Path) -> None:
    config_path = _write_config(
        tmp_path,
        [
            "partition_method: voronoi",
            "passage_geometry_mode: legacy_orthogonal",
        ],
    )
    with pytest.raises(ConfigError, match="legacy_orthogonal"):
        load_config(config_path)


def test_max_selection_attempts_below_one_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["max_selection_attempts: 0"])
    with pytest.raises(ConfigError, match="max_selection_attempts"):
        load_config(config_path)


def test_ground_thickness_below_zero_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["ground_thickness: 0"])
    with pytest.raises(ConfigError, match="ground_thickness"):
        load_config(config_path)


def test_floor_tile_size_below_zero_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["floor_tile_size: 0"])
    with pytest.raises(ConfigError, match="floor_tile_size"):
        load_config(config_path)


def test_textures_enabled_defaults_false(tmp_path: Path) -> None:
    config = load_config(_write_config(tmp_path, []))
    assert config.textures_enabled is False
    assert config.floor_tile_size == pytest.approx(0.5)
    assert config.fixture_mode == "none"
    assert config.fixture_models_dir is None
    assert config.fixture_toilet_offset_x == pytest.approx(-0.458)
    assert config.fixture_toilet_offset_y == pytest.approx(0.0)
    assert config.fixture_toilet_offset_z == pytest.approx(0.0)
    assert config.fixture_toilet_offset_yaw == pytest.approx(0.0)
    assert config.fixture_urinal_offset_x == pytest.approx(0.0)
    assert config.fixture_basin_offset_yaw == pytest.approx(0.0)
    assert config.cubicle_door_width == pytest.approx(0.65)
    assert config.cubicle_wall_height is None


def test_cubicle_settings_load_from_yaml(tmp_path: Path) -> None:
    config = load_config(
        _write_config(
            tmp_path,
            [
                "cubicle_door_width: 0.7",
                "cubicle_wall_height: 2.0",
            ],
        )
    )
    assert config.cubicle_door_width == pytest.approx(0.7)
    assert config.cubicle_wall_height == pytest.approx(2.0)


def test_cubicle_door_width_too_large_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["cubicle_door_width: 1.4"])
    with pytest.raises(ConfigError, match="cubicle_door_width"):
        load_config(config_path)


def test_cubicle_wall_height_above_wall_height_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["cubicle_wall_height: 3.0"])
    with pytest.raises(ConfigError, match="cubicle_wall_height"):
        load_config(config_path)


def test_cubicle_wall_height_non_positive_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["cubicle_wall_height: 0"])
    with pytest.raises(ConfigError, match="cubicle_wall_height"):
        load_config(config_path)


def test_fixture_visual_offsets_load_from_yaml(tmp_path: Path) -> None:
    config = load_config(
        _write_config(
            tmp_path,
            [
                "fixture_toilet_offset_x: -0.25",
                "fixture_toilet_offset_y: 0.1",
                "fixture_toilet_offset_z: -0.05",
                "fixture_toilet_offset_yaw: 0.2",
                "fixture_urinal_offset_x: 0.01",
                "fixture_basin_offset_yaw: -1.5",
            ],
        )
    )
    assert config.fixture_toilet_offset_x == pytest.approx(-0.25)
    assert config.fixture_toilet_offset_y == pytest.approx(0.1)
    assert config.fixture_toilet_offset_z == pytest.approx(-0.05)
    assert config.fixture_toilet_offset_yaw == pytest.approx(0.2)
    assert config.fixture_urinal_offset_x == pytest.approx(0.01)
    assert config.fixture_basin_offset_yaw == pytest.approx(-1.5)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("fixture_toilet_offset_x", "nan"),
        ("fixture_urinal_offset_y", "inf"),
        ("fixture_basin_offset_z", "-inf"),
        ("fixture_toilet_offset_yaw", "true"),
    ],
)
def test_non_finite_fixture_offset_raises(
    tmp_path: Path,
    field: str,
    value: str,
) -> None:
    config_path = _write_config(tmp_path, [f"{field}: {value}"])
    with pytest.raises(ConfigError, match=field):
        load_config(config_path)


def test_max_attempts_below_one_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["max_attempts: 0"])
    with pytest.raises(ConfigError, match="max_attempts"):
        load_config(config_path)


def test_open_edges_below_two_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["max_open_edges_per_passage: 1"])
    with pytest.raises(ConfigError, match="max_open_edges_per_passage"):
        load_config(config_path)


def test_open_edges_above_four_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["max_open_edges_per_passage: 5"])
    with pytest.raises(ConfigError, match="max_open_edges_per_passage"):
        load_config(config_path)


def test_openings_per_edge_below_one_raises(tmp_path: Path) -> None:
    config_path = _write_config(tmp_path, ["max_openings_per_passage_edge: 0"])
    with pytest.raises(ConfigError, match="max_openings_per_passage_edge"):
        load_config(config_path)


def test_missing_field_raises_clear_error(tmp_path: Path) -> None:
    bad_config = tmp_path / "missing.yaml"
    bad_config.write_text("world_width: 20.0\n", encoding="utf-8")

    with pytest.raises(ConfigError, match="Missing required config field"):
        load_config(bad_config)


def test_same_seed_produces_identical_rng_state() -> None:
    first = [create_seeded_rng(42).random() for _ in range(5)]
    second = [create_seeded_rng(42).random() for _ in range(5)]
    assert first == second


def test_with_seed_override() -> None:
    config = load_config(Path("configs/default.yaml"))
    updated = config.with_seed(99)
    assert updated.random_seed == 99
    assert config.random_seed == 10667
