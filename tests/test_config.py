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
    assert config.cubicle_wall_color == pytest.approx((0.36, 0.47, 0.55))
    assert config.lighting_mode == "directional"
    assert config.light_height == pytest.approx(2.2)
    assert config.corridor_light_spacing == pytest.approx(8.0)
    assert config.scene_ambient == pytest.approx((0.28, 0.28, 0.28))
    assert config.scene_background == pytest.approx((0.7, 0.7, 0.7))
    assert config.physics_profile == "ignored"
    assert config.counter_specular == pytest.approx((0.4, 0.4, 0.4))
    assert config.fixture_friction_mu == pytest.approx(10000.2)
    assert config.fixture_toilet_count_min == 2
    assert config.fixture_toilet_count_max == 5
    assert config.fixture_urinal_count_min == 2
    assert config.fixture_urinal_count_max == 5
    assert config.fixture_basin_count_min == 1
    assert config.fixture_basin_count_max == 3


def test_fixture_count_settings_load_from_yaml(tmp_path: Path) -> None:
    config = load_config(
        _write_config(
            tmp_path,
            [
                "fixture_toilet_count_min: 3",
                "fixture_toilet_count_max: 4",
                "fixture_urinal_count_min: 2",
                "fixture_urinal_count_max: 6",
                "fixture_basin_count_min: 2",
                "fixture_basin_count_max: 2",
            ],
        )
    )
    assert config.fixture_toilet_count_min == 3
    assert config.fixture_toilet_count_max == 4
    assert config.fixture_urinal_count_min == 2
    assert config.fixture_urinal_count_max == 6
    assert config.fixture_basin_count_min == 2
    assert config.fixture_basin_count_max == 2


@pytest.mark.parametrize(
    ("field", "value", "match"),
    [
        ("fixture_toilet_count_min", "1.5", "fixture_toilet_count_min"),
        ("fixture_urinal_count_max", "true", "fixture_urinal_count_max"),
        ("fixture_basin_count_min", "0", "fixture_basin_count_min"),
        ("fixture_basin_count_max", "-1", "fixture_basin_count_max"),
        ("fixture_toilet_count_min", "6", "fixture toilet count"),
    ],
)
def test_fixture_count_validation_raises(
    tmp_path: Path,
    field: str,
    value: str,
    match: str,
) -> None:
    config_path = _write_config(tmp_path, [f"{field}: {value}"])
    with pytest.raises(ConfigError, match=match):
        load_config(config_path)


def test_fixture_count_feasibility_raises_in_corridor_mode(tmp_path: Path) -> None:
    if not Path(
        "/home/griffinlabs/tcr/ros_ws/src/utils/tcr_ignition/models"
    ).is_dir():
        pytest.skip("fixture models directory not available")
    config_path = tmp_path / "corridor_infeasible.yaml"
    config_path.write_text(
        "\n".join(
            [
                "layout_mode: corridor",
                "corridor_length: 10.0",
                "corridor_width: 2.0",
                "entrance_width: 0.9",
                "room_width_min: 2.0",
                "room_width_max: 2.0",
                "room_depth_min: 2.0",
                "room_depth_max: 2.0",
                "wall_height: 2.5",
                "wall_thickness: 0.15",
                "map_resolution: 0.05",
                "random_seed: 42",
                "fixture_mode: restroom_clusters",
                "fixture_models_dir: /home/griffinlabs/tcr/ros_ws/src/utils/tcr_ignition/models",
                "fixture_toilet_count_min: 2",
            ]
        ),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="fixture toilet"):
        load_config(config_path)


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


def test_visual_physics_settings_load_from_yaml(tmp_path: Path) -> None:
    config = load_config(
        _write_config(
            tmp_path,
            [
                "cubicle_wall_color: [0.1, 0.2, 0.3]",
                "lighting_mode: point",
                "light_height: 3.0",
                "corridor_light_spacing: 4.0",
                "scene_ambient: [0.1, 0.1, 0.1]",
                "scene_background: [0.5, 0.5, 0.5]",
                "physics_profile: ode",
                "counter_specular: [0.2, 0.2, 0.2]",
                "fixture_friction_mu: 42.0",
            ],
        )
    )
    assert config.cubicle_wall_color == pytest.approx((0.1, 0.2, 0.3))
    assert config.lighting_mode == "point"
    assert config.light_height == pytest.approx(3.0)
    assert config.corridor_light_spacing == pytest.approx(4.0)
    assert config.scene_ambient == pytest.approx((0.1, 0.1, 0.1))
    assert config.scene_background == pytest.approx((0.5, 0.5, 0.5))
    assert config.physics_profile == "ode"
    assert config.counter_specular == pytest.approx((0.2, 0.2, 0.2))
    assert config.fixture_friction_mu == pytest.approx(42.0)


@pytest.mark.parametrize(
    ("field", "value", "match"),
    [
        ("lighting_mode", "spot", "lighting_mode"),
        ("physics_profile", "bullet", "physics_profile"),
        ("light_height", "0", "light_height"),
        ("corridor_light_spacing", "-1", "corridor_light_spacing"),
        ("fixture_friction_mu", "0", "fixture_friction_mu"),
        ("scene_ambient", "[1.5, 0.5, 0.5]", "scene_ambient\\[0\\]"),
        ("cubicle_wall_color", "[0.5, 0.5]", "cubicle_wall_color"),
    ],
)
def test_visual_physics_validation_raises(
    tmp_path: Path,
    field: str,
    value: str,
    match: str,
) -> None:
    config_path = _write_config(tmp_path, [f"{field}: {value}"])
    with pytest.raises(ConfigError, match=match):
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
