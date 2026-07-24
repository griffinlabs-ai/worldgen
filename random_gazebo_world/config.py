from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml

from random_gazebo_world.geometry import EPS


class ConfigError(ValueError):
    """Raised when a config file is invalid or fails validation."""


@dataclass(frozen=True)
class Config:
    world_width: float
    world_height: float
    min_cell_size: float
    max_cell_size: float
    min_room_count: int
    max_room_count: int
    wall_height: float
    wall_thickness: float
    gate_width_min: float
    gate_width_max: float
    passage_width_min: float
    passage_width_max: float
    extra_loop_probability: float
    map_resolution: float
    random_seed: int
    max_openings_per_passage_edge: int = 1
    max_open_edges_per_passage: int = 4
    max_attempts: int = 100000
    max_selection_attempts: int = 64
    ground_thickness: float = 0.1
    partition_method: str = "bsp"
    voronoi_seed_count: int = 16
    voronoi_lloyd_iterations: int = 8
    voronoi_min_cell_area: float = 1.0
    voronoi_max_cell_area: float = 64.0
    passage_geometry_mode: str = "curved"
    layout_mode: str = "partition"
    corridor_width: float | None = None
    corridor_length: float | None = None
    entrance_width: float | None = None
    room_width_min: float | None = None
    room_width_max: float | None = None
    room_depth_min: float | None = None
    room_depth_max: float | None = None
    textures_enabled: bool = False
    floor_tile_size: float = 0.5
    fixture_mode: str = "none"
    fixture_models_dir: str | None = None
    fixture_toilet_offset_x: float = -0.458
    fixture_toilet_offset_y: float = 0.0
    fixture_toilet_offset_z: float = 0.0
    fixture_toilet_offset_yaw: float = 0.0
    fixture_urinal_offset_x: float = 0.0
    fixture_urinal_offset_y: float = 0.0
    fixture_urinal_offset_z: float = 0.0
    fixture_urinal_offset_yaw: float = 0.0
    fixture_basin_offset_x: float = 0.0
    fixture_basin_offset_y: float = 0.0
    fixture_basin_offset_z: float = 0.0
    fixture_basin_offset_yaw: float = 0.0
    fixture_toilet_count_min: int = 2
    fixture_toilet_count_max: int = 5
    fixture_urinal_count_min: int = 2
    fixture_urinal_count_max: int = 5
    fixture_basin_count_min: int = 1
    fixture_basin_count_max: int = 3
    cubicle_door_width: float = 0.65
    cubicle_wall_height: float | None = None
    cubicle_wall_color: tuple[float, float, float] = (0.36, 0.47, 0.55)
    lighting_mode: str = "directional"
    light_height: float = 2.2
    corridor_light_spacing: float = 8.0
    scene_ambient: tuple[float, float, float] = (0.28, 0.28, 0.28)
    scene_background: tuple[float, float, float] = (0.7, 0.7, 0.7)
    physics_profile: str = "ignored"
    counter_specular: tuple[float, float, float] = (0.4, 0.4, 0.4)
    fixture_friction_mu: float = 10000.2

    def validate(self) -> None:
        if self.layout_mode not in ("partition", "corridor"):
            raise ConfigError(
                "layout_mode must be 'partition' or 'corridor', got "
                f"{self.layout_mode!r}"
            )
        if self.layout_mode == "corridor":
            self._validate_corridor_mode()
            self._validate_fixture_settings()
            return
        self._validate_partition_mode()
        self._validate_fixture_settings()

    def _validate_partition_mode(self) -> None:
        _require_positive(self.world_width, "world_width")
        _require_positive(self.world_height, "world_height")
        _require_positive(self.min_cell_size, "min_cell_size")
        _require_positive(self.max_cell_size, "max_cell_size")
        _require_min_max(self.min_cell_size, self.max_cell_size, "cell size")
        _require_positive_int(self.min_room_count, "min_room_count")
        _require_positive_int(self.max_room_count, "max_room_count")
        _require_min_max(self.min_room_count, self.max_room_count, "room count")
        _require_positive(self.wall_height, "wall_height")
        _require_positive(self.wall_thickness, "wall_thickness")
        _require_positive(self.gate_width_min, "gate_width_min")
        _require_positive(self.gate_width_max, "gate_width_max")
        _require_min_max(self.gate_width_min, self.gate_width_max, "gate width")
        _require_positive(self.passage_width_min, "passage_width_min")
        _require_positive(self.passage_width_max, "passage_width_max")
        _require_min_max(
            self.passage_width_min, self.passage_width_max, "passage width"
        )
        _require_probability(self.extra_loop_probability, "extra_loop_probability")
        _require_positive(self.map_resolution, "map_resolution")
        _require_positive_int(
            self.max_openings_per_passage_edge, "max_openings_per_passage_edge"
        )
        _require_positive_int(
            self.max_open_edges_per_passage, "max_open_edges_per_passage"
        )
        if self.partition_method not in ("bsp", "voronoi"):
            raise ConfigError(
                "partition_method must be 'bsp' or 'voronoi', got "
                f"{self.partition_method!r}"
            )
        if self.partition_method == "bsp":
            if not 2 <= self.max_open_edges_per_passage <= 4:
                raise ConfigError(
                    "max_open_edges_per_passage must be between 2 and 4 for bsp, got "
                    f"{self.max_open_edges_per_passage}"
                )
        elif self.max_open_edges_per_passage < 2:
            raise ConfigError(
                "max_open_edges_per_passage must be at least 2, got "
                f"{self.max_open_edges_per_passage}"
            )
        _require_positive_int(self.voronoi_seed_count, "voronoi_seed_count")
        _require_non_negative_int(
            self.voronoi_lloyd_iterations, "voronoi_lloyd_iterations"
        )
        _require_positive(self.voronoi_min_cell_area, "voronoi_min_cell_area")
        _require_positive(self.voronoi_max_cell_area, "voronoi_max_cell_area")
        _require_min_max(
            self.voronoi_min_cell_area, self.voronoi_max_cell_area, "voronoi cell area"
        )
        _require_positive_int(self.max_attempts, "max_attempts")
        _require_positive_int(
            self.max_selection_attempts, "max_selection_attempts"
        )
        _require_positive(self.ground_thickness, "ground_thickness")
        _require_positive(self.floor_tile_size, "floor_tile_size")
        _validate_fixture_visual_offsets(self)
        _validate_cubicle_settings(self)
        _validate_visual_physics_settings(self)
        if self.passage_geometry_mode not in ("curved", "legacy_orthogonal"):
            raise ConfigError(
                "passage_geometry_mode must be 'curved' or 'legacy_orthogonal', got "
                f"{self.passage_geometry_mode!r}"
            )
        if (
            self.passage_geometry_mode == "legacy_orthogonal"
            and self.partition_method != "bsp"
        ):
            raise ConfigError(
                "passage_geometry_mode 'legacy_orthogonal' requires "
                "partition_method 'bsp', got "
                f"{self.partition_method!r}"
            )

    def _validate_fixture_settings(self) -> None:
        if self.fixture_mode not in ("none", "restroom_clusters"):
            raise ConfigError(
                "fixture_mode must be 'none' or 'restroom_clusters', got "
                f"{self.fixture_mode!r}"
            )
        _validate_fixture_count_settings(self)
        if self.fixture_mode == "none":
            return
        _validate_fixture_count_feasibility(self)
        if not self.fixture_models_dir:
            raise ConfigError(
                "fixture_models_dir is required when fixture_mode is not 'none'"
            )
        models_dir = Path(self.fixture_models_dir)
        if not models_dir.is_dir():
            raise ConfigError(
                f"fixture_models_dir must be an existing directory, got {models_dir}"
            )
        if (
            self.fixture_mode == "restroom_clusters"
            and self.partition_method == "voronoi"
        ):
            raise ConfigError(
                "fixture_mode 'restroom_clusters' is not supported with "
                "partition_method 'voronoi'"
            )

    def _validate_corridor_mode(self) -> None:
        corridor_fields = {
            "corridor_width": self.corridor_width,
            "corridor_length": self.corridor_length,
            "entrance_width": self.entrance_width,
            "room_width_min": self.room_width_min,
            "room_width_max": self.room_width_max,
            "room_depth_min": self.room_depth_min,
            "room_depth_max": self.room_depth_max,
        }
        for name, value in corridor_fields.items():
            if value is None:
                raise ConfigError(f"{name} is required when layout_mode is 'corridor'")
            _require_positive(value, name)

        assert self.corridor_width is not None
        assert self.corridor_length is not None
        assert self.entrance_width is not None
        assert self.room_width_min is not None
        assert self.room_width_max is not None
        assert self.room_depth_min is not None
        assert self.room_depth_max is not None

        _require_min_max(self.room_width_min, self.room_width_max, "room width")
        _require_min_max(self.room_depth_min, self.room_depth_max, "room depth")
        _require_positive(self.wall_height, "wall_height")
        _require_positive(self.wall_thickness, "wall_thickness")
        _require_probability(self.extra_loop_probability, "extra_loop_probability")
        _require_positive(self.map_resolution, "map_resolution")
        _require_positive_int(self.max_attempts, "max_attempts")
        _require_positive(self.ground_thickness, "ground_thickness")
        _require_positive(self.floor_tile_size, "floor_tile_size")
        _validate_fixture_visual_offsets(self)
        _validate_cubicle_settings(self)
        _validate_visual_physics_settings(self)

        min_room_width = self.entrance_width + 2.0 * self.wall_thickness
        if self.room_width_min + EPS < min_room_width:
            raise ConfigError(
                "room_width_min must be >= entrance_width + 2 * wall_thickness "
                f"({min_room_width}), got {self.room_width_min}"
            )

    def with_seed(self, seed: int) -> Config:
        return replace(self, random_seed=seed)


def load_config(path: Path | str) -> Config:
    config_path = Path(path)
    if not config_path.is_file():
        raise ConfigError(f"Config file not found: {config_path}")

    with config_path.open(encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    if not isinstance(raw, dict):
        raise ConfigError(f"Config root must be a mapping: {config_path}")

    layout_mode = raw.get("layout_mode", "partition")
    if layout_mode not in ("partition", "corridor"):
        raise ConfigError(
            f"layout_mode must be 'partition' or 'corridor', got {layout_mode!r}"
        )

    is_corridor = layout_mode == "corridor"

    try:
        config = Config(
            world_width=raw.get("world_width", 1.0) if is_corridor else _require_field(raw, "world_width"),
            world_height=raw.get("world_height", 1.0) if is_corridor else _require_field(raw, "world_height"),
            min_cell_size=raw.get("min_cell_size", 1.0) if is_corridor else _require_field(raw, "min_cell_size"),
            max_cell_size=raw.get("max_cell_size", 10.0) if is_corridor else _require_field(raw, "max_cell_size"),
            min_room_count=raw.get("min_room_count", 1) if is_corridor else _require_field(raw, "min_room_count"),
            max_room_count=raw.get("max_room_count", 100) if is_corridor else _require_field(raw, "max_room_count"),
            wall_height=_require_field(raw, "wall_height"),
            wall_thickness=_require_field(raw, "wall_thickness"),
            gate_width_min=(
                raw.get("gate_width_min", raw.get("entrance_width", 0.8))
                if is_corridor
                else _require_field(raw, "gate_width_min")
            ),
            gate_width_max=(
                raw.get("gate_width_max", raw.get("entrance_width", 1.2))
                if is_corridor
                else _require_field(raw, "gate_width_max")
            ),
            passage_width_min=(
                raw.get("passage_width_min", 1.0)
                if is_corridor
                else _require_field(raw, "passage_width_min")
            ),
            passage_width_max=(
                raw.get("passage_width_max", 1.0)
                if is_corridor
                else _require_field(raw, "passage_width_max")
            ),
            extra_loop_probability=(
                raw.get("extra_loop_probability", 0.0)
                if is_corridor
                else _require_field(raw, "extra_loop_probability")
            ),
            map_resolution=_require_field(raw, "map_resolution"),
            random_seed=_require_field(raw, "random_seed"),
            max_openings_per_passage_edge=raw.get(
                "max_openings_per_passage_edge", 1
            ),
            max_open_edges_per_passage=raw.get("max_open_edges_per_passage", 4),
            max_attempts=raw.get("max_attempts", 100000),
            max_selection_attempts=raw.get("max_selection_attempts", 64),
            ground_thickness=raw.get("ground_thickness", 0.1),
            partition_method=raw.get("partition_method", "bsp"),
            voronoi_seed_count=raw.get("voronoi_seed_count", 16),
            voronoi_lloyd_iterations=raw.get("voronoi_lloyd_iterations", 8),
            voronoi_min_cell_area=raw.get("voronoi_min_cell_area", 1.0),
            voronoi_max_cell_area=raw.get("voronoi_max_cell_area", 64.0),
            passage_geometry_mode=raw.get("passage_geometry_mode", "curved"),
            layout_mode=layout_mode,
            corridor_width=raw.get("corridor_width"),
            corridor_length=raw.get("corridor_length"),
            entrance_width=raw.get("entrance_width"),
            room_width_min=raw.get("room_width_min"),
            room_width_max=raw.get("room_width_max"),
            room_depth_min=raw.get("room_depth_min"),
            room_depth_max=raw.get("room_depth_max"),
            textures_enabled=raw.get("textures_enabled", False),
            floor_tile_size=raw.get("floor_tile_size", 0.5),
            fixture_mode=raw.get("fixture_mode", "none"),
            fixture_models_dir=raw.get("fixture_models_dir"),
            fixture_toilet_offset_x=raw.get("fixture_toilet_offset_x", -0.458),
            fixture_toilet_offset_y=raw.get("fixture_toilet_offset_y", 0.0),
            fixture_toilet_offset_z=raw.get("fixture_toilet_offset_z", 0.0),
            fixture_toilet_offset_yaw=raw.get("fixture_toilet_offset_yaw", 0.0),
            fixture_urinal_offset_x=raw.get("fixture_urinal_offset_x", 0.0),
            fixture_urinal_offset_y=raw.get("fixture_urinal_offset_y", 0.0),
            fixture_urinal_offset_z=raw.get("fixture_urinal_offset_z", 0.0),
            fixture_urinal_offset_yaw=raw.get("fixture_urinal_offset_yaw", 0.0),
            fixture_basin_offset_x=raw.get("fixture_basin_offset_x", 0.0),
            fixture_basin_offset_y=raw.get("fixture_basin_offset_y", 0.0),
            fixture_basin_offset_z=raw.get("fixture_basin_offset_z", 0.0),
            fixture_basin_offset_yaw=raw.get("fixture_basin_offset_yaw", 0.0),
            fixture_toilet_count_min=raw.get("fixture_toilet_count_min", 2),
            fixture_toilet_count_max=raw.get("fixture_toilet_count_max", 5),
            fixture_urinal_count_min=raw.get("fixture_urinal_count_min", 2),
            fixture_urinal_count_max=raw.get("fixture_urinal_count_max", 5),
            fixture_basin_count_min=raw.get("fixture_basin_count_min", 1),
            fixture_basin_count_max=raw.get("fixture_basin_count_max", 3),
            cubicle_door_width=raw.get("cubicle_door_width", 0.65),
            cubicle_wall_height=raw.get("cubicle_wall_height"),
            cubicle_wall_color=_load_rgb(
                raw, "cubicle_wall_color", (0.36, 0.47, 0.55)
            ),
            lighting_mode=raw.get("lighting_mode", "directional"),
            light_height=raw.get("light_height", 2.2),
            corridor_light_spacing=raw.get("corridor_light_spacing", 8.0),
            scene_ambient=_load_rgb(raw, "scene_ambient", (0.28, 0.28, 0.28)),
            scene_background=_load_rgb(raw, "scene_background", (0.7, 0.7, 0.7)),
            physics_profile=raw.get("physics_profile", "ignored"),
            counter_specular=_load_rgb(raw, "counter_specular", (0.4, 0.4, 0.4)),
            fixture_friction_mu=raw.get("fixture_friction_mu", 10000.2),
        )
    except KeyError as exc:
        raise ConfigError(f"Missing required config field: {exc.args[0]}") from exc
    except TypeError as exc:
        raise ConfigError(f"Invalid config value in {config_path}: {exc}") from exc

    config.validate()
    return config


def _require_field(raw: dict[str, Any], name: str) -> Any:
    if name not in raw:
        raise KeyError(name)
    return raw[name]


def _require_positive(value: float, name: str) -> None:
    if value <= 0:
        raise ConfigError(f"{name} must be positive, got {value}")


def _require_positive_int(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ConfigError(f"{name} must be an integer, got {value!r}")
    if value <= 0:
        raise ConfigError(f"{name} must be positive, got {value}")


def _require_non_negative_int(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ConfigError(f"{name} must be an integer, got {value!r}")
    if value < 0:
        raise ConfigError(f"{name} must be non-negative, got {value}")


def _require_min_max(min_value: float, max_value: float, label: str) -> None:
    if min_value > max_value:
        raise ConfigError(
            f"min {label} ({min_value}) must be <= max {label} ({max_value})"
        )


def _require_probability(value: float, name: str) -> None:
    if not 0.0 <= value <= 1.0:
        raise ConfigError(f"{name} must be between 0 and 1, got {value}")


def _require_finite_number(value: float, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a finite numeric value, got {value!r}")
    numeric = float(value)
    if not numeric == numeric or numeric in (float("inf"), float("-inf")):
        raise ConfigError(f"{name} must be a finite numeric value, got {value!r}")


def _validate_fixture_count_settings(config: Config) -> None:
    for kind in ("toilet", "urinal", "basin"):
        min_name = f"fixture_{kind}_count_min"
        max_name = f"fixture_{kind}_count_max"
        count_min = getattr(config, min_name)
        count_max = getattr(config, max_name)
        _require_positive_int(count_min, min_name)
        _require_positive_int(count_max, max_name)
        _require_min_max(count_min, count_max, f"fixture {kind} count")


def _max_room_wall_span(config: Config) -> float:
    if config.layout_mode == "corridor":
        assert config.room_width_max is not None
        assert config.room_depth_max is not None
        return (
            max(config.room_width_max, config.room_depth_max)
            - 2.0 * config.wall_thickness
        )
    return config.max_cell_size - 2.0 * config.wall_thickness


def _validate_fixture_count_feasibility(config: Config) -> None:
    from random_gazebo_world.fixtures import FIXTURE_SPECS

    available = _max_room_wall_span(config)
    if config.layout_mode == "corridor":
        size_keys = "room_width_max or room_depth_max"
    else:
        size_keys = "max_cell_size"

    for kind in ("toilet", "urinal", "basin"):
        count_min = getattr(config, f"fixture_{kind}_count_min")
        pitch = FIXTURE_SPECS[kind].pitch
        required_span = count_min * pitch
        if required_span > available + EPS:
            raise ConfigError(
                f"fixture {kind} cluster requires at least {required_span} m of wall "
                f"(fixture_{kind}_count_min={count_min} * pitch={pitch}), but the "
                f"largest possible room wall is at most {available} m "
                f"({size_keys} minus 2 * wall_thickness). Lower "
                f"fixture_{kind}_count_min or increase {size_keys}."
            )


def _validate_fixture_visual_offsets(config: Config) -> None:
    for name in (
        "fixture_toilet_offset_x",
        "fixture_toilet_offset_y",
        "fixture_toilet_offset_z",
        "fixture_toilet_offset_yaw",
        "fixture_urinal_offset_x",
        "fixture_urinal_offset_y",
        "fixture_urinal_offset_z",
        "fixture_urinal_offset_yaw",
        "fixture_basin_offset_x",
        "fixture_basin_offset_y",
        "fixture_basin_offset_z",
        "fixture_basin_offset_yaw",
    ):
        _require_finite_number(getattr(config, name), name)


_TOILET_CUBICLE_PITCH = 1.5


def _validate_cubicle_settings(config: Config) -> None:
    _require_finite_number(config.cubicle_door_width, "cubicle_door_width")
    _require_positive(config.cubicle_door_width, "cubicle_door_width")
    min_front_wall = config.wall_thickness
    if config.cubicle_door_width + min_front_wall > _TOILET_CUBICLE_PITCH + EPS:
        raise ConfigError(
            "cubicle_door_width + wall_thickness must be <= toilet cubicle pitch "
            f"({_TOILET_CUBICLE_PITCH}), got "
            f"{config.cubicle_door_width + min_front_wall}"
        )
    if config.cubicle_wall_height is None:
        pass
    else:
        _require_finite_number(config.cubicle_wall_height, "cubicle_wall_height")
        _require_positive(config.cubicle_wall_height, "cubicle_wall_height")
        if config.cubicle_wall_height > config.wall_height + EPS:
            raise ConfigError(
                "cubicle_wall_height must be <= wall_height "
                f"({config.wall_height}), got {config.cubicle_wall_height}"
            )


def _load_rgb(
    raw: dict[str, Any],
    name: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    value = raw.get(name, default)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ConfigError(f"{name} must be an RGB list of three values, got {value!r}")
    return tuple(float(channel) for channel in value)


def _validate_rgb_channels(value: tuple[float, float, float], name: str) -> None:
    for index, channel in enumerate(value):
        channel_name = f"{name}[{index}]"
        _require_finite_number(channel, channel_name)
        if not 0.0 <= channel <= 1.0:
            raise ConfigError(f"{channel_name} must be in [0, 1], got {channel}")


def _validate_visual_physics_settings(config: Config) -> None:
    _validate_rgb_channels(config.cubicle_wall_color, "cubicle_wall_color")
    _validate_rgb_channels(config.scene_ambient, "scene_ambient")
    _validate_rgb_channels(config.scene_background, "scene_background")
    _validate_rgb_channels(config.counter_specular, "counter_specular")

    if config.lighting_mode not in ("directional", "point"):
        raise ConfigError(
            "lighting_mode must be 'directional' or 'point', got "
            f"{config.lighting_mode!r}"
        )
    _require_finite_number(config.light_height, "light_height")
    _require_positive(config.light_height, "light_height")
    _require_finite_number(config.corridor_light_spacing, "corridor_light_spacing")
    _require_positive(config.corridor_light_spacing, "corridor_light_spacing")

    if config.physics_profile not in ("ignored", "ode"):
        raise ConfigError(
            "physics_profile must be 'ignored' or 'ode', got "
            f"{config.physics_profile!r}"
        )

    _require_finite_number(config.fixture_friction_mu, "fixture_friction_mu")
    _require_positive(config.fixture_friction_mu, "fixture_friction_mu")
