from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from shapely.geometry import Polygon
from shapely.ops import unary_union

from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.export_map import build_nav_task, cell_to_world
from random_gazebo_world.pipeline import (
    REQUIRED_DEBUG_STAGES,
    generate_valid_world,
    required_outputs,
    write_world_outputs,
)
from random_gazebo_world.two_room import (
    PASSAGE_CELL_ID,
    ROOM_A_CELL_ID,
    ROOM_B_CELL_ID,
    generate_two_room_corner_layout,
    generate_two_room_gate_layout,
)


def _gate_config(**overrides: float | int | str) -> Config:
    values = {
        "world_width": 1.0,
        "world_height": 1.0,
        "min_cell_size": 1.0,
        "max_cell_size": 10.0,
        "min_room_count": 1,
        "max_room_count": 100,
        "wall_height": 2.5,
        "wall_thickness": 0.05,
        "gate_width_min": 0.78,
        "gate_width_max": 0.78,
        "passage_width_min": 0.78,
        "passage_width_max": 0.78,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
        "layout_mode": "two_room_gate",
        "room_size": 4.0,
        "gate_width": 0.78,
        "divider_thickness": 0.5,
        "passage_geometry_mode": "legacy_orthogonal",
        "fixture_mode": "none",
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _corner_config(**overrides: float | int | str) -> Config:
    values = {
        "world_width": 1.0,
        "world_height": 1.0,
        "min_cell_size": 1.0,
        "max_cell_size": 10.0,
        "min_room_count": 1,
        "max_room_count": 100,
        "wall_height": 2.5,
        "wall_thickness": 0.05,
        "gate_width_min": 0.78,
        "gate_width_max": 0.78,
        "passage_width_min": 0.78,
        "passage_width_max": 0.78,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
        "layout_mode": "two_room_corner",
        "room_size": 4.0,
        "leg_a_width": 0.78,
        "leg_a_length": 3.0,
        "leg_b_width": 0.78,
        "leg_b_length": 3.0,
        "fixture_mode": "none",
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def test_gate_layout_is_deterministic_across_seeds() -> None:
    layout_a = generate_two_room_gate_layout(_gate_config(random_seed=1))
    layout_b = generate_two_room_gate_layout(_gate_config(random_seed=999))
    assert layout_a.partition.world_width == layout_b.partition.world_width
    assert layout_a.partition.world_height == layout_b.partition.world_height
    assert len(layout_a.opening_layout.openings) == len(
        layout_b.opening_layout.openings
    )
    for opening_a, opening_b in zip(
        layout_a.opening_layout.openings,
        layout_b.opening_layout.openings,
        strict=True,
    ):
        assert opening_a.width == pytest.approx(opening_b.width)
        assert opening_a.span_start == pytest.approx(opening_b.span_start)
        assert opening_a.span_end == pytest.approx(opening_b.span_end)


def test_gate_layout_has_two_rooms_one_divider_and_centered_openings() -> None:
    config = _gate_config()
    layout = generate_two_room_gate_layout(config)

    assert layout.room_selection.room_count == 2
    assert layout.applied_layout.passage_cell_ids == frozenset({PASSAGE_CELL_ID})
    assert len(layout.opening_layout.openings) == 2
    assert all(
        opening.width == pytest.approx(config.gate_width)
        for opening in layout.opening_layout.openings
    )

    divider = next(
        cell for cell in layout.partition.cells if cell.id == PASSAGE_CELL_ID
    )
    assert divider.width == pytest.approx(config.divider_thickness)
    assert divider.height == pytest.approx(config.room_size)

    for opening in layout.opening_layout.openings:
        assert opening.center == pytest.approx(config.room_size / 2.0)

    assert layout.room_centers[0].x == pytest.approx(config.room_size / 2.0)
    assert layout.room_centers[1].x == pytest.approx(
        config.room_size + config.divider_thickness + config.room_size / 2.0
    )


def test_corner_layout_builds_single_l_corridor() -> None:
    config = _corner_config()
    layout = generate_two_room_corner_layout(config)
    assert layout.passage_geometry is not None

    corridor = layout.passage_geometry.corridor_for(PASSAGE_CELL_ID)
    assert corridor is not None
    assert corridor.geom_type == "Polygon"

    inset = config.wall_thickness / 2.0
    horizontal = Polygon(
        [
            (config.room_size, inset),
            (config.room_size + config.leg_a_length, inset),
            (config.room_size + config.leg_a_length, inset + config.leg_a_width),
            (config.room_size, inset + config.leg_a_width),
        ]
    )
    vertical = Polygon(
        [
            (
                config.room_size + config.leg_a_length - config.leg_b_width,
                inset + config.leg_a_width,
            ),
            (config.room_size + config.leg_a_length, inset + config.leg_a_width),
            (
                config.room_size + config.leg_a_length,
                inset + config.leg_a_width + config.leg_b_length,
            ),
            (
                config.room_size + config.leg_a_length - config.leg_b_width,
                inset + config.leg_a_width + config.leg_b_length,
            ),
        ]
    )
    expected = unary_union([horizontal, vertical])
    assert corridor.equals(expected)


def test_corner_openings_use_leg_widths() -> None:
    layout = generate_two_room_corner_layout(_corner_config())
    widths = sorted(opening.width for opening in layout.opening_layout.openings)
    assert widths == pytest.approx([0.78, 0.78])


def test_gate_end_to_end_outputs_and_pins_nav_task(tmp_path: Path) -> None:
    config = load_config(Path("configs/two_room_gate.yaml"))
    world = generate_valid_world(config)
    out_dir = tmp_path / "gate_world"
    write_world_outputs(world, out_dir)

    for relative_path in required_outputs(out_dir):
        assert (out_dir / relative_path).is_file()

    metadata = json.loads((out_dir / "metadata.json").read_text(encoding="utf-8"))
    assert len(metadata["room_centers"]) == 2
    assert metadata["room_centers"][0]["cell_id"] == ROOM_A_CELL_ID
    assert metadata["room_centers"][1]["cell_id"] == ROOM_B_CELL_ID

    nav_task = json.loads((out_dir / "nav_task.json").read_text(encoding="utf-8"))
    resolution = config.map_resolution
    assert nav_task["start"]["x"] == pytest.approx(
        metadata["room_centers"][0]["x"], abs=resolution
    )
    assert nav_task["start"]["y"] == pytest.approx(
        metadata["room_centers"][0]["y"], abs=resolution
    )
    assert nav_task["goal"]["x"] == pytest.approx(
        metadata["room_centers"][1]["x"], abs=resolution
    )
    assert nav_task["goal"]["y"] == pytest.approx(
        metadata["room_centers"][1]["y"], abs=resolution
    )


def test_corner_end_to_end_outputs_and_pins_nav_task(tmp_path: Path) -> None:
    config = load_config(Path("configs/two_room_corner.yaml"))
    world = generate_valid_world(config)
    out_dir = tmp_path / "corner_world"
    write_world_outputs(world, out_dir)

    for relative_path in required_outputs(out_dir):
        assert (out_dir / relative_path).is_file()

    debug_dir = out_dir / "debug"
    for stage in REQUIRED_DEBUG_STAGES:
        assert (debug_dir / f"{stage}.png").is_file()

    metadata = json.loads((out_dir / "metadata.json").read_text(encoding="utf-8"))
    assert "room_centers" in metadata
    nav_task = build_nav_task(world.occupancy)
    start_x, start_y = cell_to_world(world.occupancy.start_cell, world.occupancy)
    assert nav_task["start"]["x"] == pytest.approx(start_x)
    assert nav_task["start"]["y"] == pytest.approx(start_y)


def test_load_example_configs() -> None:
    gate = load_config(Path("configs/two_room_gate.yaml"))
    corner = load_config(Path("configs/two_room_corner.yaml"))
    assert gate.layout_mode == "two_room_gate"
    assert corner.layout_mode == "two_room_corner"


def test_gate_room_size_too_small_raises() -> None:
    with pytest.raises(ConfigError, match="room_size"):
        _gate_config(room_size=0.5, gate_width=0.78)


def test_corner_room_size_too_small_raises() -> None:
    with pytest.raises(ConfigError, match="room_size"):
        _corner_config(room_size=0.5, leg_a_width=0.78)


def test_fixture_mode_not_allowed() -> None:
    with pytest.raises(ConfigError, match="fixture_mode"):
        _gate_config(fixture_mode="restroom_clusters")


def test_zero_jitter_pins_start_goal_at_room_centers() -> None:
    config = _gate_config(
        start_jitter_x=0.0,
        start_jitter_y=0.0,
        start_jitter_yaw_deg=0.0,
        goal_jitter_x=0.0,
        goal_jitter_y=0.0,
        goal_jitter_yaw_deg=0.0,
    )
    world = generate_valid_world(config)
    layout = generate_two_room_gate_layout(config)
    nav_task = build_nav_task(world.occupancy)
    resolution = config.map_resolution

    assert nav_task["start"]["x"] == pytest.approx(layout.room_centers[0].x, abs=resolution)
    assert nav_task["start"]["y"] == pytest.approx(layout.room_centers[0].y, abs=resolution)
    assert nav_task["goal"]["x"] == pytest.approx(layout.room_centers[1].x, abs=resolution)
    assert nav_task["goal"]["y"] == pytest.approx(layout.room_centers[1].y, abs=resolution)


def test_same_seed_and_jitter_produces_identical_nav_task() -> None:
    overrides = {
        "start_jitter_x": 0.4,
        "start_jitter_y": 0.4,
        "start_jitter_yaw_deg": 25.0,
        "goal_jitter_x": 0.4,
        "goal_jitter_y": 0.4,
        "goal_jitter_yaw_deg": 10.0,
        "random_seed": 7,
    }
    world_a = generate_valid_world(_gate_config(**overrides))
    world_b = generate_valid_world(_gate_config(**overrides))
    assert build_nav_task(world_a.occupancy) == build_nav_task(world_b.occupancy)


def test_different_seeds_produce_different_jittered_poses_within_bounds() -> None:
    overrides = {
        "start_jitter_x": 0.4,
        "start_jitter_y": 0.4,
        "start_jitter_yaw_deg": 25.0,
        "goal_jitter_x": 0.4,
        "goal_jitter_y": 0.4,
        "goal_jitter_yaw_deg": 25.0,
    }
    world_a = generate_valid_world(_gate_config(random_seed=1, **overrides))
    world_b = generate_valid_world(_gate_config(random_seed=2, **overrides))
    nav_a = build_nav_task(world_a.occupancy)
    nav_b = build_nav_task(world_b.occupancy)
    layout = generate_two_room_gate_layout(_gate_config(**overrides))

    assert (nav_a["start"]["x"], nav_a["start"]["y"]) != (
        nav_b["start"]["x"],
        nav_b["start"]["y"],
    )
    assert nav_a["start"]["yaw"] != pytest.approx(nav_b["start"]["yaw"])

    for nav, center_index in ((nav_a, 0), (nav_b, 0), (nav_a, 1), (nav_b, 1)):
        prefix = "start" if center_index == 0 else "goal"
        center = layout.room_centers[center_index]
        assert abs(nav[prefix]["x"] - center.x) <= 0.4 + 1e-9
        assert abs(nav[prefix]["y"] - center.y) <= 0.4 + 1e-9

    assert world_a.start_goal_jitter is not None
    for name, bound in (
        ("start_jitter_x", 0.4),
        ("start_jitter_y", 0.4),
        ("start_jitter_yaw_deg", 25.0),
        ("goal_jitter_x", 0.4),
        ("goal_jitter_y", 0.4),
        ("goal_jitter_yaw_deg", 25.0),
    ):
        assert abs(world_a.start_goal_jitter[name]) <= bound + 1e-9


def test_yaw_jitter_offsets_heading_without_moving_poses() -> None:
    config = _gate_config(
        start_jitter_x=0.0,
        start_jitter_y=0.0,
        goal_jitter_x=0.0,
        goal_jitter_y=0.0,
        start_jitter_yaw_deg=15.0,
        goal_jitter_yaw_deg=10.0,
        random_seed=3,
    )
    world = generate_valid_world(config)
    layout = generate_two_room_gate_layout(config)
    nav_task = build_nav_task(world.occupancy)
    resolution = config.map_resolution

    assert nav_task["start"]["x"] == pytest.approx(layout.room_centers[0].x, abs=resolution)
    assert nav_task["start"]["y"] == pytest.approx(layout.room_centers[0].y, abs=resolution)
    assert nav_task["goal"]["x"] == pytest.approx(layout.room_centers[1].x, abs=resolution)
    assert nav_task["goal"]["y"] == pytest.approx(layout.room_centers[1].y, abs=resolution)

    heading = math.atan2(
        nav_task["goal"]["y"] - nav_task["start"]["y"],
        nav_task["goal"]["x"] - nav_task["start"]["x"],
    )
    assert world.start_goal_jitter is not None
    start_yaw_offset = math.radians(world.start_goal_jitter["start_jitter_yaw_deg"])
    goal_yaw_offset = math.radians(world.start_goal_jitter["goal_jitter_yaw_deg"])
    assert nav_task["start"]["yaw"] == pytest.approx(heading + start_yaw_offset, abs=1e-9)
    assert nav_task["goal"]["yaw"] == pytest.approx(heading + goal_yaw_offset, abs=1e-9)
    assert nav_task["start"]["yaw"] != pytest.approx(nav_task["goal"]["yaw"])
