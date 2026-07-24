from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.corridor import (
    CORRIDOR_CELL_ID,
    CorridorError,
    generate_corridor_layout,
)
from random_gazebo_world.geometry import EPS, get_shared_wall
from random_gazebo_world.pipeline import (
    REQUIRED_DEBUG_STAGES,
    REQUIRED_OUTPUTS,
    generate_valid_world,
    write_world_outputs,
)
from random_gazebo_world.rng import create_seeded_rng


def _corridor_config(**overrides: float | int | str) -> Config:
    values = {
        "world_width": 1.0,
        "world_height": 1.0,
        "min_cell_size": 1.0,
        "max_cell_size": 10.0,
        "min_room_count": 1,
        "max_room_count": 100,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.9,
        "gate_width_max": 0.9,
        "passage_width_min": 1.0,
        "passage_width_max": 1.0,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
        "layout_mode": "corridor",
        "corridor_width": 2.5,
        "corridor_length": 20.0,
        "entrance_width": 0.9,
        "room_width_min": 2.5,
        "room_width_max": 5.0,
        "room_depth_min": 2.0,
        "room_depth_max": 4.0,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def test_corridor_side_widths_tile_exactly() -> None:
    config = _corridor_config(corridor_length=18.0)
    layout = generate_corridor_layout(config, create_seeded_rng(7))
    corridor = layout.partition.cells[0]
    assert corridor.width == pytest.approx(config.corridor_length)

    for cell in layout.partition.cells[1:]:
        if cell.id == CORRIDOR_CELL_ID:
            continue
        assert cell.width > 0.0

    bottom_widths = [
        cell.width
        for cell in layout.partition.cells[1:]
        if cell.y_max == corridor.y_min
    ]
    top_widths = [
        cell.width
        for cell in layout.partition.cells[1:]
        if cell.y_min == corridor.y_max
    ]
    assert sum(bottom_widths) == pytest.approx(config.corridor_length, abs=1e-6)
    assert sum(top_widths) == pytest.approx(config.corridor_length, abs=1e-6)


def test_each_room_has_one_entrance_with_margins() -> None:
    config = _corridor_config()
    layout = generate_corridor_layout(config, create_seeded_rng(99))
    corridor = layout.partition.cells[0]
    cells_by_id = {cell.id: cell for cell in layout.partition.cells}
    margin = config.wall_thickness

    room_openings = [
        opening
        for opening in layout.opening_layout.openings
        if CORRIDOR_CELL_ID in (opening.cell_a_id, opening.cell_b_id)
    ]
    assert len(room_openings) == len(layout.room_selection.room_cell_ids)

    for opening in room_openings:
        assert opening.kind == "gate"
        assert opening.width == pytest.approx(config.entrance_width)
        assert opening.span_start >= margin - EPS
        assert opening.shared_wall.length - opening.span_end >= margin - EPS

        room_id = (
            opening.cell_b_id
            if opening.cell_a_id == CORRIDOR_CELL_ID
            else opening.cell_a_id
        )
        room_cell = cells_by_id[room_id]
        shared_wall = get_shared_wall(room_cell, corridor)
        assert shared_wall == opening.shared_wall


def test_corridor_layout_is_deterministic_for_seed() -> None:
    config = _corridor_config(random_seed=123)
    first = generate_corridor_layout(config, create_seeded_rng(123))
    second = generate_corridor_layout(config, create_seeded_rng(123))
    assert first.partition == second.partition
    assert first.opening_layout.openings == second.opening_layout.openings


def test_corridor_config_validation(tmp_path: Path) -> None:
    config_path = tmp_path / "corridor.yaml"
    config_path.write_text(
        "\n".join(
            [
                "layout_mode: corridor",
                "corridor_length: 10.0",
                "corridor_width: 2.0",
                "entrance_width: 0.9",
                "room_width_min: 2.5",
                "room_width_max: 4.0",
                "room_depth_min: 2.0",
                "room_depth_max: 3.0",
                "wall_height: 2.5",
                "wall_thickness: 0.15",
                "map_resolution: 0.05",
                "random_seed: 1",
            ]
        ),
        encoding="utf-8",
    )
    load_config(config_path)

    bad = tmp_path / "bad_corridor.yaml"
    bad.write_text(
        config_path.read_text(encoding="utf-8").replace(
            "room_width_min: 2.5", "room_width_min: 1.0"
        ),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="room_width_min"):
        load_config(bad)

    missing = tmp_path / "missing_corridor.yaml"
    missing.write_text(
        "\n".join(
            [
                "layout_mode: corridor",
                "wall_height: 2.5",
                "wall_thickness: 0.15",
                "map_resolution: 0.05",
                "random_seed: 1",
            ]
        ),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="is required when layout_mode is 'corridor'"):
        load_config(missing)


def test_corridor_end_to_end_outputs(tmp_path: Path) -> None:
    config = load_config(Path("configs/corridor.yaml"))
    world = generate_valid_world(config, max_attempts=100)
    out_dir = tmp_path / "corridor_world"
    write_world_outputs(world, out_dir)

    for relative_path in REQUIRED_OUTPUTS:
        assert (out_dir / relative_path).is_file()

    debug_dir = out_dir / "debug"
    for stage in REQUIRED_DEBUG_STAGES:
        assert (debug_dir / f"{stage}.png").is_file()
        if stage != "09_occupancy_map_preview":
            assert (debug_dir / f"{stage}.svg").is_file()


def test_load_corridor_example_config() -> None:
    config = load_config(Path("configs/corridor.yaml"))
    assert config.layout_mode == "corridor"
    assert config.corridor_length == 24.0


def test_rescaled_room_too_narrow_raises() -> None:
    config = _corridor_config(
        corridor_length=1.0,
        room_width_min=2.5,
        room_width_max=5.0,
        entrance_width=0.9,
    )
    with pytest.raises(CorridorError):
        generate_corridor_layout(config, create_seeded_rng(0))
