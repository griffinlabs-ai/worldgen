from __future__ import annotations

from pathlib import Path

import pytest
from PIL import Image

from random_gazebo_world.animate import (
    ANIMATION_FILENAME,
    NEVER_ANIMATED_STAGES,
    PARTITION_ONLY_STAGES,
    animation_stages,
)
from random_gazebo_world.config import Config, ConfigError
from random_gazebo_world.pipeline import (
    REQUIRED_DEBUG_STAGES,
    generate_valid_world,
    write_world_outputs,
)


def _gate_config(**overrides: object) -> Config:
    values: dict[str, object] = {
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


def test_animation_skips_stages_the_mode_never_ran(tmp_path: Path) -> None:
    """A two-room world drops the selection stages, the raster, and empty fixtures."""
    config = _gate_config(debug_animation=True)
    world = generate_valid_world(config)
    out_dir = tmp_path / "gate"
    write_world_outputs(world, out_dir)

    stages = animation_stages(
        REQUIRED_DEBUG_STAGES, config, world.wall_layout, world.fixture_layout
    )

    assert not PARTITION_ONLY_STAGES & set(stages)
    assert not NEVER_ANIMATED_STAGES & set(stages)
    assert "12_fixtures" not in stages, "fixture_mode is none — the frame is empty"
    assert "10_final_floorplan" in stages
    assert list(stages) == [s for s in REQUIRED_DEBUG_STAGES if s in stages], (
        "animation must preserve render order"
    )

    gif = out_dir / "debug" / ANIMATION_FILENAME
    assert gif.is_file()
    with Image.open(gif) as handle:
        assert handle.n_frames == len(stages)
        assert handle.info["duration"] == 1000  # 1 Hz default

    # Every stage PNG still lands on disk: the animation selects, it does not skip
    # rendering, so the debug contract is untouched.
    for stage in REQUIRED_DEBUG_STAGES:
        assert (out_dir / "debug" / f"{stage}.png").is_file()


def test_partition_mode_keeps_the_selection_stages() -> None:
    """The three topology stages are meaningful only where selection actually runs."""
    config = _gate_config(layout_mode="two_room_gate")
    partition_like = _gate_config(
        layout_mode="partition",
        world_width=12.0,
        world_height=8.0,
        min_cell_size=3.0,
        max_cell_size=6.0,
        min_room_count=2,
        max_room_count=2,
        room_size=None,
        gate_width=None,
        divider_thickness=None,
    )
    world = generate_valid_world(partition_like)

    stages = animation_stages(
        REQUIRED_DEBUG_STAGES,
        partition_like,
        world.wall_layout,
        world.fixture_layout,
    )
    assert PARTITION_ONLY_STAGES <= set(stages)
    assert config.layout_mode != "partition"


def test_animation_is_off_by_default(tmp_path: Path) -> None:
    """One world per trial × a campaign is why this cannot default to on."""
    world = generate_valid_world(_gate_config())
    out_dir = tmp_path / "no_gif"
    write_world_outputs(world, out_dir)
    assert not (out_dir / "debug" / ANIMATION_FILENAME).exists()


@pytest.mark.parametrize(
    "overrides",
    [
        {"debug_animation_fps": 0.0},
        {"debug_animation_fps": -1.0},
        {"debug_animation_max_px": 0},
        {"debug_animation_max_px": 1.5},
    ],
)
def test_invalid_animation_settings_are_rejected(overrides: dict[str, object]) -> None:
    with pytest.raises(ConfigError):
        _gate_config(**overrides)
