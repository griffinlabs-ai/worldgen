from __future__ import annotations

import io

import pytest

from random_gazebo_world.config import Config
from random_gazebo_world.feasibility import (
    FeasibilityError,
    check_feasibility,
    enforce_feasibility,
)


def _baseline_config(**overrides) -> Config:
    config = Config(
        world_width=20.0,
        world_height=20.0,
        min_cell_size=2.0,
        max_cell_size=6.0,
        min_room_count=7,
        max_room_count=15,
        wall_height=2.5,
        wall_thickness=0.15,
        gate_width_min=0.5,
        gate_width_max=0.8,
        passage_width_min=0.5,
        passage_width_max=0.8,
        extra_loop_probability=0.2,
        map_resolution=0.05,
        random_seed=42,
        partition_method="bsp",
        passage_geometry_mode="curved",
    )
    if overrides:
        from dataclasses import replace

        config = replace(config, **overrides)
    config.validate()
    return config


def _issue_codes(issues) -> set[tuple[str, str]]:
    return {(i.severity, i.code) for i in issues}


def test_baseline_has_no_errors() -> None:
    issues = check_feasibility(_baseline_config())
    errors = [i for i in issues if i.severity == "error"]
    assert errors == []


def test_room_count_exceeds_max_cells() -> None:
    config = _baseline_config(min_room_count=101, max_room_count=101)
    codes = _issue_codes(check_feasibility(config))
    assert ("error", "room_count_exceeds_max_cells") in codes


def test_partition_may_underflow_room_count() -> None:
    config = _baseline_config(min_room_count=17, max_room_count=17)
    codes = _issue_codes(check_feasibility(config))
    assert ("warning", "partition_may_underflow_room_count") in codes


def test_opening_cannot_fit_wall() -> None:
    config = _baseline_config(gate_width_min=7.0, gate_width_max=7.5)
    codes = _issue_codes(check_feasibility(config))
    assert ("error", "opening_cannot_fit_wall") in codes


def test_openings_frequently_too_wide() -> None:
    config = _baseline_config(gate_width_min=2.5, gate_width_max=3.0)
    codes = _issue_codes(check_feasibility(config))
    assert ("warning", "openings_frequently_too_wide") in codes


def test_corridor_too_wide_for_cell() -> None:
    config = _baseline_config(passage_width_min=0.5, passage_width_max=1.8)
    codes = _issue_codes(check_feasibility(config))
    assert ("warning", "corridor_too_wide_for_cell") in codes


def test_sub_pixel_features() -> None:
    config = _baseline_config(wall_thickness=0.04)
    codes = _issue_codes(check_feasibility(config))
    assert ("warning", "sub_pixel_features") in codes


def test_voronoi_seeds_below_room_count() -> None:
    config = _baseline_config(
        partition_method="voronoi",
        voronoi_seed_count=5,
        min_room_count=7,
    )
    codes = _issue_codes(check_feasibility(config))
    assert ("error", "voronoi_seeds_below_room_count") in codes


def test_voronoi_avg_area_outside_filter() -> None:
    config = _baseline_config(
        partition_method="voronoi",
        voronoi_seed_count=500,
        min_room_count=7,
    )
    codes = _issue_codes(check_feasibility(config))
    assert ("warning", "voronoi_avg_area_outside_filter") in codes


def test_enforce_feasibility_raises_on_error() -> None:
    config = _baseline_config(min_room_count=101, max_room_count=101)
    with pytest.raises(FeasibilityError, match="room_count_exceeds_max_cells"):
        enforce_feasibility(config, stream=io.StringIO())


def test_enforce_feasibility_strict_raises_on_warning_only() -> None:
    config = _baseline_config(min_room_count=17, max_room_count=17)
    enforce_feasibility(config, strict=False, stream=io.StringIO())
    with pytest.raises(FeasibilityError, match="strict-config"):
        enforce_feasibility(config, strict=True, stream=io.StringIO())
