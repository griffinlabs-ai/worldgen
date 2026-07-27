from __future__ import annotations

import math
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class FeasibilityIssue:
    severity: str
    code: str
    message: str


EPS = 1e-9


class FeasibilityError(ValueError):
    """Raised when a config is provably infeasible to generate."""


def check_feasibility(config) -> list[FeasibilityIssue]:
    issues: list[FeasibilityIssue] = []

    if config.layout_mode == "two_room_gate":
        issues.extend(_check_two_room_gate_feasibility(config))
        return issues

    if config.layout_mode == "two_room_corner":
        issues.extend(_check_two_room_corner_feasibility(config))
        return issues

    w = config.world_width
    h = config.world_height

    if config.partition_method == "bsp":
        max_cells = math.floor((w * h) / (config.min_cell_size**2) + EPS)
        if config.min_room_count > max_cells:
            issues.append(
                FeasibilityIssue(
                    severity="error",
                    code="room_count_exceeds_max_cells",
                    message=(
                        f"min_room_count ({config.min_room_count}) exceeds the maximum "
                        f"number of cells ({max_cells}) the partition can hold: each cell "
                        f"has area >= min_cell_size^2 ({config.min_cell_size**2:.4g}), "
                        f"so at most floor(world_area / min_cell_size^2) cells fit."
                    ),
                )
            )

        forced_cells = math.ceil(w / config.max_cell_size) * math.ceil(
            h / config.max_cell_size
        )
        if config.min_room_count > forced_cells:
            issues.append(
                FeasibilityIssue(
                    severity="warning",
                    code="partition_may_underflow_room_count",
                    message=(
                        f"min_room_count ({config.min_room_count}) exceeds the ~"
                        f"{forced_cells} cells the partition is forced to create "
                        f"(ceil(W/max_cell_size)*ceil(H/max_cell_size)); seeds that "
                        f"produce fewer cells will be rejected and retried, slowing "
                        f"generation."
                    ),
                )
            )

        gate_too_wide = config.gate_width_min > config.max_cell_size + EPS
        passage_too_wide = config.passage_width_min > config.max_cell_size + EPS
        if gate_too_wide or passage_too_wide:
            offenders: list[str] = []
            if gate_too_wide:
                offenders.append(
                    f"gate_width_min ({config.gate_width_min}) > max_cell_size "
                    f"({config.max_cell_size})"
                )
            if passage_too_wide:
                offenders.append(
                    f"passage_width_min ({config.passage_width_min}) > max_cell_size "
                    f"({config.max_cell_size})"
                )
            issues.append(
                FeasibilityIssue(
                    severity="error",
                    code="opening_cannot_fit_wall",
                    message=(
                        "A shared wall is at most max_cell_size long, so an opening "
                        f"of this minimum width can never be placed: {'; '.join(offenders)}."
                    ),
                )
            )

        gate_frequent = config.gate_width_min > config.min_cell_size + EPS
        passage_frequent = config.passage_width_min > config.min_cell_size + EPS
        if gate_frequent or passage_frequent:
            offenders = []
            if gate_frequent:
                offenders.append(
                    f"gate_width_min ({config.gate_width_min}) > min_cell_size "
                    f"({config.min_cell_size})"
                )
            if passage_frequent:
                offenders.append(
                    f"passage_width_min ({config.passage_width_min}) > min_cell_size "
                    f"({config.min_cell_size})"
                )
            issues.append(
                FeasibilityIssue(
                    severity="warning",
                    code="openings_frequently_too_wide",
                    message=(
                        "Many walls are near min_cell_size (and BSP T-junctions make "
                        "many shorter), so a large fraction of openings will be "
                        f"rejected, slowing generation: {'; '.join(offenders)}."
                    ),
                )
            )

        threshold = config.min_cell_size - 2 * config.wall_thickness
        if config.passage_width_max > threshold - EPS:
            issues.append(
                FeasibilityIssue(
                    severity="warning",
                    code="corridor_too_wide_for_cell",
                    message=(
                        f"Corridors of passage_width_max ({config.passage_width_max}) "
                        f"may not fit inside the smallest cells (min_cell_size="
                        f"{config.min_cell_size}) once walls (wall_thickness="
                        f"{config.wall_thickness}) are accounted for; usable width "
                        f"threshold is {threshold:.4g}, causing passage-geometry "
                        f"failures."
                    ),
                )
            )

    sub_pixel_offenders: list[str] = []
    if config.wall_thickness < config.map_resolution:
        sub_pixel_offenders.append(
            f"wall_thickness ({config.wall_thickness}) < map_resolution "
            f"({config.map_resolution})"
        )
    if config.gate_width_min < 2 * config.map_resolution:
        sub_pixel_offenders.append(
            f"gate_width_min ({config.gate_width_min}) < 2*map_resolution "
            f"({2 * config.map_resolution})"
        )
    if config.passage_width_min < 2 * config.map_resolution:
        sub_pixel_offenders.append(
            f"passage_width_min ({config.passage_width_min}) < 2*map_resolution "
            f"({2 * config.map_resolution})"
        )
    if sub_pixel_offenders:
        issues.append(
            FeasibilityIssue(
                severity="warning",
                code="sub_pixel_features",
                message=(
                    "These features are smaller than ~1-2 occupancy-map pixels "
                    f"(map_resolution={config.map_resolution}) and may vanish or "
                    f"break connectivity on the Nav2 map: {'; '.join(sub_pixel_offenders)}."
                ),
            )
        )

    if config.partition_method == "voronoi":
        if config.voronoi_seed_count < config.min_room_count:
            issues.append(
                FeasibilityIssue(
                    severity="error",
                    code="voronoi_seeds_below_room_count",
                    message=(
                        f"Voronoi produces at most voronoi_seed_count "
                        f"({config.voronoi_seed_count}) cells (fewer after area "
                        f"filtering), which is below min_room_count "
                        f"({config.min_room_count}), so room selection can never "
                        f"succeed."
                    ),
                )
            )

        avg_area = (w * h) / config.voronoi_seed_count
        if (
            avg_area < config.voronoi_min_cell_area
            or avg_area > config.voronoi_max_cell_area
        ):
            issues.append(
                FeasibilityIssue(
                    severity="warning",
                    code="voronoi_avg_area_outside_filter",
                    message=(
                        f"Expected average cell area ({avg_area:.4g} = world_area / "
                        f"voronoi_seed_count) falls outside "
                        f"[voronoi_min_cell_area={config.voronoi_min_cell_area}, "
                        f"voronoi_max_cell_area={config.voronoi_max_cell_area}], so "
                        f"most generated cells will be filtered out, causing heavy "
                        f"retries."
                    ),
                )
            )

    return issues


def _check_two_room_common_feasibility(config) -> list[FeasibilityIssue]:
    issues: list[FeasibilityIssue] = []
    sub_pixel_offenders: list[str] = []
    if config.wall_thickness < config.map_resolution:
        sub_pixel_offenders.append(
            f"wall_thickness ({config.wall_thickness}) < map_resolution "
            f"({config.map_resolution})"
        )
    if sub_pixel_offenders:
        issues.append(
            FeasibilityIssue(
                severity="warning",
                code="sub_pixel_features",
                message=(
                    "These features are smaller than ~1 occupancy-map pixel "
                    f"(map_resolution={config.map_resolution}) and may vanish or "
                    f"break connectivity on the Nav2 map: "
                    f"{'; '.join(sub_pixel_offenders)}."
                ),
            )
        )
    return issues


def _check_two_room_gate_feasibility(config) -> list[FeasibilityIssue]:
    issues = _check_two_room_common_feasibility(config)
    assert config.gate_width is not None
    assert config.room_size is not None
    assert config.divider_thickness is not None

    if config.gate_width < 2 * config.map_resolution:
        issues.append(
            FeasibilityIssue(
                severity="warning",
                code="sub_pixel_features",
                message=(
                    f"gate_width ({config.gate_width}) < 2*map_resolution "
                    f"({2 * config.map_resolution}) and may not appear on the map."
                ),
            )
        )

    min_room_span = config.gate_width + 2.0 * config.wall_thickness
    if config.room_size + EPS < min_room_span:
        issues.append(
            FeasibilityIssue(
                severity="error",
                code="gate_cannot_fit_room_wall",
                message=(
                    f"room_size ({config.room_size}) must be >= gate_width + "
                    f"2 * wall_thickness ({min_room_span})."
                ),
            )
        )

    if config.divider_thickness < config.wall_thickness:
        issues.append(
            FeasibilityIssue(
                severity="warning",
                code="divider_thinner_than_wall",
                message=(
                    f"divider_thickness ({config.divider_thickness}) is smaller than "
                    f"wall_thickness ({config.wall_thickness}); the passage cell may "
                    f"be narrower than the wall slabs."
                ),
            )
        )

    return issues


def _check_two_room_corner_feasibility(config) -> list[FeasibilityIssue]:
    issues = _check_two_room_common_feasibility(config)
    assert config.room_size is not None
    assert config.leg_a_width is not None
    assert config.leg_b_width is not None

    for name, width in (
        ("leg_a_width", config.leg_a_width),
        ("leg_b_width", config.leg_b_width),
    ):
        if width < 2 * config.map_resolution:
            issues.append(
                FeasibilityIssue(
                    severity="warning",
                    code="sub_pixel_features",
                    message=(
                        f"{name} ({width}) < 2*map_resolution "
                        f"({2 * config.map_resolution}) and may not appear on the map."
                    ),
                )
            )

    min_room_a_span = config.leg_a_width + 2.0 * config.wall_thickness
    min_room_b_span = config.leg_b_width + 2.0 * config.wall_thickness
    if config.room_size + EPS < min_room_a_span:
        issues.append(
            FeasibilityIssue(
                severity="error",
                code="leg_a_cannot_fit_room_wall",
                message=(
                    f"room_size ({config.room_size}) must be >= leg_a_width + "
                    f"2 * wall_thickness ({min_room_a_span})."
                ),
            )
        )
    if config.room_size + EPS < min_room_b_span:
        issues.append(
            FeasibilityIssue(
                severity="error",
                code="leg_b_cannot_fit_room_wall",
                message=(
                    f"room_size ({config.room_size}) must be >= leg_b_width + "
                    f"2 * wall_thickness ({min_room_b_span})."
                ),
            )
        )

    return issues


def enforce_feasibility(config, *, strict: bool = False, stream=sys.stderr) -> None:
    issues = check_feasibility(config)
    warnings = [i for i in issues if i.severity == "warning"]
    errors = [i for i in issues if i.severity == "error"]
    for i in warnings:
        print(f"[feasibility][warning][{i.code}] {i.message}", file=stream)
    if strict and warnings and not errors:
        raise FeasibilityError(
            "Config has feasibility warnings and --strict-config is set:\n"
            + "\n".join(f"  [{i.code}] {i.message}" for i in warnings)
        )
    if errors:
        raise FeasibilityError(
            "Config is infeasible:\n"
            + "\n".join(f"  [{i.code}] {i.message}" for i in errors)
        )
