from __future__ import annotations

import random
import shutil
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, TypeVar

import networkx as nx

from random_gazebo_world.adjacency import AdjacencyGraph, build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.corridor import CorridorError, generate_corridor_layout
from random_gazebo_world.fixtures import FixtureError, FixtureLayout, generate_fixtures
from random_gazebo_world.export_map import (
    OccupancyMap,
    OccupancyMapError,
    export_nav_task_json,
    generate_occupancy_map,
    write_occupancy_map_files,
)
from random_gazebo_world.export_sdf import export_world_sdf
from random_gazebo_world.metadata import (
    LayoutDocument,
    build_layout_document,
    export_layout_json,
    export_metadata_json,
)
from random_gazebo_world.openings import OpeningError, OpeningLayout, generate_openings
from random_gazebo_world.partition import Partition, PartitionError, generate_partition
from random_gazebo_world.passage_geometry import (
    PassageGeometryError,
    PassageGeometryLayout,
    generate_passage_geometry,
)
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    AppliedLayout,
    AppliedLayoutError,
    CandidateConnectionError,
    CandidateConnections,
    ConnectionType,
    RoomGraphSelectionError,
    RoomSelection,
    RoomSelectionError,
    SelectedRoomGraph,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
    select_rooms,
    validate_passage_constraints,
    validate_selected_room_graph,
)
from random_gazebo_world.walls import WallGenerationError, WallLayout, generate_walls
from random_gazebo_world.visualize import (
    render_adjacency_graph,
    render_candidate_connections,
    render_final_floorplan,
    render_openings,
    render_partition,
    render_passage_cells,
    render_passage_geometry,
    render_fixtures,
    render_selected_room_graph,
    render_selected_rooms,
    render_wall_segments,
)



REQUIRED_OUTPUTS = (
    "world.sdf",
    "map.png",
    "map.yaml",
    "layout.json",
    "metadata.json",
    "nav_task.json",
)

REQUIRED_DEBUG_STAGES = (
    "01_partition",
    "02_selected_rooms",
    "03_cell_adjacency_graph",
    "04_candidate_connections",
    "05_selected_room_graph",
    "06_passage_cells",
    "07_openings",
    "08_wall_segments",
    "09_occupancy_map_preview",
    "10_final_floorplan",
    "11_passage_geometry",
    "12_fixtures",
)


class WorldValidationError(RuntimeError):
    """Raised when a generated world fails end-to-end validation."""


class WorldGenerationError(RuntimeError):
    """Raised when no valid world could be generated within retry limits."""


class _RetryStageError(RuntimeError):
    """Internal wrapper that annotates retryable failures with a stage."""

    def __init__(self, stage: str, context: dict[str, int], cause: Exception) -> None:
        super().__init__(f"{stage}: {cause}")
        self.stage = stage
        self.context = context
        self.cause = cause


RETRYABLE_ERRORS = (
    WorldValidationError,
    RoomGraphSelectionError,
    RoomSelectionError,
    CandidateConnectionError,
    AppliedLayoutError,
    OpeningError,
    PassageGeometryError,
    WallGenerationError,
    OccupancyMapError,
    PartitionError,
    CorridorError,
    FixtureError,
)


@dataclass(frozen=True)
class GeneratedWorld:
    config: Config
    partition: Partition
    adjacency: AdjacencyGraph
    room_selection: RoomSelection
    candidates: CandidateConnections
    selected_graph: SelectedRoomGraph
    applied_layout: AppliedLayout
    opening_layout: OpeningLayout
    passage_geometry: PassageGeometryLayout
    wall_layout: WallLayout
    fixture_layout: FixtureLayout
    occupancy: OccupancyMap
    layout_document: LayoutDocument
    attempt: int


class _RetryDiagnostics:
    def __init__(self, enabled: bool, summary_interval: int) -> None:
        self.enabled = enabled
        self.summary_interval = max(1, summary_interval)
        self.total_rejections = 0
        self.stage_counts: Counter[str] = Counter()
        self.error_counts: Counter[str] = Counter()
        self.last_context: dict[str, int] = {}
        self.last_error = ""

    def record_rejection(
        self,
        stage: str,
        context: dict[str, int],
        error: Exception,
    ) -> None:
        if not self.enabled:
            return
        self.total_rejections += 1
        self.stage_counts[stage] += 1
        error_key = f"{type(error).__name__}: {error}"
        self.error_counts[error_key] += 1
        self.last_context = dict(context)
        self.last_error = error_key
        if self.total_rejections % self.summary_interval == 0:
            self._emit_summary("retry summary")

    def emit_success(self, world: GeneratedWorld) -> None:
        if not self.enabled or self.total_rejections == 0:
            return
        self._emit_summary(
            f"success attempt={world.attempt} seed={world.config.random_seed}"
        )

    def emit_failure(self, structural_attempts: int) -> None:
        if not self.enabled or self.total_rejections == 0:
            return
        self._emit_summary(f"failure attempts={structural_attempts}")

    def _emit_summary(self, prefix: str) -> None:
        stage_summary = ", ".join(
            f"{stage}:{count}" for stage, count in self.stage_counts.most_common(4)
        )
        top_errors = ", ".join(
            f"{error} x{count}" for error, count in self.error_counts.most_common(2)
        )
        context_summary = _format_retry_context(self.last_context)
        print(
            "[debug-retries] "
            f"{prefix}; rejected={self.total_rejections}; "
            f"stages=[{stage_summary}]; "
            f"context=[{context_summary}]; "
            f"last_error={self.last_error}; "
            f"top_errors=[{top_errors}]",
            file=sys.stderr,
        )


def generate_valid_world(
    config: Config,
    max_attempts: int | None = None,
    debug_retries: bool = False,
    debug_retry_summary_interval: int = 25,
) -> GeneratedWorld:
    structural_attempts = (
        max_attempts if max_attempts is not None else config.max_attempts
    )
    selection_attempts = config.max_selection_attempts
    last_error: Exception | None = None
    diagnostics = _RetryDiagnostics(
        enabled=debug_retries,
        summary_interval=debug_retry_summary_interval,
    )

    for attempt in range(structural_attempts):
        attempt_config = config.with_seed(config.random_seed + attempt)
        rng = create_seeded_rng(attempt_config.random_seed)
        if attempt_config.layout_mode == "corridor":
            try:
                world = _attempt_corridor_world(
                    attempt_config,
                    rng,
                    attempt,
                    diagnostics,
                )
                diagnostics.emit_success(world)
                return world
            except _RetryStageError as exc:
                last_error = exc.cause
                diagnostics.record_rejection(
                    stage=exc.stage,
                    context={"attempt": attempt, "seed": attempt_config.random_seed}
                    | exc.context,
                    error=exc.cause,
                )
                continue
            except RETRYABLE_ERRORS as exc:
                last_error = exc
                diagnostics.record_rejection(
                    stage="corridor_layout",
                    context={"attempt": attempt, "seed": attempt_config.random_seed},
                    error=exc,
                )
                continue
            continue

        try:
            structure = _build_structure(attempt_config, rng)
        except _RetryStageError as exc:
            last_error = exc.cause
            diagnostics.record_rejection(
                stage=exc.stage,
                context={"attempt": attempt, "seed": attempt_config.random_seed}
                | exc.context,
                error=exc.cause,
            )
            continue
        except RETRYABLE_ERRORS as exc:
            last_error = exc
            diagnostics.record_rejection(
                stage="build_structure",
                context={"attempt": attempt, "seed": attempt_config.random_seed},
                error=exc,
            )
            continue

        for selection_retry in range(selection_attempts):
            try:
                world = _attempt_selection(
                    structure=structure,
                    config=attempt_config,
                    rng=rng,
                    attempt=attempt,
                    selection_retry=selection_retry,
                    selection_attempts=selection_attempts,
                )
                diagnostics.emit_success(world)
                return world
            except _RetryStageError as exc:
                last_error = exc.cause
                diagnostics.record_rejection(
                    stage=exc.stage,
                    context={"attempt": attempt, "seed": attempt_config.random_seed}
                    | exc.context,
                    error=exc.cause,
                )
                continue
            except RETRYABLE_ERRORS as exc:
                last_error = exc
                diagnostics.record_rejection(
                    stage="selection",
                    context={
                        "attempt": attempt,
                        "seed": attempt_config.random_seed,
                        "selection_retry": selection_retry + 1,
                        "selection_attempts": selection_attempts,
                    },
                    error=exc,
                )
                continue

    diagnostics.emit_failure(structural_attempts)
    raise WorldGenerationError(
        f"Failed to generate a valid world after {structural_attempts} attempts"
    ) from last_error


def write_world_outputs(world: GeneratedWorld, out_dir: Path) -> None:
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = out_dir / "debug"
    debug_dir.mkdir(parents=True, exist_ok=True)

    export_layout_json(out_dir / "layout.json", world.layout_document)
    nav_task = export_nav_task_json(out_dir / "nav_task.json", world.occupancy)
    export_metadata_json(
        out_dir / "metadata.json",
        world.config,
        world.layout_document,
        world.selected_graph,
        nav_task=nav_task,
    )
    render_partition(world.partition, debug_dir / "01_partition")
    render_selected_rooms(
        world.partition,
        world.room_selection,
        debug_dir / "02_selected_rooms",
    )
    render_adjacency_graph(
        world.partition,
        world.adjacency,
        debug_dir / "03_cell_adjacency_graph",
    )
    render_candidate_connections(
        world.partition,
        world.room_selection,
        world.candidates,
        debug_dir / "04_candidate_connections",
    )
    render_selected_room_graph(
        world.partition,
        world.selected_graph,
        debug_dir / "05_selected_room_graph",
    )
    render_passage_cells(world.applied_layout, debug_dir / "06_passage_cells")
    render_openings(world.opening_layout, debug_dir / "07_openings")
    render_wall_segments(world.wall_layout, debug_dir / "08_wall_segments")
    write_occupancy_map_files(world.occupancy, out_dir)
    export_world_sdf(
        world.wall_layout,
        world.config,
        out_dir / "world.sdf",
        fixture_layout=world.fixture_layout,
    )
    render_final_floorplan(world.wall_layout, debug_dir / "10_final_floorplan")
    render_passage_geometry(world.wall_layout, debug_dir / "11_passage_geometry")
    render_fixtures(
        world.wall_layout,
        world.fixture_layout,
        debug_dir / "12_fixtures",
    )

    _validate_output_tree(out_dir)


def validate_world_connectivity(world: GeneratedWorld) -> None:
    if world.config.layout_mode == "corridor":
        return

    room_ids = sorted(world.room_selection.room_cell_ids)
    if len(room_ids) <= 1:
        return

    _validate_candidate_connectivity(world.room_selection, world.candidates)
    validate_selected_room_graph(world.selected_graph, world.config)


def _validate_candidate_connectivity(
    room_selection: RoomSelection,
    candidates: CandidateConnections,
) -> None:
    room_ids = sorted(room_selection.room_cell_ids)
    if len(room_ids) <= 1:
        return

    candidate_graph = nx.Graph()
    candidate_graph.add_nodes_from(room_ids)
    for connection in candidates.connections:
        candidate_graph.add_edge(connection.room_a_id, connection.room_b_id)

    if not nx.is_connected(candidate_graph):
        raise WorldValidationError("Selected rooms cannot be connected via candidates")


@dataclass(frozen=True)
class _Structure:
    partition: Partition
    adjacency: AdjacencyGraph
    room_selection: RoomSelection
    candidates: CandidateConnections


T = TypeVar("T")


def _run_retryable_stage(
    stage: str,
    context: dict[str, int],
    operation: Callable[[], T],
) -> T:
    try:
        return operation()
    except RETRYABLE_ERRORS as exc:
        raise _RetryStageError(stage=stage, context=context, cause=exc) from exc


def _count_candidate_types(candidates: CandidateConnections) -> tuple[int, int]:
    gate_candidates = sum(
        1
        for connection in candidates.connections
        if connection.connection_type == ConnectionType.GATE
    )
    passage_candidates = len(candidates.connections) - gate_candidates
    return gate_candidates, passage_candidates


def _format_retry_context(context: dict[str, int]) -> str:
    selection_summary = ""
    if "selection_retry" in context and "selection_attempts" in context:
        selection_summary = (
            f"; selection={context['selection_retry']}/{context['selection_attempts']}"
        )
    return (
        f"attempt={context.get('attempt', -1)}; "
        f"seed={context.get('seed', -1)}; "
        f"cells={context.get('cells', -1)}; "
        f"selected_rooms={context.get('selected_rooms', -1)}; "
        f"candidates={context.get('candidate_count', -1)}; "
        f"gate_candidates={context.get('gate_candidate_count', -1)}; "
        f"passage_candidates={context.get('passage_candidate_count', -1)}"
        f"{selection_summary}"
    )


def _build_structure(config: Config, rng: random.Random) -> _Structure:
    partition = _run_retryable_stage(
        "build_structure",
        context={},
        operation=lambda: generate_partition(config, rng),
    )
    context: dict[str, int] = {"cells": len(partition.cells)}
    adjacency = build_adjacency_graph(partition)
    room_selection = _run_retryable_stage(
        "build_structure",
        context=context,
        operation=lambda: select_rooms(partition, config, rng),
    )
    context = context | {"selected_rooms": room_selection.room_count}
    candidates = _run_retryable_stage(
        "build_structure",
        context=context,
        operation=lambda: generate_candidate_connections(room_selection, adjacency, config),
    )
    gate_candidates, passage_candidates = _count_candidate_types(candidates)
    context = context | {
        "candidate_count": len(candidates.connections),
        "gate_candidate_count": gate_candidates,
        "passage_candidate_count": passage_candidates,
    }
    _run_retryable_stage(
        "build_structure",
        context=context,
        operation=lambda: _validate_candidate_connectivity(room_selection, candidates),
    )
    return _Structure(
        partition=partition,
        adjacency=adjacency,
        room_selection=room_selection,
        candidates=candidates,
    )


def _attempt_corridor_world(
    config: Config,
    rng: random.Random,
    attempt: int,
    diagnostics: _RetryDiagnostics,
) -> GeneratedWorld:
    del diagnostics
    base_context = {"attempt": attempt, "seed": config.random_seed}
    layout = _run_retryable_stage(
        "corridor_layout",
        context=base_context,
        operation=lambda: generate_corridor_layout(config, rng),
    )
    attempt_config = layout.config
    passage_geometry = PassageGeometryLayout(
        opening_layout=layout.opening_layout,
        cells=(),
    )
    wall_layout = _run_retryable_stage(
        "walls",
        context=base_context,
        operation=lambda: generate_walls(
            layout.opening_layout,
            layout.adjacency,
            attempt_config,
            passage_geometry,
        ),
    )
    fixture_layout = _run_retryable_stage(
        "fixtures",
        context=base_context,
        operation=lambda: generate_fixtures(wall_layout, attempt_config, rng),
    )
    wall_layout = _merge_fixture_walls(wall_layout, fixture_layout)
    occupancy = _run_retryable_stage(
        "map_task",
        context=base_context,
        operation=lambda: generate_occupancy_map(
            wall_layout, attempt_config, rng, fixture_layout=fixture_layout
        ),
    )
    layout_document = build_layout_document(
        layout.applied_layout,
        layout.opening_layout,
        wall_layout,
        passage_geometry,
        fixture_layout=fixture_layout,
    )
    return GeneratedWorld(
        config=attempt_config,
        partition=layout.partition,
        adjacency=layout.adjacency,
        room_selection=layout.room_selection,
        candidates=layout.candidates,
        selected_graph=layout.selected_graph,
        applied_layout=layout.applied_layout,
        opening_layout=layout.opening_layout,
        passage_geometry=passage_geometry,
        wall_layout=wall_layout,
        fixture_layout=fixture_layout,
        occupancy=occupancy,
        layout_document=layout_document,
        attempt=attempt,
    )


def _attempt_selection(
    structure: _Structure,
    config: Config,
    rng: random.Random,
    attempt: int,
    selection_retry: int,
    selection_attempts: int,
) -> GeneratedWorld:
    adjacency = structure.adjacency
    gate_candidates, passage_candidates = _count_candidate_types(structure.candidates)
    base_context = {
        "attempt": attempt,
        "seed": config.random_seed,
        "cells": len(structure.partition.cells),
        "selected_rooms": structure.room_selection.room_count,
        "candidate_count": len(structure.candidates.connections),
        "gate_candidate_count": gate_candidates,
        "passage_candidate_count": passage_candidates,
        "selection_retry": selection_retry + 1,
        "selection_attempts": selection_attempts,
    }
    selected_graph = _run_retryable_stage(
        "selection",
        context=base_context,
        operation=lambda: select_room_graph(structure.candidates, adjacency, config, rng),
    )
    applied_layout = _run_retryable_stage(
        "apply_connections",
        context=base_context,
        operation=lambda: apply_connections(selected_graph, adjacency),
    )
    _run_retryable_stage(
        "passage_constraints",
        context=base_context,
        operation=lambda: validate_passage_constraints(applied_layout, config),
    )
    opening_layout = _run_retryable_stage(
        "openings",
        context=base_context,
        operation=lambda: generate_openings(applied_layout, config, rng),
    )
    passage_geometry = _run_retryable_stage(
        "passage_geometry",
        context=base_context,
        operation=lambda: generate_passage_geometry(opening_layout, config),
    )
    wall_layout = _run_retryable_stage(
        "walls",
        context=base_context,
        operation=lambda: generate_walls(opening_layout, adjacency, config, passage_geometry),
    )
    fixture_layout = _run_retryable_stage(
        "fixtures",
        context=base_context,
        operation=lambda: generate_fixtures(wall_layout, config, rng),
    )
    wall_layout = _merge_fixture_walls(wall_layout, fixture_layout)
    occupancy = _run_retryable_stage(
        "map_task",
        context=base_context,
        operation=lambda: generate_occupancy_map(
            wall_layout, config, rng, fixture_layout=fixture_layout
        ),
    )
    layout_document = build_layout_document(
        applied_layout,
        opening_layout,
        wall_layout,
        passage_geometry,
        fixture_layout=fixture_layout,
    )

    world = GeneratedWorld(
        config=config,
        partition=structure.partition,
        adjacency=adjacency,
        room_selection=structure.room_selection,
        candidates=structure.candidates,
        selected_graph=selected_graph,
        applied_layout=applied_layout,
        opening_layout=opening_layout,
        passage_geometry=passage_geometry,
        wall_layout=wall_layout,
        fixture_layout=fixture_layout,
        occupancy=occupancy,
        layout_document=layout_document,
        attempt=attempt,
    )
    _run_retryable_stage(
        "map_task",
        context=base_context,
        operation=lambda: validate_world_connectivity(world),
    )
    return world


def _merge_fixture_walls(
    wall_layout: WallLayout,
    fixture_layout: FixtureLayout,
) -> WallLayout:
    if not fixture_layout.extra_wall_segments:
        return wall_layout
    return WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + fixture_layout.extra_wall_segments,
        passage_geometry=wall_layout.passage_geometry,
        unused_solids=wall_layout.unused_solids,
    )


def _validate_output_tree(out_dir: Path) -> None:
    for relative_path in REQUIRED_OUTPUTS:
        path = out_dir / relative_path
        if not path.is_file():
            raise WorldValidationError(f"Missing required output: {path}")

    debug_dir = out_dir / "debug"
    for stage in REQUIRED_DEBUG_STAGES:
        png_path = debug_dir / f"{stage}.png"
        if stage == "09_occupancy_map_preview":
            if not png_path.is_file():
                raise WorldValidationError(f"Missing debug output: {png_path}")
            continue
        svg_path = debug_dir / f"{stage}.svg"
        if not png_path.is_file() or not svg_path.is_file():
            raise WorldValidationError(
                f"Missing debug output: {png_path} or {svg_path}"
            )
