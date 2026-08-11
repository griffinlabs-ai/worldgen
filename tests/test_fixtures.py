from __future__ import annotations

import json
from pathlib import Path

import math
import pytest
import xml.etree.ElementTree as ET
from shapely import contains_xy

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.export_map import build_nav_task, generate_occupancy_map
from random_gazebo_world.export_sdf import (
    _sanitize_sdf_name,
    export_world_sdf,
    validate_world_sdf,
)
from random_gazebo_world.fixtures import (
    EMPTY_FIXTURE_LAYOUT,
    FIXTURE_SPECS,
    FixtureError,
    box_collision_footprint,
    fixture_collision_footprint,
    fixture_count_range_for_kind,
    generate_fixtures,
)
from random_gazebo_world.geometry import Cell
from random_gazebo_world.metadata import (
    build_layout_document,
    export_layout_json,
    fixture_instance_to_dict,
    load_layout_json,
)
from random_gazebo_world.geometry import EPS
from random_gazebo_world.openings import OpeningLayout, generate_openings
from random_gazebo_world.partition import Partition
from random_gazebo_world.pipeline import (
    REQUIRED_DEBUG_STAGES,
    generate_valid_world,
    write_world_outputs,
)
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    RoomSelection,
    SelectedRoomGraph,
)
from random_gazebo_world.walls import WallLayout, generate_walls


FIXTURE_MODELS_DIR = Path(
    "~/tcr/ros_ws/src/utils/tcr_ignition/models"
).expanduser()


def _sample_config(**overrides: float | int | str | bool | None) -> Config:
    values = {
        "world_width": 20.0,
        "world_height": 20.0,
        "min_cell_size": 2.0,
        "max_cell_size": 6.0,
        "min_room_count": 3,
        "max_room_count": 8,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.8,
        "gate_width_max": 1.2,
        "passage_width_min": 0.8,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _large_room_wall_layout(seed: int = 42) -> WallLayout:
    cell = Cell.from_origin_size(0, 0.0, 0.0, 8.0, 6.0)
    partition = Partition(cells=(cell,), world_width=8.0, world_height=6.0)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset({0}))
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    applied = AppliedLayout(
        partition=partition,
        room_selection=selection,
        selected_graph=selected,
        passage_cell_ids=frozenset(),
        logical_openings=(),
    )
    opening_layout = OpeningLayout(applied_layout=applied, openings=())
    config = _sample_config(fixture_mode="restroom_clusters", fixture_models_dir=str(FIXTURE_MODELS_DIR))
    return generate_walls(opening_layout, build_adjacency_graph(partition), config)


def _tiny_room_wall_layout() -> WallLayout:
    cell = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 2.0)
    partition = Partition(cells=(cell,), world_width=2.0, world_height=2.0)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset({0}))
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    applied = AppliedLayout(
        partition=partition,
        room_selection=selection,
        selected_graph=selected,
        passage_cell_ids=frozenset(),
        logical_openings=(),
    )
    opening_layout = OpeningLayout(applied_layout=applied, openings=())
    config = _sample_config(fixture_mode="restroom_clusters", fixture_models_dir=str(FIXTURE_MODELS_DIR))
    return generate_walls(opening_layout, build_adjacency_graph(partition), config)


def test_fixture_mode_defaults_to_none(tmp_path: Path) -> None:
    config = load_config(_write_config(tmp_path, []))
    assert config.fixture_mode == "none"
    assert config.fixture_models_dir is None


def test_fixture_mode_requires_models_dir(tmp_path: Path) -> None:
    config_path = _write_config(
        tmp_path,
        ["fixture_mode: restroom_clusters"],
    )
    with pytest.raises(ConfigError, match="fixture_models_dir"):
        load_config(config_path)


def test_fixture_mode_rejects_missing_models_dir(tmp_path: Path) -> None:
    config_path = _write_config(
        tmp_path,
        [
            "fixture_mode: restroom_clusters",
            "fixture_models_dir: /no/such/models",
        ],
    )
    with pytest.raises(ConfigError, match="existing directory"):
        load_config(config_path)


def test_fixture_mode_rejects_voronoi(tmp_path: Path) -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    config_path = _write_config(
        tmp_path,
        [
            "partition_method: voronoi",
            "fixture_mode: restroom_clusters",
            f"fixture_models_dir: {FIXTURE_MODELS_DIR}",
        ],
    )
    with pytest.raises(ConfigError, match="voronoi"):
        load_config(config_path)


def test_generate_fixtures_none_mode_returns_empty() -> None:
    wall_layout = _large_room_wall_layout()
    config = _sample_config(fixture_mode="none")
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(1))
    assert layout is EMPTY_FIXTURE_LAYOUT or layout.clusters == ()


def test_restroom_clusters_place_three_kinds_per_room() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    kinds = {cluster.kind for cluster in layout.clusters}
    assert kinds == {"toilet", "urinal", "basin"}
    assert len(layout.clusters) == 3


def test_restroom_clusters_footprints_do_not_overlap() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    footprints = layout.footprints
    for left in range(len(footprints)):
        for right in range(left + 1, len(footprints)):
            assert not footprints[left].intersects(footprints[right])


def test_restroom_clusters_counts_within_clamps() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    for cluster in layout.clusters:
        count_min, count_max = fixture_count_range_for_kind(config, cluster.kind)
        assert count_min <= len(cluster.instances) <= count_max


def test_restroom_clusters_honor_configured_toilet_count() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
        fixture_toilet_count_min=2,
        fixture_toilet_count_max=2,
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    toilet_clusters = [cluster for cluster in layout.clusters if cluster.kind == "toilet"]
    assert toilet_clusters
    for cluster in toilet_clusters:
        assert len(cluster.instances) == 2


def test_restroom_clusters_deterministic_for_seed() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    first = generate_fixtures(wall_layout, config, create_seeded_rng(4242))
    second = generate_fixtures(wall_layout, config, create_seeded_rng(4242))
    assert first.instances == second.instances
    assert first.boxes == second.boxes
    assert first.extra_wall_segments == second.extra_wall_segments
    assert first.cubicles == second.cubicles


def test_toilet_cluster_emits_enclosed_cubicles() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
        cubicle_door_width=0.65,
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    toilet_clusters = [cluster for cluster in layout.clusters if cluster.kind == "toilet"]
    assert toilet_clusters
    for cluster in toilet_clusters:
        assert len(cluster.cubicles) == len(cluster.instances)
        pitch = FIXTURE_SPECS["toilet"].pitch
        depth = FIXTURE_SPECS["toilet"].cluster_depth
        for cubicle in cluster.cubicles:
            assert cubicle.polygon.area == pytest.approx(pitch * depth)
            door_length = math.hypot(
                cubicle.door_span.p2[0] - cubicle.door_span.p1[0],
                cubicle.door_span.p2[1] - cubicle.door_span.p1[1],
            )
            assert door_length == pytest.approx(config.cubicle_door_width, abs=1e-3)
        expected_partitions = len(cluster.instances) + 1
        partition_segments = [
            segment
            for segment in cluster.wall_segments
            if abs(segment.length - depth) <= 0.01
        ]
        assert len(partition_segments) == expected_partitions
        front_segments = [
            segment
            for segment in cluster.wall_segments
            if abs(segment.length - (pitch - config.cubicle_door_width)) <= 0.01
        ]
        assert len(front_segments) == len(cluster.instances)


def test_cubicle_wall_segments_use_laminate_material_key() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    assert layout.extra_wall_segments
    for segment in layout.extra_wall_segments:
        assert segment.material_key == "laminate"


def test_wall_segment_material_key_metadata_round_trip() -> None:
    from random_gazebo_world.metadata import (
        wall_segment_from_dict,
        wall_segment_to_dict,
    )
    from random_gazebo_world.walls import WallSegment

    segment = WallSegment(p1=(0.0, 0.0), p2=(1.0, 0.0), material_key="laminate")
    payload = wall_segment_to_dict(segment)
    assert payload["material_key"] == "laminate"
    restored = wall_segment_from_dict(payload)
    assert restored == segment


def test_cubicle_wall_segments_respect_wall_thickness() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    for segment in layout.extra_wall_segments:
        assert segment.length + EPS >= config.wall_thickness


def test_restroom_clusters_raises_on_tiny_room() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _tiny_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    with pytest.raises(FixtureError, match="toilet cluster in room 0"):
        generate_fixtures(wall_layout, config, create_seeded_rng(1))


def test_fixture_collisions_stamp_occupancy_map() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    rng = create_seeded_rng(7)
    layout = generate_fixtures(wall_layout, config, rng)
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + layout.extra_wall_segments,
        passage_geometry=wall_layout.passage_geometry,
        unused_solids=wall_layout.unused_solids,
    )
    occupancy = generate_occupancy_map(
        merged, config, create_seeded_rng(7), fixture_layout=layout
    )
    assert layout.instances
    collision = fixture_collision_footprint(layout.instances[0])
    min_x, min_y, max_x, max_y = collision.bounds
    sample_x = (min_x + max_x) / 2.0
    sample_y = (min_y + max_y) / 2.0
    col = int((sample_x - occupancy.origin_x) / occupancy.resolution)
    row = occupancy.height - 1 - int((sample_y - occupancy.origin_y) / occupancy.resolution)
    assert 0 <= row < occupancy.height
    assert 0 <= col < occupancy.width
    assert int(occupancy.data[row, col]) == 0


def test_cubicle_interior_stays_free_in_occupancy_map() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(99))
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + layout.extra_wall_segments,
        passage_geometry=wall_layout.passage_geometry,
        unused_solids=wall_layout.unused_solids,
    )
    occupancy = generate_occupancy_map(
        merged, config, create_seeded_rng(7), fixture_layout=layout
    )
    toilet_clusters = [cluster for cluster in layout.clusters if cluster.kind == "toilet"]
    assert toilet_clusters
    cubicle = toilet_clusters[0].cubicles[0]
    centroid = cubicle.polygon.centroid
    col = int((centroid.x - occupancy.origin_x) / occupancy.resolution)
    row = occupancy.height - 1 - int((centroid.y - occupancy.origin_y) / occupancy.resolution)
    assert 0 <= row < occupancy.height
    assert 0 <= col < occupancy.width
    assert int(occupancy.data[row, col]) == 254


@pytest.mark.skipif(not FIXTURE_MODELS_DIR.is_dir(), reason="fixture models missing")
def test_corridor_integration_with_fixtures(tmp_path: Path) -> None:
    config = load_config(Path("configs/corridor.yaml"))
    world = generate_valid_world(config.with_seed(4242), max_attempts=200)
    out_dir = tmp_path / "corridor_fixtures"
    write_world_outputs(world, out_dir)

    for relative_path in (
        f"{out_dir.name}.sdf",
        "map.png",
        "layout.json",
        "metadata.json",
    ):
        assert (out_dir / relative_path).is_file()

    assert (out_dir / "debug" / "12_fixtures.png").is_file()
    assert "12_fixtures" in REQUIRED_DEBUG_STAGES

    assert not (out_dir / "meshes" / "fixtures").exists()
    sdf_path = out_dir / f"{out_dir.name}.sdf"
    sdf_text = sdf_path.read_text(encoding="utf-8")
    assert "model://" in sdf_text
    assert "<albedo_map>floor_texture.png</albedo_map>" in sdf_text

    metadata = json.loads((out_dir / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["counts"]["fixture_instances"] > 0
    assert metadata["counts"]["fixtures_toilet"] > 0
    assert metadata["counts"]["fixture_cubicles"] > 0

    loaded = load_layout_json(out_dir / "layout.json")
    assert loaded.fixture_instances
    assert loaded.fixture_cubicles

    validate_world_sdf(
        sdf_path,
        world.wall_layout,
        world.config,
        fixture_layout=world.fixture_layout,
    )

    nav_task = build_nav_task(world.occupancy)
    start = (nav_task["start"]["x"], nav_task["start"]["y"])
    goal = (nav_task["goal"]["x"], nav_task["goal"]["y"])
    for footprint in world.fixture_layout.footprints:
        assert not contains_xy(footprint, [start[0]], [start[1]])
        assert not contains_xy(footprint, [goal[0]], [goal[1]])


def test_export_sdf_uses_model_uri_fixture_meshes(tmp_path: Path) -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(3))
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + layout.extra_wall_segments,
        passage_geometry=wall_layout.passage_geometry,
        unused_solids=wall_layout.unused_solids,
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(merged, config, sdf_path, fixture_layout=layout)
    assert not (tmp_path / "meshes" / "fixtures").exists()

    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    assert world is not None
    for instance in layout.instances:
        model_name = _sanitize_sdf_name(instance.name)
        uri = world.findtext(
            f"./model[@name='{model_name}']/link/visual/geometry/mesh/uri"
        )
        assert uri == f"model://{instance.mesh_relpath}"


def test_layout_document_serializes_fixtures(tmp_path: Path) -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(5))
    document = build_layout_document(
        wall_layout.opening_layout.applied_layout,
        wall_layout.opening_layout,
        wall_layout,
        fixture_layout=layout,
    )
    assert document.fixture_instances
    assert document.fixture_boxes
    assert document.fixture_cubicles

    layout_path = tmp_path / "layout.json"
    export_layout_json(layout_path, document)
    loaded = load_layout_json(layout_path)
    assert loaded.fixture_instances == document.fixture_instances
    assert loaded.fixture_boxes == document.fixture_boxes
    assert loaded.fixture_cubicles == document.fixture_cubicles
    for instance in loaded.fixture_instances:
        payload = fixture_instance_to_dict(instance)
        assert "visual_offset" in payload
        assert payload["visual_offset"]["x"] == pytest.approx(instance.visual_offset.x)


def test_toilet_instances_carry_default_visual_offset() -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(5))
    toilets = [item for item in layout.instances if item.kind == "toilet"]
    assert toilets
    for toilet in toilets:
        assert toilet.visual_offset.x == pytest.approx(-0.458)
        assert toilet.visual_offset.y == pytest.approx(0.0)
        assert toilet.visual_offset.z == pytest.approx(0.0)
        assert toilet.visual_offset.yaw == pytest.approx(0.0)


def test_export_sdf_emits_independent_static_fixture_models(tmp_path: Path) -> None:
    if not FIXTURE_MODELS_DIR.is_dir():
        pytest.skip("fixture models directory not available")
    wall_layout = _large_room_wall_layout()
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(FIXTURE_MODELS_DIR),
    )
    layout = generate_fixtures(wall_layout, config, create_seeded_rng(3))
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + layout.extra_wall_segments,
        passage_geometry=wall_layout.passage_geometry,
        unused_solids=wall_layout.unused_solids,
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(merged, config, sdf_path, fixture_layout=layout)

    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    assert world is not None
    assert world.find("./model[@name='fixtures']") is None

    expected_names = {
        _sanitize_sdf_name(instance.name) for instance in layout.instances
    } | {_sanitize_sdf_name(box.name) for box in layout.boxes}
    fixture_models = [
        model
        for model in world.findall("model")
        if model.get("name") in expected_names
    ]
    assert len(fixture_models) == len(expected_names)

    for model in fixture_models:
        assert model.findtext("static") == "true"
        assert model.find("pose") is not None
        link = model.find("link")
        assert link is not None

    for instance in layout.instances:
        model_name = _sanitize_sdf_name(instance.name)
        model = world.find(f"./model[@name='{model_name}']")
        assert model is not None
        model_pose = (model.find("pose").text or "").split()  # type: ignore[union-attr]
        assert float(model_pose[0]) == pytest.approx(instance.x)
        assert float(model_pose[1]) == pytest.approx(instance.y)
        assert float(model_pose[2]) == pytest.approx(instance.z)
        assert float(model_pose[5]) == pytest.approx(instance.yaw)

        visual = model.find("./link/visual")
        assert visual is not None
        visual_pose = (visual.find("pose").text or "").split()  # type: ignore[union-attr]
        assert float(visual_pose[0]) == pytest.approx(instance.visual_offset.x)
        assert float(visual_pose[1]) == pytest.approx(instance.visual_offset.y)
        assert float(visual_pose[2]) == pytest.approx(instance.visual_offset.z)
        assert float(visual_pose[5]) == pytest.approx(instance.visual_offset.yaw)

        collision = model.find("./link/collision")
        assert collision is not None
        assert (collision.find("pose").text or "").strip() == "0 0 0 0 0 0"  # type: ignore[union-attr]

    for box in layout.boxes:
        model_name = _sanitize_sdf_name(box.name)
        model = world.find(f"./model[@name='{model_name}']")
        assert model is not None
        assert model.findtext("static") == "true"
        model_pose = (model.find("pose").text or "").split()  # type: ignore[union-attr]
        assert float(model_pose[0]) == pytest.approx(box.x)
        assert float(model_pose[5]) == pytest.approx(box.yaw)
        for element_name in ("visual", "collision"):
            element = model.find(f"./link/{element_name}")
            assert element is not None
            assert (element.find("pose").text or "").strip() == "0 0 0 0 0 0"  # type: ignore[union-attr]


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
