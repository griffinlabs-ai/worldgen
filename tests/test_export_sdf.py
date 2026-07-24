from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from shapely.geometry import Polygon

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.export_sdf import (
    export_world_sdf,
    ground_box,
    validate_world_sdf,
    wall_segment_to_box,
)
from random_gazebo_world.geometry import Cell
from random_gazebo_world.openings import OpeningLayout, generate_openings
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.passage_geometry import PassageCellGeometry, PassageGeometryLayout
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    RoomSelection,
    SelectedRoomGraph,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
)
from random_gazebo_world.walls import WallLayout, WallSegment, generate_walls


def _sample_config(**overrides: float | int) -> Config:
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


def _grid_partition() -> Partition:
    cells = (
        Cell.from_origin_size(0, 0.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(1, 5.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(2, 0.0, 5.0, 5.0, 5.0),
        Cell.from_origin_size(3, 5.0, 5.0, 5.0, 5.0),
    )
    return Partition(cells=cells, world_width=10.0, world_height=10.0)


def _build_wall_layout(room_ids: set[int], config: Config, seed: int):
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset(room_ids))
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(seed))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(seed + 1000))
    return generate_walls(opening_layout, adjacency, config)


def _manual_opening_layout(
    partition: Partition,
    *,
    room_ids: frozenset[int] = frozenset(),
    passage_ids: frozenset[int] = frozenset(),
):
    selection = RoomSelection(partition=partition, room_cell_ids=room_ids)
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
        passage_cell_ids=passage_ids,
        logical_openings=(),
    )
    return OpeningLayout(applied_layout=applied, openings=())


def test_wall_segment_to_box_vertical() -> None:
    segment = WallSegment("vertical", 5.0, 0.0, 4.0)
    box = wall_segment_to_box(segment, wall_height=2.5, wall_thickness=0.15, index=0)
    assert box.center_x == pytest.approx(5.0)
    assert box.center_y == pytest.approx(2.0)
    assert box.center_z == pytest.approx(1.25)
    assert box.size_x == pytest.approx(0.15)
    assert box.size_y == pytest.approx(4.0)
    assert box.size_z == pytest.approx(2.5)


def test_wall_segment_to_box_horizontal() -> None:
    segment = WallSegment("horizontal", 3.0, 1.0, 6.0)
    box = wall_segment_to_box(segment, wall_height=2.0, wall_thickness=0.2, index=1)
    assert box.center_x == pytest.approx(3.5)
    assert box.center_y == pytest.approx(3.0)
    assert box.size_x == pytest.approx(5.0)
    assert box.size_y == pytest.approx(0.2)
    assert box.size_z == pytest.approx(2.0)


def test_wall_segment_to_box_uses_segment_height_override() -> None:
    segment = WallSegment("vertical", 5.0, 0.0, 4.0, height=1.8)
    box = wall_segment_to_box(segment, wall_height=2.5, wall_thickness=0.15, index=2)
    assert box.center_z == pytest.approx(0.9)
    assert box.size_z == pytest.approx(1.8)


def test_export_world_sdf_reflects_cubicle_wall_height(tmp_path: Path) -> None:
    fixture_models = Path(
        "/home/griffinlabs/tcr/ros_ws/src/utils/tcr_ignition/models"
    )
    if not fixture_models.is_dir():
        pytest.skip("fixture models directory not available")

    from random_gazebo_world.fixtures import generate_fixtures

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
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(fixture_models),
        cubicle_wall_height=1.8,
    )
    wall_layout = generate_walls(
        opening_layout, build_adjacency_graph(partition), config
    )
    fixture_layout = generate_fixtures(
        wall_layout, config, create_seeded_rng(99)
    )
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + fixture_layout.extra_wall_segments,
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(merged, config, sdf_path, fixture_layout=fixture_layout)

    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    assert world is not None
    walls_model = world.find("./model[@name='walls']")
    assert walls_model is not None
    wall_heights = [
        float((size.text or "").split()[2])
        for size in walls_model.findall(".//collision/geometry/box/size")
        if size.text
    ]
    assert wall_heights
    assert any(height == pytest.approx(1.8) for height in wall_heights)
    assert any(height == pytest.approx(config.wall_height) for height in wall_heights)
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    assert root.tag == "sdf"
    assert root.find("world/model") is not None
    validate_world_sdf(sdf_path, wall_layout, config)


def test_export_world_sdf_uses_static_elements(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    world = ET.parse(sdf_path).getroot().find("world")
    assert world is not None
    for name in ("ground", "walls"):
        model = world.find(f"./model[@name='{name}']")
        assert model is not None
        assert model.get("static") is None
        assert model.findtext("static") == "true"


def test_export_world_sdf_matches_all_wall_segments(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    world = ET.parse(sdf_path).getroot().find("world")
    walls_model = next(
        model for model in world.findall("model") if model.get("name") == "walls"
    )
    link = walls_model.find("link")
    assert link is not None
    wall_collisions = [
        item for item in link.findall("collision")
        if (item.get("name") or "").startswith("wall_")
    ]
    wall_visuals = [
        item for item in link.findall("visual")
        if (item.get("name") or "").startswith("wall_")
    ]
    assert len(wall_collisions) == len(wall_layout.segments)
    assert len(wall_visuals) == len(wall_layout.segments)


def test_hybrid_export_unused_bsp_cell_as_one_solid_box(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1}, config, 42)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    world = ET.parse(sdf_path).getroot().find("world")
    assert world is not None
    link = world.find("./model[@name='walls']/link")
    assert link is not None
    solid_boxes = [
        item for item in link.findall("collision")
        if (item.get("name") or "").startswith("solid_")
        and item.find("./geometry/box") is not None
    ]
    assert len(solid_boxes) == len(wall_layout.unused_solids)
    sizes = [box.findtext("./geometry/box/size") for box in solid_boxes]
    assert sizes == ["5.000000 5.000000 2.500000"] * len(wall_layout.unused_solids)
    assert link.find("collision/geometry/polyline") is None


def test_hybrid_export_decomposes_orthogonal_leftover(tmp_path: Path) -> None:
    config = _sample_config()
    cell = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 2.0)
    partition = Partition(cells=(cell,), world_width=2.0, world_height=2.0)
    opening_layout = _manual_opening_layout(partition, passage_ids=frozenset({0}))
    leftover = Polygon(
        [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 2.0),
            (0.0, 2.0),
        ]
    )
    passage_geometry = PassageGeometryLayout(
        opening_layout=opening_layout,
        cells=(
            PassageCellGeometry(
                cell_id=0,
                corridor=cell.polygon.difference(leftover),
                solids=(leftover,),
            ),
        ),
    )
    wall_layout = WallLayout(
        opening_layout=opening_layout,
        segments=(),
        passage_geometry=passage_geometry,
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    link = ET.parse(sdf_path).getroot().find("./world/model[@name='walls']/link")
    assert link is not None
    sizes = sorted(
        item.findtext("./geometry/box/size")
        for item in link.findall("collision")
        if item.find("./geometry/box") is not None
    )
    assert sizes == ["1.000000 1.000000 2.500000", "2.000000 1.000000 2.500000"]
    assert link.find("collision/geometry/mesh") is None


def test_hybrid_export_general_polygon_as_mesh(tmp_path: Path) -> None:
    config = _sample_config()
    cell = Cell.from_polygon(
        0,
        (
            (0.0, 0.0),
            (2.0, 0.0),
            (2.4, 1.1),
            (1.0, 2.0),
            (-0.2, 1.0),
        ),
    )
    partition = Partition(cells=(cell,), world_width=3.0, world_height=3.0)
    opening_layout = _manual_opening_layout(partition)
    wall_layout = WallLayout(
        opening_layout=opening_layout,
        segments=(),
        unused_solids=(cell.polygon,),
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    link = ET.parse(sdf_path).getroot().find("./world/model[@name='walls']/link")
    assert link is not None
    mesh_uri = link.findtext("collision/geometry/mesh/uri")
    assert mesh_uri is not None
    assert mesh_uri.startswith("file://")
    mesh_path = Path(mesh_uri.removeprefix("file://"))
    assert mesh_path.is_file()
    assert mesh_path.read_text(encoding="utf-8").startswith("# Generated")
    assert link.find("collision/geometry/polyline") is None


def test_polyline_export_mode_keeps_legacy_solid_polylines(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1}, config, 42)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(
        wall_layout,
        config,
        sdf_path,
        solid_export_mode="polyline",
    )

    link = ET.parse(sdf_path).getroot().find("./world/model[@name='walls']/link")
    assert link is not None
    assert link.find("collision/geometry/polyline") is not None


def test_ground_box_matches_world_dimensions() -> None:
    config = _sample_config(ground_thickness=0.2)
    box = ground_box(config)
    assert box.center_x == pytest.approx(10.0)
    assert box.center_y == pytest.approx(10.0)
    assert box.center_z == pytest.approx(-0.1)
    assert box.size_x == pytest.approx(20.0)
    assert box.size_y == pytest.approx(20.0)
    assert box.size_z == pytest.approx(0.2)


def test_export_world_sdf_includes_ground_model(tmp_path: Path) -> None:
    config = _sample_config(ground_thickness=0.15)
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    world = ET.parse(sdf_path).getroot().find("world")
    ground_model = next(
        model for model in world.findall("model") if model.get("name") == "ground"
    )
    assert ground_model.get("static") is None
    assert ground_model.findtext("static") == "true"

    collision_size = ground_model.find("link/collision/geometry/box/size")
    assert collision_size is not None
    assert collision_size.text.split() == ["20.000000", "20.000000", "0.150000"]

    collision_pose = ground_model.find("link/collision/pose")
    assert collision_pose is not None
    assert collision_pose.text.startswith("10.000000 10.000000 -0.075000")


def test_export_world_sdf_uses_np_world_lighting_and_material(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    root = ET.parse(sdf_path).getroot()
    assert root.get("version") == "1.10"

    world = root.find("world")
    assert world is not None
    assert world.find("scene/ambient").text == "0.280000 0.280000 0.280000 1"
    assert world.find("scene/background").text == "0.700000 0.700000 0.700000 1"
    assert world.find("scene/shadows").text == "true"
    assert world.find("gravity").text == "0 0 -9.8000000000000007"
    assert world.find("physics").get("type") == "ignored"
    assert world.find("physics/max_step_size").text == "0.001"
    assert world.find("atmosphere") is not None
    assert world.findall("plugin") == []

    sun = next(
        light for light in world.findall("light") if light.get("name") == "sun"
    )
    fill = next(
        light
        for light in world.findall("light")
        if light.get("name") == "fill_light"
    )
    assert sun.get("type") == "directional"
    assert fill.get("type") == "directional"

    walls_model = next(
        model for model in world.findall("model") if model.get("name") == "walls"
    )
    visual = walls_model.find("link/visual")
    assert visual is not None
    material = visual.find("material")
    assert material.find("lighting").text == "true"
    assert material.find("ambient").text == "0.219999999 0.25 0.270000011 1"
    assert material.find("diffuse").text == "0.8 0.8 0.8 1"
    assert material.find("pbr/metal/metalness").text == "0.0"
    assert material.find("pbr/metal/roughness").text == "0.85"


def test_generated_world_sdf_exports(tmp_path: Path) -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(99))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(1001))
    wall_layout = generate_walls(opening_layout, adjacency, config)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)
    validate_world_sdf(sdf_path, wall_layout, config)


def test_export_world_sdf_with_textures_enabled(tmp_path: Path) -> None:
    config = _sample_config(textures_enabled=True, random_seed=7)
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    texture_path = tmp_path / "floor_texture.png"
    assert texture_path.is_file()

    world = ET.parse(sdf_path).getroot().find("world")
    assert world is not None

    ground_visual = world.find("./model[@name='ground']/link/visual")
    assert ground_visual is not None
    albedo_map = ground_visual.findtext("./material/pbr/metal/albedo_map")
    assert albedo_map is not None
    assert albedo_map.startswith("file://")
    assert texture_path.resolve().as_posix() in albedo_map

    walls_link = world.find("./model[@name='walls']/link")
    assert walls_link is not None
    skirt_visuals = [
        item
        for item in walls_link.findall("visual")
        if (item.get("name") or "").startswith("skirt_")
    ]
    assert len(skirt_visuals) == len(wall_layout.segments)

    wall_visual = walls_link.find("./visual[@name='wall_0_visual']")
    assert wall_visual is not None
    assert wall_visual.findtext("./material/diffuse") == "0.880000 0.870000 0.830000 1"

    validate_world_sdf(sdf_path, wall_layout, config)


def test_export_world_sdf_textures_disabled_unchanged(tmp_path: Path) -> None:
    config = _sample_config(textures_enabled=False)
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    assert not (tmp_path / "floor_texture.png").exists()

    walls_model = ET.parse(sdf_path).getroot().find("./world/model[@name='walls']")
    assert walls_model is not None
    link = walls_model.find("link")
    assert link is not None
    assert not any(
        (item.get("name") or "").startswith("skirt_") for item in link.findall("visual")
    )

    visual = link.find("visual")
    assert visual is not None
    material = visual.find("material")
    assert material.find("lighting").text == "true"
    assert material.find("pbr/metal/roughness").text == "0.85"
    validate_world_sdf(sdf_path, wall_layout, config)


def test_export_world_sdf_point_lighting_mode(tmp_path: Path) -> None:
    config = _sample_config(lighting_mode="point", corridor_light_spacing=4.0)
    wall_layout = _build_wall_layout({0, 1, 3}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    world = ET.parse(sdf_path).getroot().find("world")
    assert world is not None
    lights = world.findall("light")
    assert lights
    assert all(light.get("type") == "point" for light in lights)
    assert world.find("./light[@name='sun']") is None
    assert world.find("./light[@name='fill_light']") is None

    room_lights = [
        light for light in lights if (light.get("name") or "").startswith("room_light_")
    ]
    assert len(room_lights) == 3
    assert all(
        light.findtext("cast_shadows") == "true" for light in room_lights
    )
    assert all(
        light.findtext("./attenuation/range") == "10" for light in lights
    )

    passage_lights = [
        light
        for light in lights
        if (light.get("name") or "").startswith("passage_light_")
    ]
    assert passage_lights
    assert all(
        light.findtext("cast_shadows") == "false" for light in passage_lights
    )


def test_export_world_sdf_ode_physics_profile(tmp_path: Path) -> None:
    config = _sample_config(physics_profile="ode")
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    world = ET.parse(sdf_path).getroot().find("world")
    assert world is not None
    plugins = world.findall("plugin")
    assert len(plugins) == 4
    assert {
        plugin.get("filename") for plugin in plugins
    } == {
        "libignition-gazebo-physics-system.so",
        "libignition-gazebo-user-commands-system.so",
        "libignition-gazebo-scene-broadcaster-system.so",
        "libignition-gazebo-forcetorque-system.so",
    }

    physics = world.find("physics")
    assert physics is not None
    assert physics.get("type") == "ode"
    assert physics.findtext("max_step_size") == "0.02"
    assert physics.findtext("real_time_update_rate") == "50"
    assert physics.findtext("./ode/solver/type") == "quick"
    assert physics.findtext("./ode/solver/iters") == "50"
    assert physics.findtext("./ode/constraints/erp") == "0.8"


def test_export_world_sdf_laminate_cubicle_walls_skip_skirting(tmp_path: Path) -> None:
    fixture_models = Path(
        "/home/griffinlabs/tcr/ros_ws/src/utils/tcr_ignition/models"
    )
    if not fixture_models.is_dir():
        pytest.skip("fixture models directory not available")

    from random_gazebo_world.fixtures import generate_fixtures

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
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(fixture_models),
        textures_enabled=True,
        cubicle_wall_color=(0.36, 0.47, 0.55),
    )
    wall_layout = generate_walls(
        opening_layout, build_adjacency_graph(partition), config
    )
    fixture_layout = generate_fixtures(
        wall_layout, config, create_seeded_rng(99)
    )
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + fixture_layout.extra_wall_segments,
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(merged, config, sdf_path, fixture_layout=fixture_layout)

    world = ET.parse(sdf_path).getroot().find("world")
    walls_link = world.find("./model[@name='walls']/link")
    assert walls_link is not None

    laminate_visuals = [
        visual
        for visual in walls_link.findall("visual")
        if visual.findtext("./material/diffuse")
        == "0.360000 0.470000 0.550000 1"
    ]
    assert laminate_visuals

    skirt_visuals = [
        item
        for item in walls_link.findall("visual")
        if (item.get("name") or "").startswith("skirt_")
    ]
    painted_wall_count = sum(
        1 for segment in merged.segments if segment.material_key != "laminate"
    )
    assert len(skirt_visuals) == painted_wall_count
    validate_world_sdf(sdf_path, merged, config, fixture_layout=fixture_layout)


def test_export_world_sdf_counter_specular_and_friction(tmp_path: Path) -> None:
    fixture_models = Path(
        "/home/griffinlabs/tcr/ros_ws/src/utils/tcr_ignition/models"
    )
    if not fixture_models.is_dir():
        pytest.skip("fixture models directory not available")

    from random_gazebo_world.fixtures import generate_fixtures

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
    config = _sample_config(
        fixture_mode="restroom_clusters",
        fixture_models_dir=str(fixture_models),
        textures_enabled=True,
        counter_specular=(0.4, 0.4, 0.4),
        fixture_friction_mu=10000.2,
    )
    wall_layout = generate_walls(
        opening_layout, build_adjacency_graph(partition), config
    )
    fixture_layout = generate_fixtures(
        wall_layout, config, create_seeded_rng(99)
    )
    merged = WallLayout(
        opening_layout=wall_layout.opening_layout,
        segments=wall_layout.segments + fixture_layout.extra_wall_segments,
    )
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(merged, config, sdf_path, fixture_layout=fixture_layout)

    world = ET.parse(sdf_path).getroot().find("world")
    counter_models = [
        model
        for model in world.findall("model")
        if (model.get("name") or "").endswith("_counter")
    ]
    assert counter_models
    counter = counter_models[0]
    assert counter.findtext("./link/visual/material/specular") == (
        "0.400000 0.400000 0.400000 1"
    )
    assert counter.findtext(
        "./link/collision/surface/friction/ode/mu"
    ) == "10000.200000"

    cabinet_models = [
        model
        for model in world.findall("model")
        if (model.get("name") or "").endswith("_cabinet")
    ]
    assert cabinet_models
    cabinet = cabinet_models[0]
    assert cabinet.findtext(
        "./link/collision/surface/friction/ode/mu2"
    ) == "10000.200000"
