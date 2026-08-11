from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Literal
from xml.dom import minidom

from shapely.geometry import Point, Polygon
from shapely.ops import triangulate

from random_gazebo_world.config import Config
from random_gazebo_world.fixtures import (
    EMPTY_FIXTURE_LAYOUT,
    BoxFixture,
    FixtureLayout,
    box_fixture_color,
)
from random_gazebo_world.geometry import EPS, Cell, Rect
from random_gazebo_world.solid_geometry import (
    SolidShape,
    collect_tagged_solids,
    decompose_orthogonal_polygon,
    rect_from_polygon_bounds,
)
from random_gazebo_world.textures import (
    FLOOR_ROUGHNESS,
    FLOOR_TEXTURE_NAME,
    SKIRT_COLOR,
    SOLID_PAINT,
    WALL_PAINT,
    floor_texture_path,
    format_color,
    generate_floor_texture,
)
from random_gazebo_world.topology import CellRole
from random_gazebo_world.walls import WallLayout, WallSegment


class SdfExportError(RuntimeError):
    """Raised when SDF export or validation fails."""


SCENE_AMBIENT = "0.25 0.25 0.25 1"
SCENE_BACKGROUND = "0.550000012 0.600000024 0.649999976 1"
GRAVITY = "0 0 -9.8000000000000007"
MAGNETIC_FIELD = "5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05"

IGNITION_PLUGINS: tuple[tuple[str, str], ...] = (
    (
        "ignition::gazebo::systems::Physics",
        "libignition-gazebo-physics-system.so",
    ),
    (
        "ignition::gazebo::systems::UserCommands",
        "libignition-gazebo-user-commands-system.so",
    ),
    (
        "ignition::gazebo::systems::SceneBroadcaster",
        "libignition-gazebo-scene-broadcaster-system.so",
    ),
    (
        "ignition::gazebo::systems::ForceTorque",
        "libignition-gazebo-forcetorque-system.so",
    ),
)

POINT_LIGHT = {
    "intensity": "1.0",
    "direction": "0 0 -1",
    "diffuse": "0.8 0.8 0.8 1",
    "specular": "0.2 0.2 0.2 1",
    "range": "10",
    "linear": "0.1",
    "constant": "0.5",
    "quadratic": "0.05",
}

VISUAL_MATERIAL = {
    "lighting": "true",
    "ambient": "0.219999999 0.25 0.270000011 1",
    "diffuse": "0.8 0.8 0.8 1",
    "specular": "0.0299999993 0.0299999993 0.0299999993 1",
    "shininess": "8",
    "emissive": "0 0 0 1",
    "double_sided": "true",
    "metalness": "0.0",
    "roughness": "0.85",
}

SUN_LIGHT = {
    "name": "sun",
    "pose": "0 0 10 0 0 0",
    "cast_shadows": "true",
    "intensity": "1",
    "direction": "-0.1 0.14 -0.81999999999999995",
    "diffuse": "1 0.959999979 0.860000014 1",
    "specular": "0.100000001 0.100000001 0.100000001 1",
    "range": "1000",
    "linear": "0.0099999997764825821",
    "constant": "0.89999997615814209",
    "quadratic": "0.0010000000474974513",
}

FILL_LIGHT = {
    "name": "fill_light",
    "pose": "0 0 0 0 0 0",
    "cast_shadows": "false",
    "intensity": "1",
    "direction": "0.45000000000000001 -0.65000000000000002 -0.59999999999999998",
    "diffuse": "0.25 0.319999993 0.400000006 1",
    "specular": "0 0 0 1",
    "range": "10",
    "linear": "1",
    "constant": "1",
    "quadratic": "0",
}


@dataclass(frozen=True)
class WallBox:
    name: str
    center_x: float
    center_y: float
    center_z: float
    size_x: float
    size_y: float
    size_z: float
    yaw: float = 0.0
    material_key: str | None = None


@dataclass(frozen=True)
class SolidPolyline:
    name: str
    points: tuple[tuple[float, float], ...]
    height: float


@dataclass(frozen=True)
class SolidMesh:
    name: str
    uri: str


@dataclass(frozen=True)
class SolidGeometryPlan:
    boxes: tuple[WallBox, ...]
    polylines: tuple[SolidPolyline, ...]
    meshes: tuple[SolidMesh, ...]


SolidExportMode = Literal["polyline", "hybrid"]


def export_world_sdf(
    wall_layout: WallLayout,
    config: Config,
    output_path: Path,
    *,
    solid_export_mode: SolidExportMode = "hybrid",
    fixture_layout: FixtureLayout | None = None,
) -> Path:
    if fixture_layout is None:
        fixture_layout = EMPTY_FIXTURE_LAYOUT
    boxes = _wall_boxes(wall_layout, config)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    floor_texture_uri: str | None = None
    if config.textures_enabled:
        texture_path = generate_floor_texture(
            floor_texture_path(output_path.parent),
            config,
            wall_layout.opening_layout.applied_layout,
        )
        floor_texture_uri = FLOOR_TEXTURE_NAME
    solid_plan = _plan_solid_geometry(
        wall_layout,
        config,
        output_path=output_path,
        mode=solid_export_mode,
    )
    fixture_mesh_uris = _build_fixture_mesh_uris(fixture_layout, config)
    world_name = output_path.stem
    tree = _build_sdf_tree(
        boxes,
        solid_plan,
        config,
        wall_layout,
        world_name=world_name,
        textures_enabled=config.textures_enabled,
        floor_texture_uri=floor_texture_uri,
        fixture_layout=fixture_layout,
        fixture_mesh_uris=fixture_mesh_uris,
    )
    _write_pretty_xml(tree, output_path)
    validate_world_sdf(
        output_path,
        wall_layout,
        config,
        solid_export_mode=solid_export_mode,
        fixture_layout=fixture_layout,
    )
    return output_path


def ground_box(config: Config) -> WallBox:
    thickness = config.ground_thickness
    return WallBox(
        name="ground",
        center_x=config.world_width / 2.0,
        center_y=config.world_height / 2.0,
        center_z=-thickness / 2.0,
        size_x=config.world_width,
        size_y=config.world_height,
        size_z=thickness,
    )


def _solid_polygons(wall_layout: WallLayout) -> tuple[Polygon, ...]:
    polygons: list[Polygon] = []
    if wall_layout.passage_geometry is not None:
        polygons.extend(wall_layout.passage_geometry.solids)
    polygons.extend(wall_layout.unused_solids)
    return tuple(polygons)


def _wall_boxes(wall_layout: WallLayout, config: Config) -> list[WallBox]:
    return [
        wall_segment_to_box(segment, config.wall_height, config.wall_thickness, index)
        for index, segment in enumerate(wall_layout.segments)
    ]


def _solid_polylines(wall_layout: WallLayout, config: Config) -> list[SolidPolyline]:
    polylines: list[SolidPolyline] = []
    for index, polygon in enumerate(_solid_polygons(wall_layout)):
        points = _polygon_points(polygon)
        if points is None:
            continue
        polylines.append(
            SolidPolyline(name=f"solid_{index}", points=points, height=config.wall_height)
        )
    return polylines


def _plan_solid_geometry(
    wall_layout: WallLayout,
    config: Config,
    *,
    output_path: Path,
    mode: SolidExportMode,
) -> SolidGeometryPlan:
    if mode == "polyline":
        return SolidGeometryPlan(
            boxes=(),
            polylines=tuple(_solid_polylines(wall_layout, config)),
            meshes=(),
        )
    if mode != "hybrid":
        raise SdfExportError(f"Unsupported solid export mode: {mode!r}")

    boxes: list[WallBox] = []
    meshes: list[SolidMesh] = []
    mesh_dir = output_path.parent / "meshes"
    tagged_solids = collect_tagged_solids(wall_layout)
    for solid_index, solid in enumerate(tagged_solids):
        if solid.shape is SolidShape.AXIS_ALIGNED_RECT:
            rects = (rect_from_polygon_bounds(solid.polygon),)
        elif solid.shape is SolidShape.ORTHOGONAL:
            rects = decompose_orthogonal_polygon(solid.polygon)
        else:
            rects = ()

        if rects:
            for rect_index, rect in enumerate(rects):
                boxes.append(
                    _solid_rect_to_box(
                        rect,
                        name=f"solid_{solid_index}_rect_{rect_index}",
                        height=config.wall_height,
                    )
                )
            continue

        mesh_path = mesh_dir / f"solid_{solid_index}.obj"
        _write_solid_mesh(solid.polygon, mesh_path, height=config.wall_height)
        meshes.append(SolidMesh(name=f"solid_{solid_index}", uri=_mesh_uri(mesh_path)))

    return SolidGeometryPlan(boxes=tuple(boxes), polylines=(), meshes=tuple(meshes))


def _solid_rect_to_box(rect: Rect, *, name: str, height: float) -> WallBox:
    center_x, center_y = rect.center
    return WallBox(
        name=name,
        center_x=center_x,
        center_y=center_y,
        center_z=height / 2.0,
        size_x=rect.width,
        size_y=rect.height,
        size_z=height,
    )


def _polygon_points(polygon: Polygon) -> tuple[tuple[float, float], ...] | None:
    if polygon.is_empty:
        return None
    coords = list(polygon.exterior.coords)
    if len(coords) < 4:
        return None
    return tuple((float(x), float(y)) for x, y in coords[:-1])


def _write_solid_mesh(polygon: Polygon, output_path: Path, *, height: float) -> None:
    triangles = [
        triangle
        for triangle in triangulate(polygon)
        if triangle.area > EPS and polygon.covers(triangle)
    ]
    triangle_area = sum(triangle.area for triangle in triangles)
    if abs(triangle_area - polygon.area) > 1e-6:
        raise SdfExportError(
            f"Could not triangulate solid polygon for mesh export: "
            f"area {triangle_area:.6f} != {polygon.area:.6f}"
        )

    vertices: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []

    def append_face(points: tuple[tuple[float, float, float], ...]) -> None:
        normal = _face_normal(points)
        indices: list[int] = []
        for x, y, z in points:
            vertices.append((round(float(x), 9), round(float(y), 9), round(float(z), 9)))
            normals.append(normal)
            indices.append(len(vertices))
        faces.append(tuple(indices))

    for triangle in triangles:
        coords = list(triangle.exterior.coords)[:3]
        append_face(tuple((x, y, height) for x, y in coords))
        append_face(tuple((x, y, 0.0) for x, y in reversed(coords)))

    for ring in [polygon.exterior, *polygon.interiors]:
        coords = list(ring.coords)
        for (ax, ay), (bx, by) in zip(coords, coords[1:], strict=False):
            append_face(
                (
                    (ax, ay, 0.0),
                    (bx, by, 0.0),
                    (bx, by, height),
                    (ax, ay, height),
                )
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Generated by random_gazebo_world\n"]
    for x, y, z in vertices:
        lines.append(f"v {x:.9f} {y:.9f} {z:.9f}\n")
    for nx, ny, nz in normals:
        lines.append(f"vn {nx:.9f} {ny:.9f} {nz:.9f}\n")
    for face in faces:
        lines.append("f " + " ".join(f"{index}//{index}" for index in face) + "\n")
    output_path.write_text("".join(lines), encoding="utf-8")


def _face_normal(
    points: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    if len(points) < 3:
        return (0.0, 0.0, 1.0)
    ax, ay, az = points[0]
    bx, by, bz = points[1]
    cx, cy, cz = points[2]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length <= EPS:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


def _mesh_uri(path: Path) -> str:
    return f"file://{path.resolve().as_posix()}"


def _model_is_static(model: ET.Element) -> bool:
    return model.get("static") == "true" or model.findtext("static") == "true"


def wall_segment_to_box(
    segment: WallSegment,
    wall_height: float,
    wall_thickness: float,
    index: int,
) -> WallBox:
    effective_height = segment.height if segment.height is not None else wall_height
    center_z = effective_height / 2.0
    length = segment.length
    orientation = segment.orientation

    if orientation == "vertical":
        return WallBox(
            name=f"wall_{index}",
            center_x=segment.fixed_coord,
            center_y=(segment.span_start + segment.span_end) / 2.0,
            center_z=center_z,
            size_x=wall_thickness,
            size_y=length,
            size_z=effective_height,
            material_key=segment.material_key,
        )

    if orientation == "horizontal":
        return WallBox(
            name=f"wall_{index}",
            center_x=(segment.span_start + segment.span_end) / 2.0,
            center_y=segment.fixed_coord,
            center_z=center_z,
            size_x=length,
            size_y=wall_thickness,
            size_z=effective_height,
            material_key=segment.material_key,
        )

    center_x, center_y = segment.midpoint
    return WallBox(
        name=f"wall_{index}",
        center_x=center_x,
        center_y=center_y,
        center_z=center_z,
        size_x=length,
        size_y=wall_thickness,
        size_z=effective_height,
        yaw=segment.yaw,
        material_key=segment.material_key,
    )


def validate_world_sdf(
    sdf_path: Path,
    wall_layout: WallLayout,
    config: Config,
    *,
    solid_export_mode: SolidExportMode = "hybrid",
    fixture_layout: FixtureLayout | None = None,
) -> None:
    if fixture_layout is None:
        fixture_layout = EMPTY_FIXTURE_LAYOUT
    if not sdf_path.is_file():
        raise SdfExportError(f"SDF file not found: {sdf_path}")

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    if root.tag != "sdf":
        raise SdfExportError("Root element must be <sdf>")

    world = root.find("world")
    if world is None:
        raise SdfExportError("SDF must contain a <world> element")

    models = world.findall("model")
    model = next((item for item in models if item.get("name") == "walls"), None)
    if model is None:
        raise SdfExportError("SDF world must contain a walls model")
    if not _model_is_static(model):
        raise SdfExportError("Wall model must be static")

    link = model.find("link")
    if link is None:
        raise SdfExportError("Wall model must contain a <link> element")

    expected_boxes = _wall_boxes(wall_layout, config)

    box_collisions = [
        item for item in link.findall("collision")
        if item.find("./geometry/box") is not None
    ]
    box_visuals = [
        item for item in link.findall("visual")
        if item.find("./geometry/box") is not None
    ]
    polyline_collisions = [
        item for item in link.findall("collision")
        if item.find("./geometry/polyline") is not None
    ]
    polyline_visuals = [
        item for item in link.findall("visual")
        if item.find("./geometry/polyline") is not None
    ]
    mesh_collisions = [
        item for item in link.findall("collision")
        if item.find("./geometry/mesh") is not None
    ]
    mesh_visuals = [
        item for item in link.findall("visual")
        if item.find("./geometry/mesh") is not None
    ]

    wall_box_collisions = [
        item for item in box_collisions if (item.get("name") or "").startswith("wall_")
    ]
    wall_box_visuals = [
        item for item in box_visuals if (item.get("name") or "").startswith("wall_")
    ]

    if len(wall_box_collisions) != len(expected_boxes) or len(wall_box_visuals) != len(
        expected_boxes
    ):
        raise SdfExportError(
            f"Expected {len(expected_boxes)} wall box collision/visual pairs, got "
            f"{len(wall_box_collisions)}/{len(wall_box_visuals)}"
        )
    _validate_solid_geometry_counts(
        sdf_path,
        wall_layout,
        config,
        solid_export_mode,
        box_collisions,
        box_visuals,
        polyline_collisions,
        polyline_visuals,
        mesh_collisions,
        mesh_visuals,
    )

    for index, expected_box in enumerate(expected_boxes):
        collision = wall_box_collisions[index]
        pose = collision.find("pose")
        size = collision.find("./geometry/box/size")
        if pose is None or size is None:
            raise SdfExportError(f"Wall {index} collision missing pose or box size")

        actual_pose = _parse_pose(pose.text or "")
        actual_size = _parse_size(size.text or "")
        expected_pose = (
            expected_box.center_x,
            expected_box.center_y,
            expected_box.center_z,
        )
        expected_size = (
            expected_box.size_x,
            expected_box.size_y,
            expected_box.size_z,
        )
        if not _approx_tuple(actual_pose, expected_pose):
            raise SdfExportError(f"Wall {index} pose mismatch")
        if not _approx_tuple(actual_size, expected_size):
            raise SdfExportError(f"Wall {index} size mismatch")

    _validate_ground_model(world, config)

    if config.textures_enabled:
        painted_wall_count = sum(
            1 for box in expected_boxes if box.material_key != "laminate"
        )
        _validate_texture_exports(
            sdf_path, link, painted_wall_count, world
        )

    if fixture_layout.instances or fixture_layout.boxes:
        _validate_fixtures_model(
            world,
            fixture_layout,
            config,
        )


def _validate_texture_exports(
    sdf_path: Path,
    link: ET.Element,
    painted_wall_count: int,
    world: ET.Element,
) -> None:
    texture_path = floor_texture_path(sdf_path.parent)
    if not texture_path.is_file():
        raise SdfExportError(f"Floor texture not found: {texture_path}")

    skirt_visuals = [
        item
        for item in link.findall("visual")
        if (item.get("name") or "").startswith("skirt_")
    ]
    if len(skirt_visuals) != painted_wall_count:
        raise SdfExportError(
            f"Expected {painted_wall_count} skirt visuals, got {len(skirt_visuals)}"
        )

    ground_model = next(
        (model for model in world.findall("model") if model.get("name") == "ground"),
        None,
    )
    if ground_model is None:
        raise SdfExportError("SDF world must contain a ground model")

    visual = ground_model.find("link/visual")
    if visual is None:
        raise SdfExportError("Ground model must contain a visual")

    albedo_map = visual.findtext("./material/pbr/metal/albedo_map")
    if albedo_map is None:
        raise SdfExportError("Ground visual missing PBR albedo map")
    if albedo_map != FLOOR_TEXTURE_NAME:
        raise SdfExportError(
            f"Ground albedo map mismatch: expected {FLOOR_TEXTURE_NAME!r}, got {albedo_map!r}"
        )


def _validate_solid_geometry_counts(
    sdf_path: Path,
    wall_layout: WallLayout,
    config: Config,
    solid_export_mode: SolidExportMode,
    box_collisions: list[ET.Element],
    box_visuals: list[ET.Element],
    polyline_collisions: list[ET.Element],
    polyline_visuals: list[ET.Element],
    mesh_collisions: list[ET.Element],
    mesh_visuals: list[ET.Element],
) -> None:
    if solid_export_mode == "polyline":
        expected_polylines = _solid_polylines(wall_layout, config)
        if len(polyline_collisions) != len(expected_polylines) or len(
            polyline_visuals
        ) != len(expected_polylines):
            raise SdfExportError(
                f"Expected {len(expected_polylines)} solid polyline pairs, got "
                f"{len(polyline_collisions)}/{len(polyline_visuals)}"
            )
        return

    solid_box_collisions = [
        item for item in box_collisions if (item.get("name") or "").startswith("solid_")
    ]
    solid_box_visuals = [
        item for item in box_visuals if (item.get("name") or "").startswith("solid_")
    ]
    if len(solid_box_collisions) != len(solid_box_visuals):
        raise SdfExportError(
            f"Solid box collision/visual mismatch: "
            f"{len(solid_box_collisions)}/{len(solid_box_visuals)}"
        )
    if len(mesh_collisions) != len(mesh_visuals):
        raise SdfExportError(
            f"Solid mesh collision/visual mismatch: {len(mesh_collisions)}/{len(mesh_visuals)}"
        )
    if polyline_collisions or polyline_visuals:
        raise SdfExportError("Hybrid SDF export must not contain solid polylines")

    expected_solids = collect_tagged_solids(wall_layout)
    if len(solid_box_collisions) + len(mesh_collisions) < len(expected_solids):
        raise SdfExportError(
            f"Expected at least {len(expected_solids)} solid geometries, got "
            f"{len(solid_box_collisions) + len(mesh_collisions)}"
        )

    for mesh in mesh_collisions + mesh_visuals:
        uri = mesh.findtext("./geometry/mesh/uri")
        if uri is None or not uri.startswith("file://"):
            raise SdfExportError(f"Solid mesh has invalid URI: {uri!r}")
        mesh_path = Path(uri.removeprefix("file://"))
        if not mesh_path.is_file():
            raise SdfExportError(f"Solid mesh file not found: {mesh_path}")


def _validate_ground_model(world: ET.Element, config: Config) -> None:
    models = world.findall("model")
    ground_model = next(
        (model for model in models if model.get("name") == "ground"), None
    )
    if ground_model is None:
        raise SdfExportError("SDF world must contain a ground model")
    if not _model_is_static(ground_model):
        raise SdfExportError("Ground model must be static")

    link = ground_model.find("link")
    if link is None:
        raise SdfExportError("Ground model must contain a link")

    expected = ground_box(config)
    collision = link.find("collision")
    visual = link.find("visual")
    if collision is None or visual is None:
        raise SdfExportError("Ground model must contain collision and visual")

    collision_pose = collision.find("pose")
    collision_size = collision.find("./geometry/box/size")
    if collision_pose is None or collision_size is None:
        raise SdfExportError("Ground collision missing pose or box size")

    actual_pose = _parse_pose(collision_pose.text or "")
    actual_size = _parse_size(collision_size.text or "")
    expected_pose = (expected.center_x, expected.center_y, expected.center_z)
    expected_size = (expected.size_x, expected.size_y, expected.size_z)
    if not _approx_tuple(actual_pose, expected_pose):
        raise SdfExportError("Ground pose mismatch")
    if not _approx_tuple(actual_size, expected_size):
        raise SdfExportError("Ground size mismatch")


def _build_sdf_tree(
    boxes: list[WallBox],
    solids: SolidGeometryPlan,
    config: Config,
    wall_layout: WallLayout,
    *,
    world_name: str,
    textures_enabled: bool = False,
    floor_texture_uri: str | None = None,
    fixture_layout: FixtureLayout | None = None,
    fixture_mesh_uris: dict[str, str] | None = None,
) -> ET.ElementTree:
    if fixture_layout is None:
        fixture_layout = EMPTY_FIXTURE_LAYOUT
    if fixture_mesh_uris is None:
        fixture_mesh_uris = {}
    sdf = ET.Element("sdf", version="1.10")
    world = ET.SubElement(sdf, "world", name=world_name)
    _append_world_environment(world, config)
    _append_ground_model(
        world,
        config,
        textures_enabled=textures_enabled,
        floor_texture_uri=floor_texture_uri,
    )
    model = ET.SubElement(world, "model", name="walls")
    static = ET.SubElement(model, "static")
    static.text = "true"
    link = ET.SubElement(model, "link", name="walls_link")

    for index, box in enumerate(boxes):
        if textures_enabled and box.material_key == "laminate":
            material_style = "laminate"
        elif textures_enabled:
            material_style = "wall_paint"
        else:
            material_style = "default"
        _append_box(
            link,
            f"{box.name}_collision",
            box,
            kind="collision",
        )
        _append_box(
            link,
            f"{box.name}_visual",
            box,
            kind="visual",
            material_style=material_style,
            laminate_color=config.cubicle_wall_color,
        )
        if textures_enabled and box.material_key != "laminate":
            _append_skirt_visual(link, box, index)

    for box in solids.boxes:
        _append_box(link, f"{box.name}_collision", box, kind="collision")
        _append_box(
            link,
            f"{box.name}_visual",
            box,
            kind="visual",
            material_style="solid_paint" if textures_enabled else "default",
        )

    for polyline in solids.polylines:
        _append_polyline(
            link,
            f"{polyline.name}_collision",
            polyline,
            kind="collision",
            textures_enabled=textures_enabled,
        )
        _append_polyline(
            link,
            f"{polyline.name}_visual",
            polyline,
            kind="visual",
            textures_enabled=textures_enabled,
        )

    for mesh in solids.meshes:
        _append_mesh(
            link,
            f"{mesh.name}_collision",
            mesh,
            kind="collision",
        )
        _append_mesh(
            link,
            f"{mesh.name}_visual",
            mesh,
            kind="visual",
            textures_enabled=textures_enabled,
        )

    if config.lighting_mode == "point":
        _append_layout_point_lights(world, wall_layout, config)
    else:
        _append_directional_light(world, SUN_LIGHT)
        _append_directional_light(world, FILL_LIGHT)
    if fixture_layout.instances or fixture_layout.boxes:
        _append_fixture_models(
            world,
            fixture_layout,
            config,
            fixture_mesh_uris,
            textures_enabled=textures_enabled,
        )
    return ET.ElementTree(sdf)


def _append_polyline(
    link: ET.Element,
    name: str,
    polyline: SolidPolyline,
    *,
    kind: str,
    textures_enabled: bool = False,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = "0 0 0 0 0 0"

    geometry = ET.SubElement(element, "geometry")
    poly = ET.SubElement(geometry, "polyline")
    for x, y in polyline.points:
        point = ET.SubElement(poly, "point")
        point.text = f"{x:.6f} {y:.6f}"
    height = ET.SubElement(poly, "height")
    height.text = f"{polyline.height:.6f}"

    if kind == "visual":
        _append_visual_material(
            element,
            material_style="solid_paint" if textures_enabled else "default",
        )


def _append_mesh(
    link: ET.Element,
    name: str,
    mesh: SolidMesh,
    *,
    kind: str,
    textures_enabled: bool = False,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = "0 0 0 0 0 0"

    geometry = ET.SubElement(element, "geometry")
    mesh_geometry = ET.SubElement(geometry, "mesh")
    uri = ET.SubElement(mesh_geometry, "uri")
    uri.text = mesh.uri

    if kind == "visual":
        _append_visual_material(
            element,
            material_style="solid_paint" if textures_enabled else "default",
        )


def _append_ground_model(
    world: ET.Element,
    config: Config,
    *,
    textures_enabled: bool = False,
    floor_texture_uri: str | None = None,
) -> None:
    box = ground_box(config)
    model = ET.SubElement(world, "model", name="ground")
    static = ET.SubElement(model, "static")
    static.text = "true"
    link = ET.SubElement(model, "link", name="ground_link")
    _append_box(link, "ground_collision", box, kind="collision")
    _append_box(
        link,
        "ground_visual",
        box,
        kind="visual",
        material_style="ground_texture" if textures_enabled else "default",
        floor_texture_uri=floor_texture_uri,
    )


def _append_world_environment(world: ET.Element, config: Config) -> None:
    if config.physics_profile == "ode":
        for plugin_name, plugin_filename in IGNITION_PLUGINS:
            ET.SubElement(
                world,
                "plugin",
                name=plugin_name,
                filename=plugin_filename,
            )
        physics = ET.SubElement(world, "physics", name="default_physics", type="ode")
        max_step_size = ET.SubElement(physics, "max_step_size")
        max_step_size.text = "0.02"
        real_time_factor = ET.SubElement(physics, "real_time_factor")
        real_time_factor.text = "1"
        real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
        real_time_update_rate.text = "50"
        ode = ET.SubElement(physics, "ode")
        solver = ET.SubElement(ode, "solver")
        solver_type = ET.SubElement(solver, "type")
        solver_type.text = "quick"
        iters = ET.SubElement(solver, "iters")
        iters.text = "50"
        sor = ET.SubElement(solver, "sor")
        sor.text = "1.3"
        constraints = ET.SubElement(ode, "constraints")
        cfm = ET.SubElement(constraints, "cfm")
        cfm.text = "0.0"
        erp = ET.SubElement(constraints, "erp")
        erp.text = "0.8"
        contact_max_correcting_vel = ET.SubElement(
            constraints, "contact_max_correcting_vel"
        )
        contact_max_correcting_vel.text = "100.0"
        contact_surface_layer = ET.SubElement(constraints, "contact_surface_layer")
        contact_surface_layer.text = "0.0001"
    else:
        physics = ET.SubElement(
            world, "physics", name="default_physics", type="ignored"
        )
        max_step_size = ET.SubElement(physics, "max_step_size")
        max_step_size.text = "0.001"
        real_time_factor = ET.SubElement(physics, "real_time_factor")
        real_time_factor.text = "1"
        real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
        real_time_update_rate.text = "1000"

    scene = ET.SubElement(world, "scene")
    ambient = ET.SubElement(scene, "ambient")
    ambient.text = format_color(config.scene_ambient)
    background = ET.SubElement(scene, "background")
    background.text = format_color(config.scene_background)
    shadows = ET.SubElement(scene, "shadows")
    shadows.text = "true"

    gravity = ET.SubElement(world, "gravity")
    gravity.text = GRAVITY
    magnetic_field = ET.SubElement(world, "magnetic_field")
    magnetic_field.text = MAGNETIC_FIELD
    ET.SubElement(world, "atmosphere", type="adiabatic")


def _append_point_light(
    world: ET.Element,
    name: str,
    x: float,
    y: float,
    z: float,
    *,
    cast_shadows: bool,
) -> None:
    light = ET.SubElement(world, "light", name=name, type="point")
    pose = ET.SubElement(light, "pose")
    pose.text = _format_pose(x, y, z)
    cast_shadows_element = ET.SubElement(light, "cast_shadows")
    cast_shadows_element.text = "true" if cast_shadows else "false"
    intensity = ET.SubElement(light, "intensity")
    intensity.text = POINT_LIGHT["intensity"]
    direction = ET.SubElement(light, "direction")
    direction.text = POINT_LIGHT["direction"]
    diffuse = ET.SubElement(light, "diffuse")
    diffuse.text = POINT_LIGHT["diffuse"]
    specular = ET.SubElement(light, "specular")
    specular.text = POINT_LIGHT["specular"]

    attenuation = ET.SubElement(light, "attenuation")
    light_range = ET.SubElement(attenuation, "range")
    light_range.text = POINT_LIGHT["range"]
    linear = ET.SubElement(attenuation, "linear")
    linear.text = POINT_LIGHT["linear"]
    constant = ET.SubElement(attenuation, "constant")
    constant.text = POINT_LIGHT["constant"]
    quadratic = ET.SubElement(attenuation, "quadratic")
    quadratic.text = POINT_LIGHT["quadratic"]


def _append_layout_point_lights(
    world: ET.Element,
    wall_layout: WallLayout,
    config: Config,
) -> None:
    layout = wall_layout.opening_layout.applied_layout
    for cell in layout.partition.cells:
        if layout.role_for(cell.id) is not CellRole.ROOM:
            continue
        centroid_x, centroid_y = cell.centroid
        _append_point_light(
            world,
            f"room_light_{cell.id}",
            centroid_x,
            centroid_y,
            config.light_height,
            cast_shadows=True,
        )

    for cell in layout.partition.cells:
        if layout.role_for(cell.id) is not CellRole.PASSAGE:
            continue
        for index, (x, y) in enumerate(
            _passage_light_positions(cell, wall_layout, config)
        ):
            _append_point_light(
                world,
                f"passage_light_{cell.id}_{index}",
                x,
                y,
                config.light_height,
                cast_shadows=False,
            )


def _passage_walkable_geometry(
    cell: Cell,
    wall_layout: WallLayout,
):
    if wall_layout.passage_geometry is not None:
        corridor = wall_layout.passage_geometry.corridor_for(cell.id)
        if corridor is not None and not corridor.is_empty:
            return corridor
    return cell.polygon


def _passage_light_positions(
    cell: Cell,
    wall_layout: WallLayout,
    config: Config,
) -> list[tuple[float, float]]:
    geometry = _passage_walkable_geometry(cell, wall_layout)
    if geometry.is_empty:
        return []

    centroid = geometry.centroid
    minx, miny, maxx, maxy = geometry.bounds
    span_x = maxx - minx
    span_y = maxy - miny
    spacing = config.corridor_light_spacing

    if span_x >= span_y:
        positions = _sample_axis_light_positions(
            geometry,
            start=minx,
            end=maxx,
            fixed_coord=centroid.y,
            axis="x",
            spacing=spacing,
        )
    else:
        positions = _sample_axis_light_positions(
            geometry,
            start=miny,
            end=maxy,
            fixed_coord=centroid.x,
            axis="y",
            spacing=spacing,
        )

    if positions:
        return positions
    return [(centroid.x, centroid.y)]


def _sample_axis_light_positions(
    geometry,
    *,
    start: float,
    end: float,
    fixed_coord: float,
    axis: Literal["x", "y"],
    spacing: float,
) -> list[tuple[float, float]]:
    span = end - start
    if span <= EPS:
        point = (
            Point(start, fixed_coord)
            if axis == "x"
            else Point(fixed_coord, start)
        )
        if geometry.contains(point) or geometry.touches(point):
            return [(point.x, point.y)]
        centroid = geometry.centroid
        return [(centroid.x, centroid.y)]

    count = max(1, math.ceil(span / spacing))
    step = span / count
    positions: list[tuple[float, float]] = []
    for index in range(count):
        coord = start + (index + 0.5) * step
        point = (
            Point(coord, fixed_coord)
            if axis == "x"
            else Point(fixed_coord, coord)
        )
        if geometry.contains(point) or geometry.touches(point):
            positions.append((point.x, point.y))
    return positions


def _append_directional_light(world: ET.Element, settings: dict[str, str]) -> None:
    light = ET.SubElement(world, "light", name=settings["name"], type="directional")
    pose = ET.SubElement(light, "pose")
    pose.text = settings["pose"]
    cast_shadows = ET.SubElement(light, "cast_shadows")
    cast_shadows.text = settings["cast_shadows"]
    intensity = ET.SubElement(light, "intensity")
    intensity.text = settings["intensity"]
    direction = ET.SubElement(light, "direction")
    direction.text = settings["direction"]
    diffuse = ET.SubElement(light, "diffuse")
    diffuse.text = settings["diffuse"]
    specular = ET.SubElement(light, "specular")
    specular.text = settings["specular"]

    attenuation = ET.SubElement(light, "attenuation")
    light_range = ET.SubElement(attenuation, "range")
    light_range.text = settings["range"]
    linear = ET.SubElement(attenuation, "linear")
    linear.text = settings["linear"]
    constant = ET.SubElement(attenuation, "constant")
    constant.text = settings["constant"]
    quadratic = ET.SubElement(attenuation, "quadratic")
    quadratic.text = settings["quadratic"]

    spot = ET.SubElement(light, "spot")
    inner_angle = ET.SubElement(spot, "inner_angle")
    inner_angle.text = "0"
    outer_angle = ET.SubElement(spot, "outer_angle")
    outer_angle.text = "0"
    falloff = ET.SubElement(spot, "falloff")
    falloff.text = "0"


def _append_visual_material(
    visual: ET.Element,
    *,
    material_style: str = "default",
    floor_texture_uri: str | None = None,
    laminate_color: tuple[float, float, float] | None = None,
) -> None:
    if material_style == "ground_texture":
        if floor_texture_uri is None:
            raise SdfExportError("Ground texture export requires floor_texture_uri")
        material = ET.SubElement(visual, "material")
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = "1 1 1 1"
        specular = ET.SubElement(material, "specular")
        specular.text = "0.1 0.1 0.1 1"
        pbr = ET.SubElement(material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        albedo_map = ET.SubElement(metal, "albedo_map")
        albedo_map.text = floor_texture_uri
        roughness = ET.SubElement(metal, "roughness")
        roughness.text = f"{FLOOR_ROUGHNESS:.6f}"
        return

    if material_style == "wall_paint":
        material = ET.SubElement(visual, "material")
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = format_color(WALL_PAINT)
        return

    if material_style == "laminate":
        if laminate_color is None:
            raise SdfExportError("Laminate wall export requires laminate_color")
        material = ET.SubElement(visual, "material")
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = format_color(laminate_color)
        return

    if material_style == "solid_paint":
        material = ET.SubElement(visual, "material")
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = format_color(SOLID_PAINT)
        return

    material = ET.SubElement(visual, "material")
    lighting = ET.SubElement(material, "lighting")
    lighting.text = VISUAL_MATERIAL["lighting"]
    ambient = ET.SubElement(material, "ambient")
    ambient.text = VISUAL_MATERIAL["ambient"]
    diffuse = ET.SubElement(material, "diffuse")
    diffuse.text = VISUAL_MATERIAL["diffuse"]
    specular = ET.SubElement(material, "specular")
    specular.text = VISUAL_MATERIAL["specular"]
    shininess = ET.SubElement(material, "shininess")
    shininess.text = VISUAL_MATERIAL["shininess"]
    emissive = ET.SubElement(material, "emissive")
    emissive.text = VISUAL_MATERIAL["emissive"]
    double_sided = ET.SubElement(material, "double_sided")
    double_sided.text = VISUAL_MATERIAL["double_sided"]
    pbr = ET.SubElement(material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    metalness = ET.SubElement(metal, "metalness")
    metalness.text = VISUAL_MATERIAL["metalness"]
    roughness = ET.SubElement(metal, "roughness")
    roughness.text = VISUAL_MATERIAL["roughness"]


def _append_box(
    link: ET.Element,
    name: str,
    box: WallBox,
    *,
    kind: str,
    material_style: str = "default",
    floor_texture_uri: str | None = None,
    laminate_color: tuple[float, float, float] | None = None,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = _format_pose(box.center_x, box.center_y, box.center_z, box.yaw)

    geometry = ET.SubElement(element, "geometry")
    box_geometry = ET.SubElement(geometry, "box")
    size = ET.SubElement(box_geometry, "size")
    size.text = _format_size(box.size_x, box.size_y, box.size_z)

    if kind == "visual":
        _append_visual_material(
            element,
            material_style=material_style,
            floor_texture_uri=floor_texture_uri,
            laminate_color=laminate_color,
        )


def _append_skirt_visual(link: ET.Element, box: WallBox, index: int) -> None:
    if box.size_x >= box.size_y:
        skirt_x = box.size_x
        skirt_y = box.size_y + 0.04
    else:
        skirt_x = box.size_x + 0.04
        skirt_y = box.size_y
    skirt_z = -(box.size_z / 2.0) + 0.05
    skirt_center_z = box.center_z + skirt_z

    element = ET.SubElement(link, "visual", name=f"skirt_{index}_visual")
    pose = ET.SubElement(element, "pose")
    pose.text = _format_pose(box.center_x, box.center_y, skirt_center_z, box.yaw)

    geometry = ET.SubElement(element, "geometry")
    box_geometry = ET.SubElement(geometry, "box")
    size = ET.SubElement(box_geometry, "size")
    size.text = _format_size(skirt_x, skirt_y, 0.1)

    material = ET.SubElement(element, "material")
    diffuse = ET.SubElement(material, "diffuse")
    diffuse.text = format_color(SKIRT_COLOR)


def _write_pretty_xml(tree: ET.ElementTree, output_path: Path) -> None:
    xml_bytes = ET.tostring(tree.getroot(), encoding="utf-8")
    pretty = minidom.parseString(xml_bytes).toprettyxml(indent="  ")
    output_path.write_text(pretty, encoding="utf-8")


def _format_pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}"


def _format_size(x: float, y: float, z: float) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f}"


def _parse_pose(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) < 3:
        raise SdfExportError(f"Invalid pose: {text!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _parse_size(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) != 3:
        raise SdfExportError(f"Invalid size: {text!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _parse_pose6(text: str) -> tuple[float, float, float, float, float, float]:
    parts = text.split()
    if len(parts) < 6:
        raise SdfExportError(f"Invalid pose: {text!r}")
    return tuple(float(part) for part in parts[:6])  # type: ignore[return-value]


def _approx_pose6(
    left: tuple[float, float, float, float, float, float],
    right: tuple[float, float, float, float, float, float],
    eps: float = 1e-6,
) -> bool:
    return all(abs(a - b) <= eps for a, b in zip(left, right, strict=True))


def _approx_tuple(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
    eps: float = 1e-6,
) -> bool:
    return all(abs(a - b) <= eps for a, b in zip(left, right, strict=True))


def _fixture_model_mesh_uri(relpath: str) -> str:
    return f"model://{relpath.replace(chr(92), '/')}"


def _build_fixture_mesh_uris(
    fixture_layout: FixtureLayout,
    config: Config,
) -> dict[str, str]:
    if not fixture_layout.mesh_relpaths:
        return {}
    if not config.fixture_models_dir:
        raise SdfExportError("fixture_models_dir is required to export fixture meshes")

    models_dir = config.fixture_models_path
    assert models_dir is not None  # guarded by the falsy check above
    uris: dict[str, str] = {}
    for relpath in sorted(fixture_layout.mesh_relpaths):
        src = models_dir / relpath
        if not src.is_file():
            raise SdfExportError(f"Fixture mesh not found: {src}")
        uris[relpath] = _fixture_model_mesh_uri(relpath)
    return uris


def _append_fixture_models(
    world: ET.Element,
    fixture_layout: FixtureLayout,
    config: Config,
    mesh_uris: dict[str, str],
    *,
    textures_enabled: bool,
) -> None:
    for instance in fixture_layout.instances:
        mesh_uri = mesh_uris[instance.mesh_relpath]
        cx, cy, cz = instance.collision_size
        _append_mesh_fixture_model(
            world,
            model_name=instance.name,
            mesh_uri=mesh_uri,
            world_x=instance.x,
            world_y=instance.y,
            world_z=instance.z,
            world_yaw=instance.yaw,
            visual_offset=instance.visual_offset,
            collision_size=(cx, cy, cz),
        )

    for box in fixture_layout.boxes:
        color = box_fixture_color(box, textures_enabled)
        _append_box_fixture_model(
            world,
            model_name=box.name,
            x=box.x,
            y=box.y,
            z=box.z,
            yaw=box.yaw,
            size_x=box.size_x,
            size_y=box.size_y,
            size_z=box.size_z,
            color=color,
            box=box,
            config=config,
        )


def _append_mesh_fixture_model(
    world: ET.Element,
    *,
    model_name: str,
    mesh_uri: str,
    world_x: float,
    world_y: float,
    world_z: float,
    world_yaw: float,
    visual_offset,
    collision_size: tuple[float, float, float],
) -> None:
    model = ET.SubElement(world, "model", name=_sanitize_sdf_name(model_name))
    static = ET.SubElement(model, "static")
    static.text = "true"
    model_pose = ET.SubElement(model, "pose")
    model_pose.text = _format_pose(world_x, world_y, world_z, world_yaw)
    link = ET.SubElement(model, "link", name=f"{model_name}_link")

    visual = ET.SubElement(link, "visual", name=f"{model_name}_visual")
    visual_pose = ET.SubElement(visual, "pose")
    visual_pose.text = _format_pose(
        visual_offset.x,
        visual_offset.y,
        visual_offset.z,
        visual_offset.yaw,
    )
    visual_geometry = ET.SubElement(visual, "geometry")
    mesh_geometry = ET.SubElement(visual_geometry, "mesh")
    uri = ET.SubElement(mesh_geometry, "uri")
    uri.text = mesh_uri

    cx, cy, cz = collision_size
    collision = ET.SubElement(link, "collision", name=f"{model_name}_collision")
    collision_pose = ET.SubElement(collision, "pose")
    collision_pose.text = "0 0 0 0 0 0"
    collision_geometry = ET.SubElement(collision, "geometry")
    box_geometry = ET.SubElement(collision_geometry, "box")
    size = ET.SubElement(box_geometry, "size")
    size.text = _format_size(cx, cy, cz)


def _append_box_fixture_model(
    world: ET.Element,
    *,
    model_name: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    size_x: float,
    size_y: float,
    size_z: float,
    color: tuple[float, float, float],
    box: BoxFixture,
    config: Config,
) -> None:
    model = ET.SubElement(world, "model", name=_sanitize_sdf_name(model_name))
    static = ET.SubElement(model, "static")
    static.text = "true"
    model_pose = ET.SubElement(model, "pose")
    model_pose.text = _format_pose(x, y, z, yaw)
    link = ET.SubElement(model, "link", name=f"{model_name}_link")

    visual = ET.SubElement(link, "visual", name=f"{model_name}_visual")
    visual_pose = ET.SubElement(visual, "pose")
    visual_pose.text = "0 0 0 0 0 0"
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box")
    visual_size = ET.SubElement(visual_box, "size")
    visual_size.text = _format_size(size_x, size_y, size_z)
    material = ET.SubElement(visual, "material")
    diffuse = ET.SubElement(material, "diffuse")
    diffuse.text = format_color(color)
    if box.color_key == "counter":
        specular = ET.SubElement(material, "specular")
        specular.text = format_color(config.counter_specular)

    collision = ET.SubElement(link, "collision", name=f"{model_name}_collision")
    collision_pose = ET.SubElement(collision, "pose")
    collision_pose.text = "0 0 0 0 0 0"
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_box = ET.SubElement(collision_geometry, "box")
    collision_size = ET.SubElement(collision_box, "size")
    collision_size.text = _format_size(size_x, size_y, size_z)
    if box.color_key in ("counter", "cabinet"):
        surface = ET.SubElement(collision, "surface")
        friction = ET.SubElement(surface, "friction")
        ode = ET.SubElement(friction, "ode")
        mu = ET.SubElement(ode, "mu")
        mu.text = f"{config.fixture_friction_mu:.6f}"
        mu2 = ET.SubElement(ode, "mu2")
        mu2.text = f"{config.fixture_friction_mu:.6f}"


def _sanitize_sdf_name(name: str) -> str:
    sanitized = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name)
    if not sanitized:
        raise SdfExportError(f"Invalid empty SDF model name derived from {name!r}")
    if sanitized[0].isdigit():
        sanitized = f"fixture_{sanitized}"
    return sanitized


def _validate_fixtures_model(
    world: ET.Element,
    fixture_layout: FixtureLayout,
    config: Config,
) -> None:
    aggregate = next(
        (model for model in world.findall("model") if model.get("name") == "fixtures"),
        None,
    )
    if aggregate is not None:
        raise SdfExportError("SDF world must not contain aggregate fixtures model")

    instance_models = {
        _sanitize_sdf_name(instance.name): instance
        for instance in fixture_layout.instances
    }
    box_models = {
        _sanitize_sdf_name(box.name): box for box in fixture_layout.boxes
    }
    expected_model_names = set(instance_models) | set(box_models)

    fixture_models = [
        model
        for model in world.findall("model")
        if model.get("name") in expected_model_names
    ]
    if len(fixture_models) != len(expected_model_names):
        raise SdfExportError(
            f"Expected {len(expected_model_names)} independent fixture models, got "
            f"{len(fixture_models)}"
        )

    for model in fixture_models:
        if not _model_is_static(model):
            raise SdfExportError(f"Fixture model {model.get('name')!r} must be static")

        link = model.find("link")
        if link is None:
            raise SdfExportError(
                f"Fixture model {model.get('name')!r} must contain a link"
            )

        model_name = model.get("name")
        assert model_name is not None

        if model_name in instance_models:
            instance = instance_models[model_name]
            model_pose = model.find("pose")
            if model_pose is None:
                raise SdfExportError(
                    f"Fixture model {model_name!r} missing model pose"
                )
            actual_model_pose = _parse_pose6(model_pose.text or "")
            expected_model_pose = (
                instance.x,
                instance.y,
                instance.z,
                0.0,
                0.0,
                instance.yaw,
            )
            if not _approx_pose6(actual_model_pose, expected_model_pose):
                raise SdfExportError(
                    f"Fixture model {model_name!r} world pose mismatch"
                )

            mesh_visuals = [
                item
                for item in link.findall("visual")
                if item.find("./geometry/mesh") is not None
            ]
            if len(mesh_visuals) != 1:
                raise SdfExportError(
                    f"Fixture model {model_name!r} must contain one mesh visual"
                )
            visual = mesh_visuals[0]
            visual_pose = visual.find("pose")
            if visual_pose is None:
                raise SdfExportError(
                    f"Fixture model {model_name!r} mesh visual missing pose"
                )
            actual_visual_pose = _parse_pose6(visual_pose.text or "")
            expected_visual_pose = (
                instance.visual_offset.x,
                instance.visual_offset.y,
                instance.visual_offset.z,
                0.0,
                0.0,
                instance.visual_offset.yaw,
            )
            if not _approx_pose6(actual_visual_pose, expected_visual_pose):
                raise SdfExportError(
                    f"Fixture model {model_name!r} mesh visual offset mismatch"
                )

            uri = visual.findtext("./geometry/mesh/uri")
            if uri is None:
                raise SdfExportError(
                    f"Fixture model {model_name!r} mesh visual missing URI"
                )
            expected_uri = _fixture_model_mesh_uri(instance.mesh_relpath)
            if uri != expected_uri:
                raise SdfExportError(
                    f"Fixture model {model_name!r} mesh URI mismatch: "
                    f"expected {expected_uri!r}, got {uri!r}"
                )
            if not config.fixture_models_dir:
                raise SdfExportError(
                    "fixture_models_dir is required to validate fixture meshes"
                )
            models_root = config.fixture_models_path
            assert models_root is not None  # guarded by the falsy check above
            mesh_path = models_root / instance.mesh_relpath
            if not mesh_path.is_file():
                raise SdfExportError(f"Fixture mesh file not found: {mesh_path}")

            box_collisions = [
                item
                for item in link.findall("collision")
                if item.find("./geometry/box") is not None
            ]
            if len(box_collisions) != 1:
                raise SdfExportError(
                    f"Fixture model {model_name!r} must contain one box collision"
                )
            collision = box_collisions[0]
            collision_pose = collision.find("pose")
            if collision_pose is None:
                raise SdfExportError(
                    f"Fixture model {model_name!r} collision missing pose"
                )
            if (collision_pose.text or "").strip() != "0 0 0 0 0 0":
                raise SdfExportError(
                    f"Fixture model {model_name!r} collision pose must be zero"
                )
            size = collision.find("./geometry/box/size")
            if size is None:
                raise SdfExportError(
                    f"Fixture model {model_name!r} collision missing box size"
                )
            actual_size = _parse_size(size.text or "")
            if not _approx_tuple(actual_size, instance.collision_size):
                raise SdfExportError(
                    f"Fixture model {model_name!r} collision size mismatch"
                )
            continue

        box = box_models[model_name]
        model_pose = model.find("pose")
        if model_pose is None:
            raise SdfExportError(f"Box fixture model {model_name!r} missing model pose")
        actual_model_pose = _parse_pose6(model_pose.text or "")
        expected_model_pose = (box.x, box.y, box.z, 0.0, 0.0, box.yaw)
        if not _approx_pose6(actual_model_pose, expected_model_pose):
            raise SdfExportError(
                f"Box fixture model {model_name!r} world pose mismatch"
            )

        box_visuals = [
            item
            for item in link.findall("visual")
            if item.find("./geometry/box") is not None
        ]
        box_collisions = [
            item
            for item in link.findall("collision")
            if item.find("./geometry/box") is not None
        ]
        if len(box_visuals) != 1 or len(box_collisions) != 1:
            raise SdfExportError(
                f"Box fixture model {model_name!r} must contain one visual and collision"
            )
        for element in (box_visuals[0], box_collisions[0]):
            pose = element.find("pose")
            if pose is None:
                raise SdfExportError(
                    f"Box fixture model {model_name!r} element missing pose"
                )
            if (pose.text or "").strip() != "0 0 0 0 0 0":
                raise SdfExportError(
                    f"Box fixture model {model_name!r} element pose must be zero"
                )

        visual_size = box_visuals[0].find("./geometry/box/size")
        collision_size = box_collisions[0].find("./geometry/box/size")
        if visual_size is None or collision_size is None:
            raise SdfExportError(
                f"Box fixture model {model_name!r} missing box geometry size"
            )
        expected_size = (box.size_x, box.size_y, box.size_z)
        if not _approx_tuple(_parse_size(visual_size.text or ""), expected_size):
            raise SdfExportError(
                f"Box fixture model {model_name!r} visual size mismatch"
            )
        if not _approx_tuple(_parse_size(collision_size.text or ""), expected_size):
            raise SdfExportError(
                f"Box fixture model {model_name!r} collision size mismatch"
            )

