from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image
from shapely import contains_xy

from random_gazebo_world.config import Config
from random_gazebo_world.topology import AppliedLayout, CellRole

FLOOR_TEXTURE_NAME = "floor_texture.png"
PIXELS_PER_METRE = 64

WALL_PAINT = (0.88, 0.87, 0.83)
SOLID_PAINT = (0.78, 0.77, 0.73)
SKIRT_COLOR = (0.25, 0.25, 0.27)

FLOOR_ROUGHNESS = 0.35
WALL_ROUGHNESS = 0.85
WALL_METALNESS = 0.0

FLOOR_BASE_RGB = np.array([201.0, 199.0, 193.0], dtype=float)
FLOOR_GROUT_RGB = np.array([152.0, 150.0, 145.0], dtype=float)

ROOM_BASE_TINT = np.array([4.0, 2.0, 0.0], dtype=float)
PASSAGE_TINT = np.array([-3.0, -2.0, 2.0], dtype=float)


def floor_texture_path(output_dir: Path) -> Path:
    return output_dir / FLOOR_TEXTURE_NAME


def generate_floor_texture(
    output_path: Path,
    config: Config,
    applied_layout: AppliedLayout,
) -> Path:
    width_px = max(1, int(round(config.world_width * PIXELS_PER_METRE)))
    height_px = max(1, int(round(config.world_height * PIXELS_PER_METRE)))
    tile_px = max(1, int(round(config.floor_tile_size * PIXELS_PER_METRE)))

    rng = np.random.default_rng(config.random_seed)
    image = _base_tile_texture(width_px, height_px, tile_px, rng)
    image = _apply_region_tints(image, config, applied_layout)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.clip(image, 0, 255).astype(np.uint8)).save(output_path)
    return output_path


def _base_tile_texture(
    width_px: int,
    height_px: int,
    tile_px: int,
    rng: np.random.Generator,
) -> np.ndarray:
    image = np.zeros((height_px, width_px, 3), dtype=float)
    for row in range(0, height_px, tile_px):
        for col in range(0, width_px, tile_px):
            tint = rng.normal(0, 1.2, 3) + rng.normal(0, 6)
            image[row : row + tile_px, col : col + tile_px] = FLOOR_BASE_RGB + tint

    grout_width = min(2, tile_px)
    for row in range(0, height_px, tile_px):
        image[row : row + grout_width, :] = FLOOR_GROUT_RGB
    for col in range(0, width_px, tile_px):
        image[:, col : col + grout_width] = FLOOR_GROUT_RGB

    image += rng.normal(0, 2.5, (height_px, width_px, 1))
    return image


def _apply_region_tints(
    image: np.ndarray,
    config: Config,
    applied_layout: AppliedLayout,
) -> np.ndarray:
    height_px, width_px = image.shape[:2]
    cols = np.arange(width_px, dtype=float)
    rows = np.arange(height_px, dtype=float)
    x_coords = (cols + 0.5) / PIXELS_PER_METRE
    y_coords = (height_px - rows - 0.5) / PIXELS_PER_METRE
    grid_x, grid_y = np.meshgrid(x_coords, y_coords)

    tint = np.zeros((height_px, width_px, 3), dtype=float)
    for cell in applied_layout.partition.cells:
        role = applied_layout.role_for(cell.id)
        if role is CellRole.UNUSED:
            continue

        if role is CellRole.ROOM:
            role_tint = _room_tint(config.random_seed, cell.id)
        else:
            role_tint = PASSAGE_TINT

        mask = contains_xy(cell.polygon, grid_x, grid_y)
        tint[mask] = role_tint

    return image + tint


def _room_tint(seed: int, cell_id: int) -> np.ndarray:
    room_rng = np.random.default_rng(seed + cell_id * 9973)
    return ROOM_BASE_TINT + room_rng.normal(0.0, 1.0, 3)


def format_color(color: tuple[float, float, float]) -> str:
    return f"{color[0]:.6f} {color[1]:.6f} {color[2]:.6f} 1"
