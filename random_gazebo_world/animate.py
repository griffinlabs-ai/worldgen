"""Compose the staged debug images into one flipbook GIF.

The generator always renders every stage in ``REQUIRED_DEBUG_STAGES``; this module
only decides which of those already-written PNGs belong in an animation, and stitches
them. Nothing here changes what lands on disk, so the debug contract validated by
``pipeline._validate_output_tree`` is untouched.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from PIL import Image

from random_gazebo_world.config import Config
from random_gazebo_world.fixtures import FixtureLayout
from random_gazebo_world.walls import WallLayout

ANIMATION_FILENAME = "stages.gif"

#: Rendered in every mode, meaningful only where room-graph selection actually runs.
#: Corridor and both two-room modes build their geometry directly, so these three
#: frames show the finished layout under a title describing a stage that never ran --
#: ``04_candidate_connections`` in corridor mode is the clearest example.
PARTITION_ONLY_STAGES = frozenset(
    {
        "03_cell_adjacency_graph",
        "04_candidate_connections",
        "05_selected_room_graph",
    }
)

#: The one debug output that is a raster at map resolution rather than a 1200x1200
#: figure (measured: 400x400, 480x394, 170x80 on three worlds). Letterboxing it into
#: the flipbook is the only way in, and ``10_final_floorplan`` already shows the same
#: free space in the world frame.
NEVER_ANIMATED_STAGES = frozenset({"09_occupancy_map_preview"})


def animation_stages(
    all_stages: Sequence[str],
    config: Config,
    wall_layout: WallLayout,
    fixture_layout: FixtureLayout | None,
) -> tuple[str, ...]:
    """Return the subset of ``all_stages`` worth animating, in render order.

    ``all_stages`` is passed in rather than imported so this module stays free of a
    cycle with :mod:`random_gazebo_world.pipeline`, which owns the stage contract.
    """
    skip = set(NEVER_ANIMATED_STAGES)

    if config.layout_mode != "partition":
        skip |= PARTITION_ONLY_STAGES

    # Gate on the config, not the mode: bsp + restroom_clusters is legal and places
    # real clusters, while a two-room world renders an empty plot titled "Fixtures".
    if config.fixture_mode == "none" or fixture_layout is None:
        skip.add("12_fixtures")

    # Corridor mode uses the whole corridor cell polygon, so there are no strips.
    if not wall_layout.passage_geometry:
        skip.add("11_passage_geometry")

    return tuple(stage for stage in all_stages if stage not in skip)


def write_stage_animation(
    debug_dir: Path,
    stages: Sequence[str],
    fps: float = 1.0,
    max_px: int = 600,
) -> Path | None:
    """Write ``stages.gif`` into ``debug_dir``. Returns the path, or None if skipped.

    Frames are the stage PNGs unchanged: every figure stage is 1200x1200 with axes
    pinned to the world bounds by ``visualize._setup_axes``, so they are already
    pixel-registered and each carries its own title. No resizing beyond the
    ``max_px`` cap, no overlays.
    """
    if not stages:
        return None

    frames: list[Image.Image] = []
    for stage in stages:
        path = debug_dir / f"{stage}.png"
        if not path.is_file():
            raise FileNotFoundError(f"Missing debug frame for animation: {path}")
        with Image.open(path) as handle:
            frame = handle.convert("RGB")
        frame.thumbnail((max_px, max_px), Image.LANCZOS)
        # Adaptive per-frame palette: these are flat-colour matplotlib figures, so
        # 256 colours is lossless in practice and keeps the file an order of
        # magnitude smaller than full-size RGB frames.
        frames.append(frame.convert("P", palette=Image.ADAPTIVE))

    output_path = debug_dir / ANIMATION_FILENAME
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=max(10, round(1000.0 / fps)),  # GIF delay granularity is 10 ms
        loop=0,
        optimize=True,
    )
    return output_path
