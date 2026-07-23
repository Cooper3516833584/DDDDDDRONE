"""Pure image-processing helpers for the simulation environment."""

from .takeoff_rectangle import TakeoffRectangle, detect_takeoff_rectangle, draw_takeoff_rectangle
from .terrain_ring import TerrainRing, detect_nearest_terrain_ring, draw_terrain_ring
from .camera_offsets import detect_nearest_ring_offset, detect_takeoff_point_offset

__all__ = [
    "TakeoffRectangle",
    "detect_takeoff_rectangle",
    "draw_takeoff_rectangle",
    "TerrainRing",
    "detect_nearest_terrain_ring",
    "draw_terrain_ring",
    "detect_takeoff_point_offset",
    "detect_nearest_ring_offset",
]
