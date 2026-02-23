# Homografía: pixel (imagen pista_qcar2.png) <-> coordenadas cartesianas (mapa / QLabs).
# Referencias: Substack (getPerspectiveTransform vs findHomography), OpenCV Feature Homography,
# GeeksforGeeks Object Tracking using Homography.

from .transform import (
    CIRCUIT_IMAGE_FILENAME,
    load_calibration,
    pixel_to_world,
    world_to_pixel,
    set_calibration_path,
    get_circuit_image_path,
    get_circuit_image_size,
)

__all__ = [
    "CIRCUIT_IMAGE_FILENAME",
    "load_calibration",
    "pixel_to_world",
    "world_to_pixel",
    "set_calibration_path",
    "get_circuit_image_path",
    "get_circuit_image_size",
]
