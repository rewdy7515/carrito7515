"""Proyección de la base del obstáculo sobre el plano del suelo."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

from camera_calibration import CameraGeometry, image_to_ground, load_config

from .types import Detection


@dataclass(frozen=True)
class GroundProjection:
    """Proyección basada en el ``CameraGeometry`` de camera_calibration.py."""

    camera: CameraGeometry

    def project_pixel(self, pixel: tuple[int, int], frame_shape: tuple[int, ...]) -> tuple[float, float] | None:
        """Proyecta un píxel actual usando la misma calibración guardada."""
        frame_height, frame_width = frame_shape[:2]
        calibrated_u = pixel[0] * self.camera.width_px / frame_width
        calibrated_v = pixel[1] * self.camera.height_px / frame_height
        return image_to_ground(calibrated_u, calibrated_v, self.camera)

    def project(self, detection: Detection, frame_shape: tuple[int, ...]) -> Detection:
        """Usa el centro inferior de la caja, donde el objeto toca el suelo."""
        x, y, width, height = detection.bounding_box
        bottom_center = (round(x + width / 2), y + height)
        ground = self.project_pixel(bottom_center, frame_shape)
        if ground is None:
            return replace(detection, bottom_center=bottom_center)
        forward_cm, lateral_cm = ground
        return replace(
            detection,
            bottom_center=bottom_center,
            forward_cm=forward_cm,
            lateral_cm=lateral_cm,
            distance_cm=math.hypot(forward_cm, lateral_cm),
            distance_is_estimated=False,
        )


def load_ground_projection(path: Path) -> GroundProjection | None:
    """Carga únicamente la calibración guardada por camera_calibration.py."""
    if not path.is_file():
        return None
    try:
        camera, _ = load_config(path)
    except (OSError, ValueError, TypeError, KeyError):
        return None
    return GroundProjection(camera)
