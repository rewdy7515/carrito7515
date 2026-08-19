"""Proyección de la zona de seguridad sobre la imagen."""

from __future__ import annotations

import math
import numpy as np


def safe_zone_polygon(frame_shape: tuple[int, ...], tuning: dict, calibration=None, vehicle_width_cm: float = 14.6, vehicle_length_cm: float = 21.15) -> np.ndarray:
    height, width = frame_shape[:2]
    if calibration is None:
        top_y = int(height * tuning["safe_zone_top_ratio"])
        return np.array([[0, height - 1], [width - 1, height - 1], [int(width * .82), top_y], [int(width * .18), top_y]], dtype=np.int32)
    safe_distance = max(vehicle_length_cm / 2 + 1, float(tuning["safe_distance_mm"]) / 10)
    def ground_to_pixel(x_cm: float, y_cm: float) -> tuple[int, int]:
        u = calibration.center_x_px + calibration.focal_x_px * y_cm / max(x_cm, 1e-6)
        angle = math.atan2(calibration.height_cm, max(x_cm, 1e-6)) - math.radians(calibration.pitch_deg)
        v = calibration.center_y_px + calibration.focal_y_px * math.tan(angle)
        return round(u * width / calibration.width_px), round(v * height / calibration.height_px)
    half_width = vehicle_width_cm / 2
    near = vehicle_length_cm / 2
    return np.array([ground_to_pixel(near, -half_width), ground_to_pixel(near, half_width), ground_to_pixel(safe_distance, half_width), ground_to_pixel(safe_distance, -half_width)], dtype=np.int32)
