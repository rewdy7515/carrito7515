"""Máscaras HSV y colores de referencia de la pista."""

from __future__ import annotations

import cv2
import numpy as np

RED_RGB, GREEN_RGB, MAGENTA_RGB = (238, 39, 55), (68, 214, 44), (255, 0, 255)


def cmyk_to_rgb(cmyk: tuple[int, int, int, int]) -> tuple[int, int, int]:
    c, m, y, k = (component / 100 for component in cmyk)
    return tuple(round(255 * (1 - component) * (1 - k)) for component in (c, m, y))


ORANGE_RGB = cmyk_to_rgb((0, 60, 100, 0))
BLUE_RGB = cmyk_to_rgb((100, 80, 0, 0))
GRAY_REFERENCE_CMYK = (0, 0, 0, 30)


def rgb_to_hsv(rgb: tuple[int, int, int]) -> np.ndarray:
    return cv2.cvtColor(np.uint8([[list(reversed(rgb))]]), cv2.COLOR_BGR2HSV)[0, 0]


def color_mask(frame_hsv: np.ndarray, rgb: tuple[int, int, int], hue_tolerance: int = 12, minimum_saturation: int = 80, minimum_value: int = 60) -> np.ndarray:
    target_hue = int(rgb_to_hsv(rgb)[0])
    low_hue, high_hue = target_hue - hue_tolerance, target_hue + hue_tolerance
    if low_hue < 0:
        return cv2.bitwise_or(cv2.inRange(frame_hsv, np.array([0, minimum_saturation, minimum_value]), np.array([high_hue, 255, 255])), cv2.inRange(frame_hsv, np.array([180 + low_hue, minimum_saturation, minimum_value]), np.array([179, 255, 255])))
    if high_hue > 179:
        return cv2.bitwise_or(cv2.inRange(frame_hsv, np.array([low_hue, minimum_saturation, minimum_value]), np.array([179, 255, 255])), cv2.inRange(frame_hsv, np.array([0, minimum_saturation, minimum_value]), np.array([high_hue - 180, 255, 255])))
    return cv2.inRange(frame_hsv, np.array([low_hue, minimum_saturation, minimum_value]), np.array([high_hue, 255, 255]))


def clean_mask(mask: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
