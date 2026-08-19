"""Cámara, streaming y detección YOLO/NCNN asíncrona del robot."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import cv2
import numpy as np

from .capture import LatestCameraFrame
from .async_yolo import AsyncYoloDetector
from .config import CAMERA_CALIBRATION_PATH, MODEL_PATH, TUNING_PATH, TuningState
from .ground_projection import load_ground_projection
from .pipeline import analyze_frame
from .rendering import draw_result
from .streaming import FrameStore, start_stream_server
from .types import VisionConfig
from .yolo_ncnn import YoloNcnnDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="0", help="Índice de cámara o dispositivo /dev/video*")
    parser.add_argument("--safe-distance-mm", type=int, default=340)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--stream-host", default="0.0.0.0")
    parser.add_argument("--stream-port", type=int, default=8000)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument(
        "--focus",
        type=float,
        default=None,
        help="Valor UVC de enfoque a congelar; por defecto usa el guardado por calibración o el actual",
    )
    parser.add_argument(
        "--zoom",
        type=float,
        default=None,
        help="Valor UVC de zoom a congelar; por defecto conserva el zoom actual",
    )
    parser.add_argument("--no-vision", action="store_true")
    parser.add_argument("--show-reference-marks", action="store_true")
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--calibration-file", type=Path, default=CAMERA_CALIBRATION_PATH)
    parser.add_argument("--yolo-confidence", type=float, default=0.35)
    parser.add_argument("--yolo-iou", type=float, default=0.45)
    parser.add_argument("--yolo-fps", type=float, default=8.0, help="Máximo de inferencias YOLO por segundo")
    parser.add_argument("--ncnn-threads", type=int, default=3, help="Hilos CPU usados por NCNN")
    parser.add_argument("--stream-fps", type=float, default=20.0, help="Máximo de actualizaciones JPEG por segundo")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    source: int | str = int(args.camera) if args.camera.isdigit() else args.camera
    config = VisionConfig(camera_index=source, safe_distance_mm=args.safe_distance_mm, show_gray_reference_lines=args.show_reference_marks)
    ground_projection = load_ground_projection(args.calibration_file)
    calibrated_focus = (
        ground_projection.camera.focus_value if ground_projection is not None else None
    )
    requested_focus = args.focus if args.focus is not None else calibrated_focus
    try:
        capture = LatestCameraFrame(
            source,
            args.width,
            args.height,
            focus_value=requested_focus,
            zoom_value=args.zoom,
        )
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    try:
        detector = YoloNcnnDetector(args.model, args.yolo_confidence, args.yolo_iou, threads=max(1, args.ncnn_threads))
    except RuntimeError as error:
        capture.close()
        raise SystemExit(str(error)) from error
    display_available = bool(os.environ.get("DISPLAY")) and not args.no_display
    store, tuning = FrameStore(), TuningState(TUNING_PATH)
    if ground_projection is None:
        print("Distancia desactivada: falta config/camera_calibration.json de camera_calibration.py.")
    optics = capture.optics_status
    print(f"Óptica congelada: focus={optics['focus']}, zoom={optics['zoom']}")
    yolo_worker = AsyncYoloDetector(detector)
    server = start_stream_server(args.stream_host, args.stream_port, store, tuning)
    print(f"Video disponible en: http://pirobot.local:{server.server_port}")
    last_yolo_submit_at = float("-inf")
    next_stream_at = 0.0
    yolo_interval = 1.0 / max(args.yolo_fps, 0.1)
    stream_interval = 1.0 / max(args.stream_fps, 0.1)
    try:
        while True:
            now = time.monotonic()
            if now < next_stream_at:
                time.sleep(min(.002, next_stream_at - now))
                continue
            frame = capture.read_latest()
            if frame is None:
                time.sleep(.01)
                continue
            if args.no_vision:
                annotated, wall_mask, line_mask = frame, np.zeros_like(frame), np.zeros_like(frame)
            else:
                tuning_values = tuning.snapshot()
                if now - last_yolo_submit_at >= yolo_interval:
                    yolo_worker.submit(frame, tuning_values)
                    last_yolo_submit_at = now
                result = analyze_frame(frame, config, tuning_values, yolo_worker.snapshot(), ground_projection)
                annotated = draw_result(frame, result, config, tuning_values)
                wall_mask, line_mask = result.wall_mask, result.line_mask
            store.update(
                frame, annotated, wall_mask, line_mask,
                [] if args.no_vision else result.wall_ground_points,
            )
            next_stream_at = now + stream_interval
            if display_available:
                cv2.imshow("Vision del robot", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(.01)
    finally:
        yolo_worker.close()
        capture.close()
        server.shutdown()
        if display_available:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
