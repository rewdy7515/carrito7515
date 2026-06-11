import csv
import json
import time
from pathlib import Path

import cv2

# Configuracion simple para ejecutar directamente desde Thonny.
STREAM_URL = "http://172.16.20.116:8080/stream.mjpg"
OUTPUT_DIR = "capturas_pc"
FILE_PREFIX = "pi_run"
OUTPUT_FPS = 20.0


def make_paths(output_dir, prefix):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    base = Path(output_dir) / f"{prefix}_{timestamp}"
    return {
        "video": base.with_suffix(".mp4"),
        "csv": base.with_suffix(".timestamps.csv"),
        "json": base.with_suffix(".json"),
    }


def main():
    output_dir = Path(OUTPUT_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = make_paths(output_dir, FILE_PREFIX)

    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir el stream: {STREAM_URL}")

    ret, frame = cap.read()
    if not ret:
        cap.release()
        raise RuntimeError("No se pudo leer el primer frame del stream.")

    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(
        str(paths["video"]),
        cv2.VideoWriter_fourcc(*"mp4v"),
        OUTPUT_FPS,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError("No se pudo crear el archivo de salida.")

    metadata = {
        "stream_url": STREAM_URL,
        "video_path": str(paths["video"]),
        "timestamps_path": str(paths["csv"]),
        "started_at_epoch": time.time(),
        "started_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "width": width,
        "height": height,
        "fps": OUTPUT_FPS,
        "markers": [],
    }

    print(f"[REC] Grabando stream desde {STREAM_URL}")
    print(f"[REC] Video: {paths['video']}")
    print("[REC] Teclas: 'm' marca error, 'q' termina.")

    frame_index = 0

    with open(paths["csv"], "w", newline="", encoding="utf-8") as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame_index", "elapsed_seconds", "epoch_seconds", "marker"])

        try:
            while True:
                if frame_index > 0:
                    ret, frame = cap.read()
                    if not ret:
                        print("[WARN] Stream interrumpido. Terminando.")
                        break

                now = time.time()
                elapsed = now - metadata["started_at_epoch"]
                writer.write(frame)
                csv_writer.writerow([frame_index, f"{elapsed:.6f}", f"{now:.6f}", ""])

                preview = frame.copy()
                cv2.putText(
                    preview,
                    f"REC {elapsed:7.2f}s frame {frame_index}",
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Raspberry Stream Recorder", preview)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("[REC] Grabacion detenida por usuario.")
                    break
                if key == ord("m"):
                    marker = {
                        "frame_index": frame_index,
                        "elapsed_seconds": round(elapsed, 6),
                        "epoch_seconds": round(now, 6),
                        "label": "manual_error",
                    }
                    metadata["markers"].append(marker)
                    print(f"[MARK] {marker}")

                frame_index += 1
        finally:
            metadata["ended_at_epoch"] = time.time()
            metadata["ended_at_local"] = time.strftime("%Y-%m-%d %H:%M:%S")
            metadata["frames_written"] = frame_index
            metadata["duration_seconds"] = round(
                metadata["ended_at_epoch"] - metadata["started_at_epoch"], 3
            )

            cap.release()
            writer.release()
            cv2.destroyAllWindows()

    with open(paths["json"], "w", encoding="utf-8") as json_file:
        json.dump(metadata, json_file, indent=2)

    print(f"[REC] Metadatos: {paths['json']}")


if __name__ == "__main__":
    main()
