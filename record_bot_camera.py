import argparse
import csv
import json
import time
from pathlib import Path

import cv2


def build_parser():
    parser = argparse.ArgumentParser(
        description="Graba video desde la camara del bot para analisis y calibracion."
    )
    parser.add_argument(
        "--device",
        default="0",
        help="Indice o ruta de la camara. Ej: 0 o /dev/video0",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=320,
        help="Ancho de captura. Usa 320 para grabar igual que MainCode-2.py.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=240,
        help="Alto de captura. Usa 240 para grabar igual que MainCode-2.py.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="FPS objetivo del archivo de salida.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Duracion maxima en segundos. 0 = hasta detener manualmente.",
    )
    parser.add_argument(
        "--output-dir",
        default="capturas_bot",
        help="Carpeta donde se guardaran video y metadatos.",
    )
    parser.add_argument(
        "--prefix",
        default="lap",
        help="Prefijo del nombre del archivo.",
    )
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="Codec FourCC. Por defecto mp4v para generar mp4 facilmente revisable.",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="No muestra la ventana de vista previa.",
    )
    return parser


def normalize_device(device_value):
    if isinstance(device_value, str) and device_value.isdigit():
        return int(device_value)
    return device_value


def make_output_paths(output_dir, prefix):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    base_name = f"{prefix}_{timestamp}"
    base_path = Path(output_dir) / base_name
    return {
        "video": base_path.with_suffix(".mp4"),
        "csv": base_path.with_suffix(".timestamps.csv"),
        "json": base_path.with_suffix(".json"),
    }


def main():
    args = build_parser().parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = make_output_paths(output_dir, args.prefix)

    device = normalize_device(args.device)
    cap = cv2.VideoCapture(device)

    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la camara/fuente: {device}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or args.fps

    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    writer = cv2.VideoWriter(
        str(paths["video"]),
        fourcc,
        args.fps,
        (actual_width, actual_height),
    )

    if not writer.isOpened():
        cap.release()
        raise RuntimeError(
            f"No se pudo crear el archivo de video con codec '{args.codec}'."
        )

    metadata = {
        "device": str(device),
        "requested_width": args.width,
        "requested_height": args.height,
        "requested_fps": args.fps,
        "actual_width": actual_width,
        "actual_height": actual_height,
        "actual_camera_fps": actual_fps,
        "output_video": str(paths["video"]),
        "timestamps_csv": str(paths["csv"]),
        "started_at_epoch": time.time(),
        "started_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    print("[REC] Grabacion iniciada")
    print(f"[REC] Video: {paths['video']}")
    print(f"[REC] Timestamps: {paths['csv']}")
    print("[REC] Controles: 'q' para terminar, 'm' para marcar un evento")

    start_time = time.time()
    frame_index = 0
    dropped_frames = 0
    markers = []

    with open(paths["csv"], "w", newline="") as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "frame_index",
                "elapsed_seconds",
                "epoch_seconds",
                "capture_pos_msec",
            ]
        )

        try:
            while True:
                ret, frame = cap.read()
                now = time.time()

                if not ret:
                    dropped_frames += 1
                    print(f"[WARN] Frame perdido. Total perdidos: {dropped_frames}")
                    if dropped_frames >= 10:
                        print("[WARN] Demasiados frames perdidos. Terminando grabacion.")
                        break
                    continue

                dropped_frames = 0
                elapsed = now - start_time

                writer.write(frame)
                csv_writer.writerow(
                    [
                        frame_index,
                        f"{elapsed:.6f}",
                        f"{now:.6f}",
                        f"{cap.get(cv2.CAP_PROP_POS_MSEC):.3f}",
                    ]
                )
                frame_index += 1

                if not args.no_preview:
                    preview = frame.copy()
                    cv2.putText(
                        preview,
                        f"REC {elapsed:7.2f}s  frame {frame_index}",
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow("Bot Camera Recorder", preview)
                    key = cv2.waitKey(1) & 0xFF
                else:
                    key = -1

                if key == ord("q"):
                    print("[REC] Grabacion detenida por usuario.")
                    break
                if key == ord("m"):
                    marker = {
                        "frame_index": frame_index - 1,
                        "elapsed_seconds": round(elapsed, 6),
                        "epoch_seconds": round(now, 6),
                    }
                    markers.append(marker)
                    print(f"[MARK] Evento marcado: {marker}")

                if args.duration > 0 and elapsed >= args.duration:
                    print("[REC] Duracion objetivo alcanzada.")
                    break
        finally:
            metadata["ended_at_epoch"] = time.time()
            metadata["ended_at_local"] = time.strftime("%Y-%m-%d %H:%M:%S")
            metadata["duration_seconds"] = round(
                metadata["ended_at_epoch"] - metadata["started_at_epoch"], 3
            )
            metadata["frames_written"] = frame_index
            metadata["dropped_frames"] = dropped_frames
            metadata["markers"] = markers

            cap.release()
            writer.release()
            cv2.destroyAllWindows()

    with open(paths["json"], "w", encoding="utf-8") as json_file:
        json.dump(metadata, json_file, indent=2)

    print(f"[REC] Metadatos: {paths['json']}")
    print(
        f"[REC] Finalizado. Frames escritos: {frame_index}. "
        f"Duracion: {metadata['duration_seconds']} s"
    )


if __name__ == "__main__":
    main()
