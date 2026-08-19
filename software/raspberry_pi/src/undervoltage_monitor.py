"""Monitor de alimentación para Raspberry Pi.

Consulta el registro de estado de firmware con ``vcgencmd get_throttled``.
No controla el Arduino, el servo ni el motor.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from datetime import datetime, timezone


FLAGS = {
    0x1: "under_voltage_now",
    0x2: "frequency_capped_now",
    0x4: "throttled_now",
    0x10000: "under_voltage_occurred",
    0x20000: "frequency_capped_occurred",
    0x40000: "throttled_occurred",
}


def read_throttled() -> tuple[int, list[str]]:
    if shutil.which("vcgencmd") is None:
        raise RuntimeError("No se encontró vcgencmd; ejecuta este programa en Raspberry Pi OS.")
    result = subprocess.run(
        ["vcgencmd", "get_throttled"],
        check=True,
        capture_output=True,
        text=True,
    )
    raw = result.stdout.strip().split("=", 1)[-1]
    value = int(raw, 16)
    return value, [name for bit, name in FLAGS.items() if value & bit]


def snapshot() -> dict[str, object]:
    value, flags = read_throttled()
    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "throttled_hex": f"0x{value:08x}",
        "undervoltage_now": "under_voltage_now" in flags,
        "undervoltage_occurred": "under_voltage_occurred" in flags,
        "flags": flags,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Segundos entre lecturas (por defecto: 1).")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Duración en segundos; 0 significa hasta Ctrl+C.")
    parser.add_argument("--once", action="store_true", help="Realiza una sola lectura.")
    parser.add_argument("--json", action="store_true", help="Imprime cada lectura como JSON.")
    args = parser.parse_args()
    if args.interval <= 0 or args.duration < 0:
        parser.error("--interval debe ser positivo y --duration no puede ser negativa.")
    return args


def main() -> None:
    args = parse_args()
    started = time.monotonic()
    try:
        while True:
            try:
                data = snapshot()
            except (OSError, ValueError, subprocess.CalledProcessError, RuntimeError) as error:
                raise SystemExit(f"No se pudo leer el estado de alimentación: {error}") from error
            if args.json:
                print(json.dumps(data, ensure_ascii=False), flush=True)
            else:
                status = "ALERTA" if data["undervoltage_now"] else "OK"
                history = ", ".join(data["flags"]) if data["flags"] else "sin flags"
                print(f"{data['timestamp_utc']} | {status} | {data['throttled_hex']} | {history}", flush=True)
            if args.once or (args.duration and time.monotonic() - started >= args.duration):
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nMonitor detenido.")


if __name__ == "__main__":
    main()
