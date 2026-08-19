"""Prueba segura de dirección para calibrar ángulo de rueda y servo.

El robot debe estar elevado: este programa siempre transmite velocidad cero.
Para ejecutar una orden física se requiere --execute; sin esa opción solo
muestra el comando serial que enviaría.
"""

from __future__ import annotations

import argparse
import glob
import json
import time
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MAPPING_FILE = PROJECT_ROOT / "config" / "steering_wheel_calibration.json"
SERIAL_BAUDRATE = 115200
ARDUINO_STARTUP_DELAY_SECONDS = 2.0
# Límites mecánicos confirmados para esta prueba física.
LOGICAL_SERVO_MIN_DEG = 50.0
LOGICAL_SERVO_MAX_DEG = 140.0
LOGICAL_SERVO_CENTER_DEG = 92.0
MAX_WHEEL_ANGLE_DEG = 30.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=False)
    target.add_argument("--wheel-angle-deg", type=float,
                        help="Ángulo real objetivo de la rueda; requiere una tabla medida.")
    target.add_argument("--servo-command-deg", type=float,
                        help="Comando lógico directo para realizar una medición física.")
    parser.add_argument("--mapping-file", type=Path, default=DEFAULT_MAPPING_FILE,
                        help=f"Tabla medida (por defecto: {DEFAULT_MAPPING_FILE})")
    parser.add_argument("--port", help="Puerto serial del Arduino, por ejemplo /dev/ttyACM0.")
    parser.add_argument("--hold-seconds", type=float, default=5.0,
                        help="Tiempo para medir antes de centrar (por defecto: 5).")
    parser.add_argument("--execute", action="store_true",
                        help="Envía la orden al Arduino. Sin esta opción solo hace dry-run.")
    parser.add_argument("--interactive", action="store_true",
                        help="Solicita varios comandos de servo y guarda el ángulo medido de cada uno.")
    parser.add_argument("--probe-limits", action="store_true",
                        help="Prueba los límites configurados paso a paso y espera confirmación manual.")
    parser.add_argument("--probe-direction", choices=("right", "left"),
                        help="Sentido único para --probe-limits; no cambia al otro lado.")
    parser.add_argument("--on-ground", action="store_true",
                        help="Indica que las ruedas estarán apoyadas; el motor sigue detenido.")
    args = parser.parse_args()
    if args.probe_limits and (args.interactive or args.wheel_angle_deg is not None or args.servo_command_deg is not None):
        parser.error("--probe-limits no se combina con otro modo de prueba.")
    if args.probe_direction and not args.probe_limits:
        parser.error("--probe-direction requiere --probe-limits.")
    if args.interactive and (args.wheel_angle_deg is not None or args.servo_command_deg is not None):
        parser.error("--interactive no se combina con --wheel-angle-deg ni --servo-command-deg.")
    if not args.probe_limits and not args.interactive and args.wheel_angle_deg is None and args.servo_command_deg is None:
        parser.error("Indica un objetivo, usa --interactive o usa --probe-limits.")
    if not 0 <= args.hold_seconds <= 30:
        parser.error("--hold-seconds debe estar entre 0 y 30.")
    if args.execute and not args.port:
        parser.error("--execute requiere --port.")
    return args


def load_pairs(mapping_file: Path) -> list[tuple[float, float]]:
    """Carga pares (ángulo real de rueda, comando lógico de servo)."""
    try:
        payload = json.loads(mapping_file.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"No existe la tabla de calibración: {mapping_file}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"JSON inválido en la tabla: {mapping_file}") from error

    pairs: list[tuple[float, float]] = []
    for entry in payload.get("pairs", []):
        try:
            wheel = float(entry["wheel_angle_deg"])
            servo = float(entry["servo_command_deg"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("Cada par debe incluir wheel_angle_deg y servo_command_deg.") from error
        pairs.append((wheel, servo))

    if not pairs:
        raise ValueError("La tabla no contiene pares medidos.")
    if len({wheel for wheel, _ in pairs}) != len(pairs):
        raise ValueError("La tabla contiene ángulos de rueda repetidos.")
    return sorted(pairs)


def servo_for_wheel_angle(wheel_angle_deg: float, pairs: list[tuple[float, float]]) -> float:
    """Interpolación lineal exclusivamente entre dos mediciones contiguas."""
    if abs(wheel_angle_deg) > MAX_WHEEL_ANGLE_DEG:
        raise ValueError(f"El ángulo de rueda debe estar entre ±{MAX_WHEEL_ANGLE_DEG:.0f}°.")
    for wheel, servo in pairs:
        if wheel_angle_deg == wheel:
            return servo
    for (left_wheel, left_servo), (right_wheel, right_servo) in zip(pairs, pairs[1:]):
        if left_wheel < wheel_angle_deg < right_wheel:
            fraction = (wheel_angle_deg - left_wheel) / (right_wheel - left_wheel)
            return left_servo + fraction * (right_servo - left_servo)
    raise ValueError("El ángulo solicitado queda fuera del rango medido; no se extrapola.")


def validate_servo_command(servo_command_deg: float) -> int:
    command = round(servo_command_deg)
    if not LOGICAL_SERVO_MIN_DEG <= command <= LOGICAL_SERVO_MAX_DEG:
        raise ValueError(
            f"El comando lógico debe estar entre {LOGICAL_SERVO_MIN_DEG:.0f}° y "
            f"{LOGICAL_SERVO_MAX_DEG:.0f}° según el firmware actual."
        )
    return command


def send_command(connection, servo_command_deg: int, dry_run: bool) -> None:
    payload = f"<0,{servo_command_deg}>\n"
    if dry_run:
        print(f"DRY-RUN TX {payload.rstrip()}")
        return
    connection.write(payload.encode("ascii"))
    connection.flush()
    reply = connection.readline().decode("ascii", errors="replace").strip()
    print(f"TX {payload.rstrip()}" + (f" | RX {reply}" if reply else ""))


def detected_serial_ports() -> str:
    """Lista los nombres de dispositivos serial habituales en Raspberry Pi."""
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ", ".join(ports) if ports else "ninguno"


def open_connection(port: str | None, execute: bool):
    if not execute:
        return None
    if serial is None:
        raise SystemExit("Falta pyserial. Instala software/raspberry_pi/requirements.txt")
    try:
        connection = serial.Serial(port, SERIAL_BAUDRATE, timeout=0.5)
        # Abrir el puerto normalmente reinicia el Arduino. Esperar el
        # bootloader evita perder el primer comando de dirección.
        time.sleep(ARDUINO_STARTUP_DELAY_SECONDS)
        connection.reset_input_buffer()
        return connection
    except serial.SerialException as error:
        raise SystemExit(
            f"No se pudo abrir {port}: {error}. Puertos detectados: {detected_serial_ports()}. "
            "Conecta el Arduino por USB y vuelve a ejecutar con --port correcto."
        ) from error


def save_measured_pair(mapping_file: Path, wheel_angle_deg: float, servo_command_deg: int) -> None:
    """Añade o actualiza un par medido sin borrar metadatos del archivo."""
    try:
        payload = json.loads(mapping_file.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        payload = {"units": {"angle": "degrees"}, "pairs": []}
    pairs = payload.setdefault("pairs", [])
    pairs[:] = [entry for entry in pairs
                if float(entry.get("wheel_angle_deg", 10_000)) != wheel_angle_deg]
    pairs.append({
        "wheel_angle_deg": wheel_angle_deg,
        "servo_command_deg": servo_command_deg,
        "source": "Measured with steering_angle_test.py",
    })
    pairs.sort(key=lambda entry: float(entry["wheel_angle_deg"]))
    mapping_file.parent.mkdir(parents=True, exist_ok=True)
    mapping_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def run_interactive(args: argparse.Namespace) -> None:
    surface = "suelo, motor detenido" if args.on_ground else "elevado, ruedas libres"
    print(f"Calibración interactiva. Robot sobre {surface}.")
    print(
        f"Introduce un comando lógico de servo ({LOGICAL_SERVO_MIN_DEG:.0f}.."
        f"{LOGICAL_SERVO_MAX_DEG:.0f}) o 'q' para terminar."
    )
    connection = open_connection(args.port, args.execute)
    try:
        while True:
            raw_command = input("Servo lógico> ").strip()
            if raw_command.lower() in {"q", "quit", "salir"}:
                break
            try:
                command = validate_servo_command(float(raw_command))
            except ValueError as error:
                print(f"Error: {error}")
                continue
            send_command(connection, command, dry_run=not args.execute)
            print("La posición queda mantenida sin límite de tiempo; mide la rueda y pulsa Enter cuando termines.")
            measured = input("Ángulo real medido en la rueda (Enter sin valor para no guardar)> ").strip()
            if not measured:
                continue
            try:
                wheel_angle = float(measured.replace(",", "."))
                if abs(wheel_angle) > MAX_WHEEL_ANGLE_DEG:
                    raise ValueError(f"debe estar entre ±{MAX_WHEEL_ANGLE_DEG:.0f}°")
            except ValueError as error:
                print(f"Medición no guardada: {error}")
                continue
            save_measured_pair(args.mapping_file, wheel_angle, command)
            print(f"Guardado: rueda {wheel_angle:+.1f}° -> servo {command}° en {args.mapping_file}")
    except (KeyboardInterrupt, EOFError):
        print("\nInterrumpido: centrando dirección.")
    finally:
        if connection is not None:
            try:
                send_command(connection, int(LOGICAL_SERVO_CENTER_DEG), dry_run=False)
            finally:
                connection.close()


def run_limit_probe(args: argparse.Namespace) -> None:
    """Recorre los límites aceptados, sin intentar superar el firmware."""
    print(f"Rango lógico configurado: {LOGICAL_SERVO_MIN_DEG:.0f}..{LOGICAL_SERVO_MAX_DEG:.0f}°")
    surface = "suelo" if args.on_ground else "elevado"
    print(f"Robot sobre {surface}, motor detenido.")
    print("Esto no determina el tope mecánico absoluto; detente si hay ruido, vibración o esfuerzo.")
    direction = args.probe_direction
    if direction is None:
        direction = input("Sentido único (right/left)> ").strip().lower()
        if direction not in {"right", "left"}:
            raise SystemExit("El sentido debe ser right o left.")
    connection = open_connection(args.port, args.execute)
    try:
        label = "derecha" if direction == "right" else "izquierda"
        values = list(range(90, int(LOGICAL_SERVO_MIN_DEG) - 1, -5)
                      if direction == "right"
                      else range(90, int(LOGICAL_SERVO_MAX_DEG) + 1, 5))
        endpoint = int(LOGICAL_SERVO_MIN_DEG if direction == "right" else LOGICAL_SERVO_MAX_DEG)
        if values[-1] != endpoint:
            values.append(endpoint)
        print(f"\nPrueba únicamente hacia {label}; no cambiará de sentido.")
        send_command(connection, 90, dry_run=not args.execute)
        steps = values[1:]
        for index, value in enumerate(steps):
            answer = input(f"Enter para avanzar a {value}°; 's' detiene > ").strip().lower()
            if answer in {"s", "stop", "q", "salir"}:
                break
            send_command(connection, value, dry_run=not args.execute)
            if index == len(steps) - 1:
                print("Límite lógico configurado alcanzado; mide el giro.")
                input("Pulsa Enter cuando termines para volver al centro > ")
            else:
                print("Paso aplicado; observa el giro y pulsa Enter para avanzar otros 5°.")
    except (KeyboardInterrupt, EOFError):
        print("\nInterrumpido: centrando dirección.")
    finally:
        if connection is not None:
            try:
                send_command(connection, int(LOGICAL_SERVO_CENTER_DEG), dry_run=False)
            finally:
                connection.close()


def main() -> None:
    args = parse_args()
    if args.probe_limits:
        run_limit_probe(args)
        return
    if args.interactive:
        run_interactive(args)
        return
    try:
        if args.wheel_angle_deg is not None:
            pairs = load_pairs(args.mapping_file)
            servo_command = servo_for_wheel_angle(args.wheel_angle_deg, pairs)
            print(f"Rueda objetivo: {args.wheel_angle_deg:+.1f}° -> servo lógico: {servo_command:.2f}°")
        else:
            servo_command = args.servo_command_deg
            print("Modo de medición directa: mide el ángulo real de la rueda y añádelo a la tabla.")
        command = validate_servo_command(servo_command)
    except ValueError as error:
        raise SystemExit(f"Error: {error}") from error

    connection = None
    try:
        connection = open_connection(args.port, args.execute)
        send_command(connection, command, dry_run=not args.execute)
        if args.execute and args.hold_seconds:
            print(f"Manteniendo {args.hold_seconds:.1f} s; robot elevado y ruedas libres.")
            time.sleep(args.hold_seconds)
    except KeyboardInterrupt:
        print("\nInterrumpido: centrando dirección.")
    finally:
        if connection is not None:
            try:
                send_command(connection, int(LOGICAL_SERVO_CENTER_DEG), dry_run=False)
            finally:
                connection.close()


if __name__ == "__main__":
    main()
