"""Control remoto temporal del robot mediante W, A, S y D.

Este módulo envía el protocolo previsto para el firmware de movimiento:
    <velocidad,angulo_logico>\n
La velocidad va de -255 (reversa) a 255 (avance). El ángulo lógico 92 es
recto; el firmware se encarga de convertirlo al centro físico de 92 grados.

El main.ino actual todavía es una prueba del servo y no interpreta este
protocolo. Por eso se puede usar --dry-run para probar el teclado mientras se
implementa el firmware de movimiento.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass


try:
    import serial
except ImportError:  # Permite probar el teclado sin pyserial instalado.
    serial = None


SERIAL_BAUDRATE = 115200
COMMAND_PERIOD_SECONDS = 0.05
KEY_TIMEOUT_SECONDS = 0.30
ARDUINO_STARTUP_DELAY_SECONDS = 2.0
MAX_SPEED = 120
STEERING_CENTER = 92
# El servo está montado invertido respecto a la orientación lógica.
# El par debe tener una desviación comparable a ambos lados del centro. El
# valor anterior 93 apenas movía el servo 1° y no producía giro apreciable.
STEERING_LEFT = 105
STEERING_RIGHT = 79


@dataclass
class KeyState:
    up: float = 0.0
    down: float = 0.0
    left: float = 0.0
    right: float = 0.0

    def active(self, timestamp: float, key_time: float) -> bool:
        return key_time > 0 and timestamp - key_time <= KEY_TIMEOUT_SECONDS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def read_key_bytes() -> bytes:
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.buffer.read(1) if readable else b""


def update_key_state(buffer: bytes, state: KeyState, timestamp: float) -> tuple[bytes, bool]:
    """Procesa teclas WASD y secuencias ANSI de flechas."""
    should_quit = False

    while buffer:
        if buffer.startswith(b"\x1b[A"):
            state.up = timestamp
            state.down = 0.0
            buffer = buffer[3:]
        elif buffer.startswith(b"\x1b[B"):
            state.down = timestamp
            state.up = 0.0
            buffer = buffer[3:]
        elif buffer.startswith(b"\x1b[C"):
            state.right = timestamp
            state.left = 0.0
            buffer = buffer[3:]
        elif buffer.startswith(b"\x1b[D"):
            state.left = timestamp
            state.right = 0.0
            buffer = buffer[3:]
        elif buffer[0] in (ord("q"), ord("Q"), 0x03):
            should_quit = True
            buffer = buffer[1:]
        elif buffer[0] in (ord("w"), ord("W")):
            state.up = timestamp
            state.down = 0.0
            buffer = buffer[1:]
        elif buffer[0] in (ord("s"), ord("S")):
            state.down = timestamp
            state.up = 0.0
            buffer = buffer[1:]
        elif buffer[0] in (ord("a"), ord("A")):
            state.left = timestamp
            state.right = 0.0
            buffer = buffer[1:]
        elif buffer[0] in (ord("d"), ord("D")):
            state.right = timestamp
            state.left = 0.0
            buffer = buffer[1:]
        elif buffer[0] == 0x1B:
            # Espera a que llegue la secuencia completa de una flecha.
            break
        else:
            buffer = buffer[1:]

    return buffer, should_quit


def calculate_command(state: KeyState, timestamp: float) -> tuple[int, int]:
    """Calcula velocidad y dirección respetando comandos opuestos."""
    up = state.active(timestamp, state.up)
    down = state.active(timestamp, state.down)
    left = state.active(timestamp, state.left)
    right = state.active(timestamp, state.right)

    if up and down:
        speed = 0
    elif up:
        speed = MAX_SPEED
    elif down:
        speed = -MAX_SPEED
    else:
        speed = 0

    if left and right:
        angle = STEERING_CENTER
    elif left:
        angle = STEERING_LEFT
    elif right:
        angle = STEERING_RIGHT
    else:
        angle = STEERING_CENTER

    return speed, angle


def send_command(connection, speed: int, angle: int, dry_run: bool) -> None:
    command = f"<{speed},{angle}>\n"
    if dry_run:
        print(f"TX {command.rstrip()}", flush=True)
    else:
        connection.write(command.encode("ascii"))


def run() -> None:
    args = parse_args()
    connection = None

    if not args.dry_run:
        if serial is None:
            raise SystemExit("Falta pyserial. Instala con: python3 -m pip install -r ../requirements.txt")
        connection = serial.Serial(args.port, SERIAL_BAUDRATE, timeout=0)
        # Abrir el puerto suele reiniciar el Arduino; no perder el primer
        # comando mientras termina el bootloader y arranca el firmware.
        time.sleep(ARDUINO_STARTUP_DELAY_SECONDS)
        connection.reset_input_buffer()

    old_settings = termios.tcgetattr(sys.stdin)
    key_buffer = b""
    state = KeyState()
    last_command = None

    print("Control remoto listo: W/A/S/D para mover, Q o Ctrl+C para salir")
    print("W/S = avance/reversa; A/D = dirección")
    print("Las combinaciones diagonales están permitidas")

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            timestamp = time.monotonic()
            key_buffer += read_key_bytes()
            key_buffer, should_quit = update_key_state(key_buffer, state, timestamp)
            if should_quit:
                break

            speed, angle = calculate_command(state, timestamp)
            send_command(connection, speed, angle, args.dry_run)
            current_command = (speed, angle)
            if not args.dry_run and current_command != last_command:
                print(f"TX <{speed},{angle}>", flush=True)
            last_command = current_command
            time.sleep(COMMAND_PERIOD_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        # La última orden siempre es detener el motor y centrar la dirección.
        if connection is not None:
            send_command(connection, 0, STEERING_CENTER, False)
            connection.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nControl remoto detenido")


if __name__ == "__main__":
    run()
