import serial
import time

# --- HARDWARE CONFIGURATION ---
# These values define how the choreography script talks to the Arduino controller.
SERIAL_PORT = "/dev/ttyUSB0" 
BAUDRATE = 115200

# =====================================================================
# 📍 MODE 2 CHOREOGRAPHY (Adjust this on the track)
# =====================================================================
# Strict format: (Speed, Angle, Time_in_seconds, "Comment")
# 
# Angle guide:
# - 86  : Completely straight
# - 60  : Hard turn to the RIGHT
# - 115 : Hard turn to the LEFT
# =====================================================================

RUTINA_MANUAL = [
    # Parking exit
    # Each tuple is executed in order with no sensor feedback.
    # This means timing and steering values are meant to be tuned physically on the track.
    (-140, 86,  0.58,  "Retroceso estacionamiento"),
    (0,   60,  0.5,  "Pausa medicion"),
    (140, 60,  0.56,  "Giro Hacia afuera Corto(Derecha)"),
    (0,   120,  0.5,  "Pausa medicion"),
    (-140, 120,  0.5,  "Retroceso Acomodo(Derecha)"),
    (0,   60,  0.5,  "Pausa medicion"),
    (140, 60,  0.8,  "Giro Hacia afuera Largo(Derecha)"),
    (140, 120,  1,  "Endereso a recta(Derecha)"),
    #(250, 86,  0.6,  "1. Initial straight acceleration (test)"),
    # First lap
    (250, 86,  0.6,  "1. Aceleración en recta inicial"),
    (180, 60,  1.8,  "2. Toma de la Curva 1 (Derecha)"),
    (250, 86,  2.8,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 2 (Derecha)"),
    (250, 86,  2.5,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 3 (Derecha)"),
    (250, 86,  2.2,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 4 (Derecha)"),
    (250, 86,  0.8,  "3. Recta corta"),
    # Second lap
    (250, 86,  1.2,  "1. Aceleración en recta inicial"),
    (180, 60,  1.6,  "2. Toma de la Curva 1 (Derecha)"),
    (250, 86,  2.6,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 2 (Derecha)"),
    (250, 86,  2,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 3 (Derecha)"),
    (250, 86,  2.4,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 4 (Derecha)"),
    (250, 86,  0.8,  "3. Recta corta"),
    # Third lap
    (250, 86,  1.2,  "1. Aceleración en recta inicial"),
    (180, 60,  1.6,  "2. Toma de la Curva 1 (Derecha)"),
    (250, 86,  2.6,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 2 (Derecha)"),
    (250, 86,  2.4,  "3. Recta media"),
    (180, 60,  1.62,  "2. Toma de la Curva 3 (Derecha)"),
    (250, 86,  2.4,  "3. Recta media"),
    (180, 60,  1.58,  "2. Toma de la Curva 4 (Derecha)"),
    (250, 86,  1.2,  "3. Recta corta"),
    # Parking entry
    (-140, 120,  0.8,  "Retroceso Entrada estacionamiento(Izquierda)"),
    (0,   86,  0.5,  "Pausa medicion"),
    (-140, 86,  0.4,  "Retroceso Acomodo(Izquierda)"),
    #(0,   60,  0.5,  "Measurement pause"),
    #(-140, 60,  0.56,  "Long inward turn (Right)"),
    #(0,   120,  0.5,  "Measurement pause"),
    #(140, 120,  0.4,  "Short inward turn (Right)"),
    #(0,   60,  0.5,  "Measurement pause"),
    #(-140, 60,  0.8,  "Straighten into parking"),
    # Command archive
    #(150, 115, 0.4,  "4. Evasion maneuver (Left turn)"),
    #(150, 60,  0.4,  "5. Recovery to center (Right turn)"),
    #(250, 86,  1.5,  "6. Long straight to the finish"),
    #(-200,86,  0.2,  "7. Brake counter-thrust (Reverse)"),
    (0,   86,  0.1,  "8. Apagado de motores")
]

# =====================================================================


class WROCoreografia:
    def __init__(self):
        # Open the serial connection if the robot hardware is available.
        # Keeping `serial_enabled` separate lets the file be inspected without the board connected.
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            self.serial_enabled = True
            # Arduino boards often reset when the serial port opens.
            time.sleep(2)
        except Exception as e:
            print(f"[ERROR] Sin Arduino: {e}")
            self.serial_enabled = False

        print("[SISTEMA] MODO 2: COREOGRAFÍA MANUAL CARGADA.")

    def esperar_boton(self):
        """Infinite loop that keeps the robot stopped until the button is pressed."""
        print("\n[READY] Esperando pulsador para iniciar secuencia...\n")
        
        # Straighten the wheels and stop the motors while waiting
        if self.serial_enabled:
            self.ser.write(b"<0,86>\n")

        while True:
            # Poll the serial buffer until the Arduino reports the button press event.
            if self.serial_enabled and self.ser.in_waiting > 0:
                linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if linea == "BTN:1":
                    print("\n[GO] ¡BOTÓN DETECTADO! Lanzando coreografía...\n")
                    # Clear any queued output so the routine starts from a clean command stream.
                    self.ser.reset_output_buffer()
                    break
            # Prevent the waiting loop from consuming a full CPU core.
            time.sleep(0.02)

    def ejecutar_rutina(self):
        """Read the RUTINA_MANUAL list step by step and execute it."""
        
        for paso in RUTINA_MANUAL:
            # Unpack the current choreography instruction into readable local variables.
            velocidad = paso[0]
            angulo = paso[1]
            duracion = paso[2]
            descripcion = paso[3]

            print(f"> Ejecutando: {descripcion} | Vel: {velocidad} | Ang: {angulo} | Tiempo: {duracion}s")

            # 1. Send the command to the motors and servos
            if self.serial_enabled:
                # The Arduino parser expects commands as plain text in `<speed,angle>` format.
                comando = f"<{velocidad},{angulo}>\n"
                self.ser.write(comando.encode('utf-8'))

            # 2. Keep that action active for the specified duration
            # Timing is the core of this mode because there is no closed-loop correction.
            time.sleep(duracion)

        # After the list ends, ensure the robot is stopped
        print("\n[FIN] Coreografía completada. Apagando sistemas.")
        if self.serial_enabled:
            self.ser.write(b"<0,86>\n")
            self.ser.close()

    def run(self):
        # Servo handshake sequence to confirm startup
        # This gives a quick visible confirmation that the steering servo is alive.
        if self.serial_enabled:
            self.ser.write(b"<0,120>\n")
            time.sleep(0.3)
            self.ser.write(b"<0,60>\n")
            time.sleep(0.3)
            self.ser.write(b"<0,86>\n")

        # 1. Wait for the button
        self.esperar_boton()
        
        # 2. Once pressed, start the routine
        self.ejecutar_rutina()


if __name__ == "__main__":
    # Script entry point for the fully manual, time-based choreography mode.
    bot = WROCoreografia()
    bot.run()
