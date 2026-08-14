import cv2
import numpy as np
import serial
import time
import threading

# Serial parameters used by the robot controller board.
# The Python side sends `<speed,angle>` packets and receives simple button events.
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class WROPrimitivoBlindado:
    def __init__(self):
        # Try to open the serial link first. The rest of the program can still run
        # without the Arduino so camera logic can be debugged on a laptop.
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            self.serial_enabled = True
            # Give the board time to reset after the serial port opens.
            time.sleep(2)
        except Exception as e:
            print(f"[ERROR] Sin Arduino: {e}")
            self.serial_enabled = False

        # Global execution flags shared by the main loop and worker threads.
        self.running = True
        self.estado_general = "ESPERA"
        self.boton_presionado = False
        self.start_time = None

        # Control variables
        self.current_speed = 0
        self.current_angle = 86

        # Lap counting and turn direction
        self.SENTIDO_GIRO = "AUTO"
        self.vueltas_completadas = 0
        self.curvas_superadas = 0
        self.en_curva = False
        self.tiempo_ultima_curva = time.time()

        # Reserved metric left available for future tuning or debugging.
        self.score_muro_frontal = 0

        # Display control (False for fully autonomous competition)
        self.VER_PANTALLAS = True

        print("[SISTEMA] Modo 1 Primitivo Blindado Inicializado.")

    def read_serial_data(self):
        # This background loop listens only for asynchronous signals from the Arduino.
        # Right now the relevant event is the external start button.
        while self.running and self.serial_enabled:
            try:
                if self.ser.in_waiting > 0:
                    # Read one line, tolerate malformed bytes, and remove line endings.
                    linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if linea == "BTN:1":
                        self.boton_presionado = True
            except:
                # Ignore transient serial read errors to keep the control loop alive.
                pass
            # Small sleep to avoid busy-waiting and unnecessary CPU usage.
            time.sleep(0.02)

    def process_vision(self):
        # Open the default camera using a small resolution to reduce latency
        # and keep image processing cheap enough for real-time control.
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Zero lag

        # A small kernel is enough to remove isolated noise after thresholding.
        kernel = np.ones((3, 3), np.uint8)

        while self.running:
            ret, frame = cap.read()
            # Skip the iteration if the camera did not deliver a frame.
            if not ret:
                continue

            current_time = time.time()
            # This timer is used to temporarily disable some detections right after launch.
            tiempo_en_carrera = current_time - self.start_time if self.start_time else 0

            # 1. Focus exclusively on the track (filtering out the ceiling)
            # Converting to grayscale and blurring makes thresholding more stable
            # under lighting noise and small texture variations.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # Crop a band of the track (rows 60 to 150 of the original video)
            # The crop removes most irrelevant background and keeps only the useful floor area.
            roi_pista = blur[60:150, :]
            _, binarizada = cv2.threshold(
                roi_pista, 95, 255, cv2.THRESH_BINARY)
            # Morphological opening deletes small white specks that could fake lane edges.
            binarizada = cv2.morphologyEx(binarizada, cv2.MORPH_OPEN, kernel)

            # 2. Separate horizon and scan line within the track
            # Free horizon (far track): upper rows of the crop
            horizonte = binarizada[0:25, :]
            # Driving line (near track): lower row of the crop
            linea_escaneo = binarizada[65, :]

            # 3. Front wall detection (corner detection)
            # The center box acts as a simple "what is directly ahead?" probe.
            caja_central = binarizada[30:70, 120:200]
            ratio_oscuro = np.mean(caja_central == 0)

            # Problem scenario 2 (inner wall at startup):
            # If the run has lasted less than 1.5 seconds, force front-wall detection off.
            # This prevents the side wall from being confused with the end of the straight.
            if self.estado_general == "CARRERA" and tiempo_en_carrera < 1.5:
                es_muro_frontal = False
            else:
                es_muro_frontal = ratio_oscuro > 0.55

            # 4. Side ray scanning
            # Search from the center outward to find the first dark pixels that behave like walls.
            # `-1` means that side was not detected on the current scan line.
            muro_izq = -1
            muro_der = -1
            for x in range(160, -1, -1):
                if linea_escaneo[x] == 0:
                    muro_izq = x
                    break
            for x in range(160, 320):
                if linea_escaneo[x] == 0:
                    muro_der = x
                    break

            # =======================================================
            # 5. Reactive state machine
            # =======================================================
            if self.SENTIDO_GIRO == "AUTO" and self.estado_general == "CARRERA":
                # Infer the race direction from the far view of the track.
                # More visible white area on one side suggests the upcoming open path.
                blancos_izq = np.sum(horizonte[:, :160] == 255)
                blancos_der = np.sum(horizonte[:, 160:] == 255)

                if blancos_der > blancos_izq:
                    self.SENTIDO_GIRO = "DERECHA"
                    print("\n[>>] HORARIO DETECTADO (DERECHA) [>>]\n")
                else:
                    self.SENTIDO_GIRO = "IZQUIERDA"
                    print("\n[<<] ANTIHORARIO DETECTADO (IZQUIERDA) [<<]\n")

            if es_muro_frontal:
                # Seeing a front wall means the robot is likely entering a corner
                # and should switch to a safer, deterministic turning behavior.

                # B) Real lap counting (4 corners = 1 lap)
                if not self.en_curva and (current_time - self.tiempo_ultima_curva > 2):
                    # `en_curva` and the time gate avoid counting the same corner multiple times.
                    self.en_curva = True
                    self.tiempo_ultima_curva = current_time
                    self.curvas_superadas += 1

                    if self.curvas_superadas % 4 == 0:
                        self.vueltas_completadas += 1
                        print(
                            f"\n[OK] VUELTA {self.vueltas_completadas}/3 COMPLETADA \n")

                        if self.vueltas_completadas >= 3:
                            self.estado_general = "RETORNO_A_META"

                # C) Fixed turn angle selection
                # Cornering uses fixed angles instead of proportional control
                # because the robot is already close to the wall and needs a decisive turn.
                if self.SENTIDO_GIRO == "DERECHA":
                    self.current_angle = 70
                elif self.SENTIDO_GIRO == "IZQUIERDA":
                    self.current_angle = 104
                else:
                    self.current_angle = 86

                self.current_speed = 180  # Safe speed in corners

            else:
                # D) Straight-line navigation (proportional control with dead zone)
                # When there is no wall ahead, prioritize speed and small steering corrections.
                self.current_speed = 250

                # Estimate the center of the track
                # If one wall is missing, estimate the track center from the wall that is still visible.
                if muro_izq != -1 and muro_der != -1:
                    centro_pista = (muro_izq + muro_der) // 2
                elif muro_izq != -1:
                    centro_pista = muro_izq + 80
                elif muro_der != -1:
                    centro_pista = muro_der - 80
                else:
                    centro_pista = 160

                # Positive error means the estimated track center is to the left of the camera center,
                # so the steering must compensate accordingly.
                error = 160 - centro_pista

                # Large dead zone (22 pixels): if it is near the center, keep it straight (86) and avoid jitter.
                if abs(error) < 22:
                    self.current_angle = 86
                else:
                    # If it drifts away, apply a soft proportional correction based on the error.
                    # Multiply by 0.15 so the adjustment is gradual and not abrupt.
                    correccion_suave = int(error * 0.15)
                    angulo_prop = 86 + correccion_suave

                    # Clamp the straight-line steering to a range that avoids zig-zagging (74 to 98)
                    self.current_angle = max(74, min(98, angulo_prop))

                # Corner release lock
                # After enough time without detecting a front wall, allow the next corner to be counted.
                if current_time - self.tiempo_ultima_curva > 3:
                    self.en_curva = False

            if self.VER_PANTALLAS:
                # The processed binary image is much easier to tune than the raw camera feed.
                cv2.imshow("Vision Procesada", binarizada)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False

            # Limit the vision loop frequency slightly so it cooperates with the other thread.
            time.sleep(0.01)
        # Release the camera cleanly when the robot stops.
        cap.release()

    def main_loop(self):
        # Initial steering sweep used as a visible hardware check before the match starts.
        if self.serial_enabled:
            self.ser.write(b"<0,120>\n")
            time.sleep(0.4)
            self.ser.write(b"<0,60>\n")
            time.sleep(0.4)
            self.ser.write(b"<0,86>\n")
            self.ser.write(b"<0,86>\n")

        # Run serial input and vision in parallel so the main loop can focus on high-level states.
        threading.Thread(target=self.read_serial_data, daemon=True).start()
        threading.Thread(target=self.process_vision, daemon=True).start()

        print("\n[SISTEMA] ESPERANDO BOTÓN PARA INICIAR...\n")

        while self.running:
            if self.estado_general == "ESPERA":
                if self.boton_presionado:
                    print("[START] Botón detectado.")
                    self.start_time = time.time()

                    # Problem scenario 1: starting already facing the front wall
                    # Check whether the camera already sees the black wall before starting
                    if self.en_curva == True:  # Means a previous visual blockage was detected
                        print("[DESPEGUE] Muro al frente. Retrocediendo...")
                        # Reverse briefly to create room before handing control to the race logic.
                        if self.serial_enabled:
                            self.ser.write(b"<-160,86>\n")
                        time.sleep(1)  # Half-second reverse
                        if self.serial_enabled:
                            self.ser.write(b"<0,86>\n")
                        time.sleep(0.1)

                    # Drop any pending serial output so the race begins with fresh commands only.
                    if self.serial_enabled:
                        self.ser.reset_output_buffer()
                    self.estado_general = "CARRERA"
                    self.start_time = time.time()
                else:
                    time.sleep(0.05)

            elif self.estado_general == "CARRERA":
                if self.serial_enabled:
                    # The Arduino expects commands in the `<speed,angle>` text format.
                    paquete = f"<{self.current_speed},{self.current_angle}>\n"
                    self.ser.write(paquete.encode())
                # This pacing keeps motor commands frequent without flooding the serial port.
                time.sleep(0.04)

            elif self.estado_general == "RETORNO_A_META":
                # Cross the finish line and brake with a counter-thrust
                if self.serial_enabled:
                    self.ser.write(b"<200,86>\n")
                    time.sleep(0.4)
                    self.ser.write(b"<-200,86>\n")
                    time.sleep(0.15)
                    self.ser.write(b"<0,86>\n")
                print("[FIN] Vuelta 3 completada con éxito.")
                self.running = False

        # Close the serial port only once the whole state machine has finished.
        if self.serial_enabled:
            self.ser.close()


if __name__ == "__main__":
    # Script entry point: instantiate the controller and hand over execution to its main loop.
    bot = WROPrimitivoBlindado()
    bot.main_loop()
