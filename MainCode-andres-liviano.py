import cv2
import numpy as np
import threading
import time

try:
    import serial
except ImportError:
    serial = None


SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200
CAMERA_INDEX = 0
WINDOW_MAIN = "Vista Principal"
WINDOW_GREEN = "Filtro VERDE"
WINDOW_RED = "Filtro ROJO"
WINDOW_BINARY = "Vision Binarizada"
SHOW_DEBUG_WINDOWS = False

DEAD_ZONE_PX = 9
EXIT_CURVE_HOLD_SECONDS = 0.65
CURVE_COMMIT_SECONDS = 0.75
MAX_MOTOR_SPEED = 255
STEER_MIN_ANGLE = 60
STEER_CENTER_ANGLE = 86
STEER_MAX_ANGLE = 120
FRONTAL_TURN_SPEED = 188
FRONTAL_RIGHT_TURN_ANGLE = 69
FRONTAL_LEFT_TURN_ANGLE = 103
CURVE_RECOVERY_SPEED = 188
CENTERED_CRUISE_SPEED = 224
SCAN_ROW_OFFSETS = (-18, -9, 0, 9, 18)
SCAN_ROW_WEIGHTS = (1.0, 1.15, 1.35, 1.15, 1.0)
TRACK_WIDTH_ALPHA = 0.18
CENTER_ALPHA_STRAIGHT = 0.30
CENTER_ALPHA_CURVE = 0.40
FRONTAL_CENTER_HALF_WIDTH = 18
FRONTAL_DARK_RATIO_THRESHOLD = 0.58
FRONTAL_WIDTH_TRIGGER_RATIO = 1.24
FRONT_WALL_SCORE_TRIGGER = 2
TURN_DIRECTION_MIN_DIFF_PIXELS = 180
TURN_DIRECTION_CONFIRM_VOTES = 2
STEERING_RATE_LIMIT_STRAIGHT = 4
STEERING_RATE_LIMIT_CURVE = 6
STEERING_RATE_LIMIT_FRONTAL = 7
FINAL_LAP_MIN_TIME_AFTER_CURVE = 0.55
FINAL_STRAIGHT_MIN_CENTERED_SECONDS = 0.18
CURVE_ENTRY_WIDTH_RATIO = 0.96
CURVE_ENTRY_DARK_RATIO_THRESHOLD = 0.14
CURVE_ENTRY_BIAS_PX = 16
CURVE_APEX_BIAS_PX = 24
CURVE_EXIT_BIAS_PX = 6
CURVE_INNER_WALL_EXTRA_BIAS_PX = 4
CURVE_INSIDE_MARGIN_PX = 46
CURVE_OUTSIDE_MARGIN_PX = 30
CURVE_ENTRY_SPEED = 196
CURVE_ENTRY_MIN_STEER_DELTA = 2
CURVE_COMMIT_MIN_STEER_DELTA = 3
CURVE_EXIT_MIN_STEER_DELTA = 1
INITIAL_DISTANCE_CAPTURE_SECONDS = 1.2
INITIAL_DISTANCE_MIN_SAMPLES = 5
INITIAL_DISTANCE_VALID_RANGE_CM = (4.0, 250.0)
INITIAL_DISTANCE_REFERENCE_MAX_CM = 185.0
FINAL_DISTANCE_TOLERANCE_CM = 1.0
FINAL_DISTANCE_BRAKE_MARGIN_CM = 34.0
FINAL_APPROACH_SPEED = 118
INITIAL_VISUAL_SIGNATURE_MIN_SAMPLES = 8
VISUAL_SIGNATURE_CENTER_TOLERANCE_PX = 22.0
VISUAL_SIGNATURE_WIDTH_TOLERANCE_PX = 24.0
VISUAL_SIGNATURE_WALL_TOLERANCE_PX = 26.0
VISUAL_SIGNATURE_DARK_TOLERANCE = 0.22
STRAIGHT_PROFILE_MIN_SAMPLES = 18
STRAIGHT_PROFILE_MAX_SAMPLES = 120
STRAIGHT_STABLE_HOLD_SECONDS = 0.12
STRAIGHT_PROFILE_DARK_RATIO_MAX = 0.36
FINAL_STRAIGHT_CENTER_TOLERANCE_PX = 7.0
FINAL_LANE_CENTER_TOLERANCE_PX = 8


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=160):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        dt = max(0.015, min(dt, 0.12))
        error = self.setpoint - current_value
        self.integral = float(np.clip(self.integral + (error * dt), -350.0, 350.0))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        derivative = float(np.clip(derivative, -400.0, 400.0))
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


class WROAutonomousCar:
    def __init__(self, serial_port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE):
        self.ser = None
        self.serial_enabled = False
        self.serial_reader_thread = None

        if serial is None:
            print("[WARN] pyserial no esta instalado. Ejecutando sin Arduino.")
        else:
            try:
                self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
                self.serial_enabled = True
                time.sleep(2)
            except Exception as exc:
                print(f"[WARN] No se pudo abrir el puerto serial {serial_port}: {exc}")
                print("[WARN] Ejecutando sin Arduino.")

        self.SENTIDO_GIRO = "AUTO"
        self.MITAD_ANCHO_PISTA_PX = 140
        self.track_half_width_px = float(self.MITAD_ANCHO_PISTA_PX)

        self.vueltas_completadas = 0
        self.curvas_superadas = 0
        self.en_curva = False
        self.ultimo_tiempo_curva = time.time()

        self.pid = PIDController(kp=0.06, ki=0.000, kd=0.20)
        self.running = True
        self.centro_suavizado = 160.0
        self.current_speed = 0
        self.current_angle = STEER_CENTER_ANGLE
        self.estado_general = "ESPERA"
        self.boton_presionado = False
        self.start_time = None

        self.last_non_centered_time = 0.0
        self.curve_commit_until = 0.0
        self.last_curve_direction = None
        self.turn_direction_candidate = None
        self.turn_direction_votes = {"DERECHA": 0, "IZQUIERDA": 0}
        self.front_wall_score = 0
        self.centered_since = None
        self.finish_pending = False
        self.finish_ready_after = 0.0
        self.finish_curve_count_target = None
        self.finish_distance_armed = False
        self.finish_distance_prev = None
        self.scan_row_offsets = SCAN_ROW_OFFSETS
        self.scan_row_weights = SCAN_ROW_WEIGHTS
        self.morph_kernel = np.ones((3, 3), dtype=np.uint8)
        self.distancia_us_filtrada = None
        self.distancia_us_inicio = None
        self.distancia_us_inicio_capturada = False
        self.distancia_us_muestras_arranque = []
        self.initial_visual_signature = None
        self.initial_visual_signature_captured = False
        self.initial_visual_signature_samples = []
        self.straight_profile_samples = []
        self.calibrated_straight_signature = None

        print(f"[SISTEMA] Algoritmo liviano iniciado. Giro ciego: {self.SENTIDO_GIRO}")

    def serial_reader_loop(self):
        while self.running and self.serial_enabled and self.ser is not None:
            try:
                raw_line = self.ser.readline()
            except Exception as exc:
                print(f"[WARN] Error leyendo serial: {exc}")
                self.serial_enabled = False
                break

            if not raw_line:
                continue

            try:
                line = raw_line.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if line == "BTN:1":
                self.boton_presionado = True
                continue

            if not line.startswith("US:"):
                continue

            try:
                distance_cm = float(line.split(":", 1)[1].strip())
            except ValueError:
                continue

            valid_min, valid_max = INITIAL_DISTANCE_VALID_RANGE_CM
            if not (valid_min <= distance_cm <= valid_max):
                continue

            if self.distancia_us_filtrada is None:
                self.distancia_us_filtrada = distance_cm
            else:
                self.distancia_us_filtrada = (0.7 * self.distancia_us_filtrada) + (0.3 * distance_cm)

            if not self.distancia_us_inicio_capturada:
                self.distancia_us_muestras_arranque.append(distance_cm)
                if len(self.distancia_us_muestras_arranque) > 12:
                    self.distancia_us_muestras_arranque.pop(0)

    def capturar_distancia_inicial(self, timeout_seconds=INITIAL_DISTANCE_CAPTURE_SECONDS):
        if not self.serial_enabled:
            return

        deadline = time.time() + timeout_seconds
        while self.running and time.time() < deadline:
            muestras_validas = [
                value for value in self.distancia_us_muestras_arranque
                if value <= INITIAL_DISTANCE_REFERENCE_MAX_CM
            ]
            if len(muestras_validas) >= INITIAL_DISTANCE_MIN_SAMPLES:
                ventana = muestras_validas[-INITIAL_DISTANCE_MIN_SAMPLES:]
                ventana_ordenada = sorted(ventana)
                mediana = float(ventana_ordenada[len(ventana_ordenada) // 2])
                dispersion = max(ventana) - min(ventana)
                if dispersion <= 6.0:
                    self.distancia_us_inicio = mediana
                    self.distancia_us_inicio_capturada = True
                    print(f"[US] Distancia trasera inicial guardada: {self.distancia_us_inicio:.1f} cm")
                    return
            time.sleep(0.02)

        muestras_validas = [
            value for value in self.distancia_us_muestras_arranque
            if value <= INITIAL_DISTANCE_REFERENCE_MAX_CM
        ]
        if muestras_validas and not self.distancia_us_inicio_capturada:
            ventana = muestras_validas[-INITIAL_DISTANCE_MIN_SAMPLES:]
            promedio = sum(ventana) / len(ventana)
            self.distancia_us_inicio = float(promedio)
            self.distancia_us_inicio_capturada = True
            print(f"[US] Distancia trasera inicial guardada con promedio: {self.distancia_us_inicio:.1f} cm")
        elif not self.distancia_us_inicio_capturada:
            print("[WARN] No se obtuvo una distancia trasera inicial valida.")

    def weighted_average(self, samples):
        if not samples:
            return None
        weighted_sum = sum(value * weight for value, weight in samples)
        total_weight = sum(weight for _value, weight in samples)
        if total_weight <= 0:
            return None
        return weighted_sum / total_weight

    def classify_wall_mode(self, muro_izq, muro_der):
        if muro_izq != -1 and muro_der != -1:
            return "both"
        if muro_izq != -1:
            return "left"
        if muro_der != -1:
            return "right"
        return "none"

    def build_visual_signature(self, muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x):
        wall_mode = self.classify_wall_mode(muro_izq, muro_der)
        if wall_mode == "none":
            return None

        width_px = float(ancho_visible) if ancho_visible is not None else float(self.track_half_width_px * 2.0)
        return {
            "wall_mode": wall_mode,
            "left_wall_px": None if muro_izq == -1 else float(muro_izq),
            "right_wall_px": None if muro_der == -1 else float(muro_der),
            "width_px": round(width_px, 2),
            "center_offset_px": round(float(centro_pista_x - 160.0), 2),
            "dark_ratio_center": round(float(dark_ratio_centro), 3),
        }

    def median_signature_from_samples(self, samples):
        if not samples:
            return None

        wall_mode_counts = {}
        for sample in samples:
            wall_mode_counts[sample["wall_mode"]] = wall_mode_counts.get(sample["wall_mode"], 0) + 1
        wall_mode = max(wall_mode_counts, key=wall_mode_counts.get)

        def median_of(key):
            values = [sample[key] for sample in samples if sample[key] is not None]
            if not values:
                return None
            values.sort()
            return float(values[len(values) // 2])

        signature = {
            "wall_mode": wall_mode,
            "left_wall_px": None if wall_mode == "right" else round(median_of("left_wall_px") or 0.0, 2),
            "right_wall_px": None if wall_mode == "left" else round(median_of("right_wall_px") or 0.0, 2),
            "width_px": round(median_of("width_px") or float(self.track_half_width_px * 2.0), 2),
            "center_offset_px": round(median_of("center_offset_px") or 0.0, 2),
            "dark_ratio_center": round(median_of("dark_ratio_center") or 0.0, 3),
        }
        if wall_mode == "left":
            signature["right_wall_px"] = None
        elif wall_mode == "right":
            signature["left_wall_px"] = None
        return signature

    def is_stable_straight_candidate(self, estado, muro_izq, muro_der, dark_ratio_centro, current_time):
        return (
            estado == "CENTRADO"
            and muro_izq != -1
            and muro_der != -1
            and self.centered_since is not None
            and current_time - self.centered_since >= STRAIGHT_STABLE_HOLD_SECONDS
            and dark_ratio_centro <= STRAIGHT_PROFILE_DARK_RATIO_MAX
        )

    def capture_initial_visual_signature(self, muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x):
        if self.initial_visual_signature_captured:
            return

        signature = self.build_visual_signature(
            muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
        )
        if signature is None:
            return

        self.initial_visual_signature_samples.append(signature)
        if len(self.initial_visual_signature_samples) < INITIAL_VISUAL_SIGNATURE_MIN_SAMPLES:
            return

        self.initial_visual_signature = self.median_signature_from_samples(
            self.initial_visual_signature_samples[:INITIAL_VISUAL_SIGNATURE_MIN_SAMPLES]
        )
        self.initial_visual_signature_captured = True
        print(f"[VISION] Firma visual inicial guardada: {self.initial_visual_signature}")

    def update_straight_profile(self, muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x):
        signature = self.build_visual_signature(
            muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
        )
        if signature is None:
            return

        self.straight_profile_samples.append(signature)
        if len(self.straight_profile_samples) > STRAIGHT_PROFILE_MAX_SAMPLES:
            self.straight_profile_samples.pop(0)

        if len(self.straight_profile_samples) >= STRAIGHT_PROFILE_MIN_SAMPLES:
            self.calibrated_straight_signature = self.median_signature_from_samples(
                self.straight_profile_samples
            )

    def signature_matches_target(self, current_signature, target, center_tolerance_px, width_ratio_tolerance):
        if current_signature is None or target is None:
            return False
        if current_signature["wall_mode"] != target["wall_mode"]:
            return False

        width_tol = max(VISUAL_SIGNATURE_WIDTH_TOLERANCE_PX, target["width_px"] * width_ratio_tolerance)
        if abs(current_signature["width_px"] - target["width_px"]) > width_tol:
            return False
        if abs(current_signature["center_offset_px"] - target["center_offset_px"]) > center_tolerance_px:
            return False
        if abs(current_signature["dark_ratio_center"] - target["dark_ratio_center"]) > VISUAL_SIGNATURE_DARK_TOLERANCE:
            return False
        if target["left_wall_px"] is not None and current_signature["left_wall_px"] is not None:
            if abs(current_signature["left_wall_px"] - target["left_wall_px"]) > VISUAL_SIGNATURE_WALL_TOLERANCE_PX:
                return False
        if target["right_wall_px"] is not None and current_signature["right_wall_px"] is not None:
            if abs(current_signature["right_wall_px"] - target["right_wall_px"]) > VISUAL_SIGNATURE_WALL_TOLERANCE_PX:
                return False
        return True

    def visual_signature_matches(self, muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x):
        if self.initial_visual_signature is None:
            return False
        current_signature = self.build_visual_signature(
            muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
        )
        return self.signature_matches_target(
            current_signature,
            self.initial_visual_signature,
            VISUAL_SIGNATURE_CENTER_TOLERANCE_PX,
            0.18,
        )

    def calibrated_straight_matches(self, muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x):
        if self.calibrated_straight_signature is None:
            return False
        current_signature = self.build_visual_signature(
            muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
        )
        return self.signature_matches_target(
            current_signature,
            self.calibrated_straight_signature,
            FINAL_STRAIGHT_CENTER_TOLERANCE_PX,
            0.15,
        )

    def scan_walls(self, binarizada):
        alto_roi, ancho_roi = binarizada.shape
        centro_x = ancho_roi // 2
        left_samples = []
        right_samples = []
        width_samples = []
        dark_ratio_samples = []

        for offset, weight in zip(self.scan_row_offsets, self.scan_row_weights):
            row_y = int(np.clip((alto_roi // 2) + offset, 0, alto_roi - 1))
            row = binarizada[row_y, :]
            muro_izq = -1
            muro_der = -1

            for x in range(centro_x, -1, -1):
                if row[x] == 0:
                    muro_izq = x
                    break
            for x in range(centro_x, ancho_roi):
                if row[x] == 0:
                    muro_der = x
                    break

            center_window = row[
                max(0, centro_x - FRONTAL_CENTER_HALF_WIDTH): min(ancho_roi, centro_x + FRONTAL_CENTER_HALF_WIDTH + 1)
            ]
            dark_ratio = float(np.mean(center_window == 0)) if center_window.size else 0.0
            dark_ratio_samples.append((dark_ratio, weight))

            if muro_izq != -1:
                left_samples.append((muro_izq, weight))
            if muro_der != -1:
                right_samples.append((muro_der, weight))
            if muro_izq != -1 and muro_der != -1 and muro_der > muro_izq:
                width_samples.append((muro_der - muro_izq, weight))

        muro_izq = self.weighted_average(left_samples)
        muro_der = self.weighted_average(right_samples)
        ancho_visible = self.weighted_average(width_samples)
        dark_ratio_centro = self.weighted_average(dark_ratio_samples) or 0.0
        return (
            -1 if muro_izq is None else int(round(muro_izq)),
            -1 if muro_der is None else int(round(muro_der)),
            None if ancho_visible is None else float(ancho_visible),
            dark_ratio_centro,
            len(width_samples),
        )

    def suavizar_centro(self, target_center_x, alpha):
        self.centro_suavizado = ((1.0 - alpha) * self.centro_suavizado) + (alpha * target_center_x)
        return self.centro_suavizado

    def limitar_velocidad(self, target_speed):
        self.current_speed = max(0, min(MAX_MOTOR_SPEED, int(round(target_speed))))
        return self.current_speed

    def limitar_direccion(self, target_angle, max_step):
        target_angle = int(round(target_angle))
        delta = target_angle - self.current_angle
        if delta > max_step:
            target_angle = self.current_angle + max_step
        elif delta < -max_step:
            target_angle = self.current_angle - max_step
        self.current_angle = max(STEER_MIN_ANGLE, min(STEER_MAX_ANGLE, int(target_angle)))
        return self.current_angle

    def aplicar_sesgo_curva(self, centro_pista_x, muro_izq, muro_der, bias_px, sentido_giro):
        if sentido_giro == "AUTO" or bias_px <= 0:
            return int(round(centro_pista_x))

        target_x = float(centro_pista_x)
        if sentido_giro == "IZQUIERDA":
            target_x += bias_px
            if muro_izq != -1:
                target_x = max(target_x, muro_izq + CURVE_INSIDE_MARGIN_PX)
            if muro_der != -1:
                target_x = min(target_x, muro_der - CURVE_OUTSIDE_MARGIN_PX)
        else:
            target_x -= bias_px
            if muro_der != -1:
                target_x = min(target_x, muro_der - CURVE_INSIDE_MARGIN_PX)
            if muro_izq != -1:
                target_x = max(target_x, muro_izq + CURVE_OUTSIDE_MARGIN_PX)
        return int(round(target_x))

    def forzar_pre_giro(self, target_angle, min_delta, sentido_giro):
        if min_delta <= 0 or sentido_giro == "AUTO":
            return target_angle
        if sentido_giro == "IZQUIERDA":
            return max(target_angle, STEER_CENTER_ANGLE + min_delta)
        return min(target_angle, STEER_CENTER_ANGLE - min_delta)

    def intentar_confirmar_sentido_giro(self, binarizada):
        if self.SENTIDO_GIRO != "AUTO":
            return

        blancos_izq = int(np.sum(binarizada[:, :160] == 255))
        blancos_der = int(np.sum(binarizada[:, 160:] == 255))
        diferencia = blancos_der - blancos_izq

        if diferencia != 0:
            self.turn_direction_candidate = "DERECHA" if diferencia > 0 else "IZQUIERDA"
        if abs(diferencia) < TURN_DIRECTION_MIN_DIFF_PIXELS:
            return

        sentido_detectado = self.turn_direction_candidate
        self.turn_direction_votes[sentido_detectado] += 1
        if self.turn_direction_votes[sentido_detectado] >= TURN_DIRECTION_CONFIRM_VOTES:
            self.SENTIDO_GIRO = sentido_detectado
            descripcion = "HORARIO" if sentido_detectado == "DERECHA" else "ANTIHORARIO"
            print(
                f"\n[INFO] SENTIDO {descripcion} CONFIRMADO "
                f"(blancos izq={blancos_izq}, der={blancos_der}, votos={self.turn_direction_votes})\n"
            )

    def obtener_sentido_giro_operativo(self):
        if self.SENTIDO_GIRO != "AUTO":
            return self.SENTIDO_GIRO
        if self.turn_direction_candidate is not None:
            return self.turn_direction_candidate
        if self.turn_direction_votes["DERECHA"] > self.turn_direction_votes["IZQUIERDA"]:
            return "DERECHA"
        if self.turn_direction_votes["IZQUIERDA"] > self.turn_direction_votes["DERECHA"]:
            return "IZQUIERDA"
        return "AUTO"

    def pared_visible_es_interior(self, estado, sentido_giro):
        if sentido_giro == "AUTO":
            return False
        if sentido_giro == "DERECHA":
            return estado == "MURO_DER"
        return estado == "MURO_IZQ"

    def pared_visible_es_exterior(self, estado, sentido_giro):
        if sentido_giro == "AUTO":
            return False
        if sentido_giro == "DERECHA":
            return estado == "MURO_IZQ"
        return estado == "MURO_DER"

    def construir_vista_principal(self, frame, estado, elapsed_seconds):
        preview = frame.copy()
        cv2.putText(preview, f"Estado: {estado}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            preview,
            f"Vel: {self.current_speed}  Ang: {self.current_angle}  t={elapsed_seconds:6.2f}s",
            (10, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        us_actual = "--.-" if self.distancia_us_filtrada is None else f"{self.distancia_us_filtrada:5.1f}"
        us_inicio = "--.-" if self.distancia_us_inicio is None else f"{self.distancia_us_inicio:5.1f}"
        cv2.putText(preview, f"US trasero: {us_actual} cm  inicio: {us_inicio} cm", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 255, 180), 1, cv2.LINE_AA)
        cv2.putText(preview, "Tecla: q=salir", (10, preview.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return preview

    def process_vision(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la camara en el indice {CAMERA_INDEX}.")

        if SHOW_DEBUG_WINDOWS:
            cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
            cv2.namedWindow(WINDOW_GREEN, cv2.WINDOW_NORMAL)
            cv2.namedWindow(WINDOW_RED, cv2.WINDOW_NORMAL)
            cv2.namedWindow(WINDOW_BINARY, cv2.WINDOW_NORMAL)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.capturar_distancia_inicial()
        last_time = time.time()

        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    continue

                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (7, 7), 0)
                roi_blur = blur[80:140, 0:320]
                _, binarizada = cv2.threshold(roi_blur, 95, 255, cv2.THRESH_BINARY)
                binarizada = cv2.morphologyEx(binarizada, cv2.MORPH_OPEN, self.morph_kernel)
                binarizada = cv2.morphologyEx(binarizada, cv2.MORPH_CLOSE, self.morph_kernel)

                roi_color = frame[80:160, 0:320]
                hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)

                lower_green = np.array([40, 70, 50])
                upper_green = np.array([85, 255, 255])
                mask_green = cv2.inRange(hsv, lower_green, upper_green)
                mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.morph_kernel)

                lower_red1 = np.array([0, 70, 50])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([170, 70, 50])
                upper_red2 = np.array([180, 255, 255])
                mask_red = cv2.bitwise_or(
                    cv2.inRange(hsv, lower_red1, upper_red1),
                    cv2.inRange(hsv, lower_red2, upper_red2),
                )
                mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.morph_kernel)

                obstaculo_tipo = "NINGUNO"
                obstaculo_cx = 160
                area_max_obs = 0
                x_obs, w_obs = 0, 0

                contornos_v, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in contornos_v:
                    area = cv2.contourArea(c)
                    if area > 350 and area > area_max_obs:
                        area_max_obs = area
                        x, _y, w, _h = cv2.boundingRect(c)
                        x_obs, w_obs = x, w
                        obstaculo_cx = x + (w // 2)
                        obstaculo_tipo = "VERDE"

                contornos_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in contornos_r:
                    area = cv2.contourArea(c)
                    if area > 350 and area > area_max_obs:
                        area_max_obs = area
                        x, _y, w, _h = cv2.boundingRect(c)
                        x_obs, w_obs = x, w
                        obstaculo_cx = x + (w // 2)
                        obstaculo_tipo = "ROJO"

                if obstaculo_tipo != "NINGUNO":
                    x_inicio = max(0, x_obs - 20)
                    x_fin = min(320, x_obs + w_obs + 20)
                    binarizada[:, x_inicio:x_fin] = 255

                muro_izq, muro_der, ancho_visible, dark_ratio_centro, filas_validas = self.scan_walls(binarizada)
                centro_pista_x = 160
                estado = "DESCONOCIDO"

                if ancho_visible is not None and 120 <= ancho_visible <= 310:
                    observed_half_width = ancho_visible / 2.0
                    self.track_half_width_px = ((1.0 - TRACK_WIDTH_ALPHA) * self.track_half_width_px) + (TRACK_WIDTH_ALPHA * observed_half_width)

                if muro_izq != -1 and muro_der != -1:
                    centro_pista_x = (muro_izq + muro_der) // 2
                    estado = "CENTRADO"
                elif muro_izq != -1:
                    centro_pista_x = int(round(muro_izq + self.track_half_width_px))
                    estado = "MURO_IZQ"
                elif muro_der != -1:
                    centro_pista_x = int(round(muro_der - self.track_half_width_px))
                    estado = "MURO_DER"
                else:
                    estado = "CEGUERA_BLANCA"

                stable_straight_candidate = self.is_stable_straight_candidate(
                    estado, muro_izq, muro_der, dark_ratio_centro, current_time
                )
                if stable_straight_candidate:
                    self.capture_initial_visual_signature(
                        muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
                    )
                    self.update_straight_profile(
                        muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
                    )

                frente_bloqueado = (
                    dark_ratio_centro >= FRONTAL_DARK_RATIO_THRESHOLD
                    and (ancho_visible is None or ancho_visible < (self.track_half_width_px * FRONTAL_WIDTH_TRIGGER_RATIO))
                )
                if frente_bloqueado:
                    self.front_wall_score = min(5, self.front_wall_score + 1)
                else:
                    self.front_wall_score = max(0, self.front_wall_score - 1)

                es_vertice_curva = self.front_wall_score >= FRONT_WALL_SCORE_TRIGGER
                if es_vertice_curva:
                    estado = "MURO_FRONTAL"

                if obstaculo_tipo != "NINGUNO" and not es_vertice_curva:
                    distancia_evasion = 110
                    if obstaculo_tipo == "ROJO":
                        centro_pista_x = obstaculo_cx + distancia_evasion
                        estado = "EVADIENDO_ROJO"
                    else:
                        centro_pista_x = obstaculo_cx - distancia_evasion
                        estado = "EVADIENDO_VERDE"
                    if muro_der != -1:
                        centro_pista_x = min(centro_pista_x, muro_der - 45)
                    if muro_izq != -1:
                        centro_pista_x = max(centro_pista_x, muro_izq + 45)

                if estado == "CENTRADO" and muro_izq != -1 and muro_der != -1:
                    if self.centered_since is None:
                        self.centered_since = current_time
                else:
                    self.centered_since = None

                curve_width_ratio = None
                if ancho_visible is not None and self.track_half_width_px > 1:
                    curve_width_ratio = ancho_visible / max(1.0, self.track_half_width_px * 2.0)

                sentido_giro_operativo = self.obtener_sentido_giro_operativo()
                pared_interior_visible = self.pared_visible_es_interior(estado, sentido_giro_operativo)
                pared_exterior_visible = self.pared_visible_es_exterior(estado, sentido_giro_operativo)
                curve_exit_active = (
                    estado == "CENTRADO" and current_time - self.last_non_centered_time < EXIT_CURVE_HOLD_SECONDS
                )
                curve_entry_active = (
                    sentido_giro_operativo != "AUTO"
                    and not es_vertice_curva
                    and (
                        (
                            estado == "CENTRADO"
                            and curve_width_ratio is not None
                            and curve_width_ratio < CURVE_ENTRY_WIDTH_RATIO
                            and dark_ratio_centro >= CURVE_ENTRY_DARK_RATIO_THRESHOLD
                        )
                        or (pared_exterior_visible and dark_ratio_centro >= CURVE_ENTRY_DARK_RATIO_THRESHOLD)
                    )
                )

                if not es_vertice_curva:
                    center_alpha = CENTER_ALPHA_CURVE if estado != "CENTRADO" else CENTER_ALPHA_STRAIGHT
                    if "EVADIENDO" in estado:
                        center_alpha = max(center_alpha, 0.42)
                    centro_pista_x = int(round(self.suavizar_centro(centro_pista_x, center_alpha)))

                    curve_bias_px = 0
                    if curve_entry_active:
                        curve_bias_px = max(curve_bias_px, CURVE_ENTRY_BIAS_PX)
                    if estado != "CENTRADO":
                        curve_bias_px = max(curve_bias_px, CURVE_APEX_BIAS_PX)
                    if pared_interior_visible:
                        curve_bias_px = max(curve_bias_px, CURVE_APEX_BIAS_PX + CURVE_INNER_WALL_EXTRA_BIAS_PX)
                    if current_time < self.curve_commit_until:
                        curve_bias_px = max(curve_bias_px, CURVE_APEX_BIAS_PX)
                    if curve_exit_active:
                        curve_bias_px = max(curve_bias_px, CURVE_EXIT_BIAS_PX)

                    centro_pista_x = self.aplicar_sesgo_curva(
                        centro_pista_x, muro_izq, muro_der, curve_bias_px, sentido_giro_operativo
                    )

                if self.SENTIDO_GIRO == "AUTO" and es_vertice_curva:
                    self.intentar_confirmar_sentido_giro(binarizada)
                    sentido_giro_operativo = self.obtener_sentido_giro_operativo()

                if es_vertice_curva:
                    if not self.en_curva and (current_time - self.ultimo_tiempo_curva > 2.5):
                        self.en_curva = True
                        self.ultimo_tiempo_curva = current_time
                        self.curvas_superadas += 1
                        self.curve_commit_until = current_time + CURVE_COMMIT_SECONDS
                        self.last_curve_direction = sentido_giro_operativo if sentido_giro_operativo != "AUTO" else None

                        if self.curvas_superadas % 4 == 0:
                            self.vueltas_completadas += 1
                            print(f"\n[INFO] VUELTA {self.vueltas_completadas}/3 COMPLETADA\n")
                            if self.vueltas_completadas >= 3:
                                self.finish_pending = True
                                self.finish_ready_after = current_time + FINAL_LAP_MIN_TIME_AFTER_CURVE
                                self.finish_curve_count_target = self.curvas_superadas
                                self.finish_distance_armed = self.distancia_us_filtrada is not None
                                self.finish_distance_prev = self.distancia_us_filtrada
                                print("\n[INFO] TERCERA VUELTA DETECTADA. ESPERANDO RECTA DE META\n")
                elif current_time - self.ultimo_tiempo_curva > 2.5:
                    self.en_curva = False

                if estado == "MURO_FRONTAL":
                    if sentido_giro_operativo == "DERECHA":
                        target_angle = FRONTAL_RIGHT_TURN_ANGLE
                    elif sentido_giro_operativo == "IZQUIERDA":
                        target_angle = FRONTAL_LEFT_TURN_ANGLE
                    else:
                        target_angle = STEER_CENTER_ANGLE
                    self.pid.integral = 0
                    self.limitar_direccion(target_angle, STEERING_RATE_LIMIT_FRONTAL)
                    velocidad_frontal = FRONTAL_TURN_SPEED if sentido_giro_operativo != "AUTO" else FINAL_APPROACH_SPEED
                    self.limitar_velocidad(velocidad_frontal)
                else:
                    error_absoluto_real = 160 - centro_pista_x
                    en_transicion_salida_curva = curve_exit_active
                    en_compromiso_curva = current_time < self.curve_commit_until
                    target_speed = CURVE_RECOVERY_SPEED

                    if abs(error_absoluto_real) < DEAD_ZONE_PX and estado == "CENTRADO":
                        target_angle = STEER_CENTER_ANGLE
                        self.pid.integral *= 0.5
                    else:
                        if "EVADIENDO" in estado:
                            self.pid.kp = 0.09
                            self.pid.kd = 0.12
                        else:
                            self.pid.kp = 0.05 if estado == "CENTRADO" else 0.10
                            self.pid.kd = 0.11 if estado == "CENTRADO" else 0.14

                        correccion_pid = self.pid.compute(centro_pista_x, dt)
                        angulo_pid = int(STEER_CENTER_ANGLE + correccion_pid)
                        if estado == "CENTRADO":
                            angle_min = 76 if curve_entry_active or en_compromiso_curva else 78
                            angle_max = 96 if curve_entry_active or en_compromiso_curva else 94
                            target_angle = max(angle_min, min(angle_max, angulo_pid))
                        else:
                            target_angle = max(70, min(108, angulo_pid))

                    if estado != "CENTRADO":
                        self.last_non_centered_time = current_time

                    curva_grados = abs(target_angle - STEER_CENTER_ANGLE)
                    if "EVADIENDO" in estado:
                        target_speed = 188 if curva_grados > 11 else 198
                    elif estado == "CENTRADO" and abs(error_absoluto_real) < 8 and curva_grados <= 3:
                        target_speed = CENTERED_CRUISE_SPEED
                    elif estado == "CENTRADO":
                        target_speed = 214 if curva_grados <= 8 else 198
                    else:
                        target_speed = 196 if curva_grados <= 10 else 184

                    if curve_entry_active:
                        target_speed = min(target_speed, CURVE_ENTRY_SPEED)
                        target_angle = self.forzar_pre_giro(target_angle, CURVE_ENTRY_MIN_STEER_DELTA, sentido_giro_operativo)
                    if en_compromiso_curva:
                        target_angle = self.forzar_pre_giro(target_angle, CURVE_COMMIT_MIN_STEER_DELTA, sentido_giro_operativo)
                        if self.last_curve_direction == "DERECHA":
                            target_angle = min(target_angle, 94)
                        elif self.last_curve_direction == "IZQUIERDA":
                            target_angle = max(target_angle, 78)
                        target_speed = min(target_speed, CURVE_RECOVERY_SPEED)
                    if en_transicion_salida_curva:
                        target_speed = min(target_speed, CURVE_RECOVERY_SPEED)
                        target_angle = self.forzar_pre_giro(target_angle, CURVE_EXIT_MIN_STEER_DELTA, sentido_giro_operativo)
                        if self.last_curve_direction == "DERECHA":
                            target_angle = min(94, max(82, min(target_angle, 94)))
                        elif self.last_curve_direction == "IZQUIERDA":
                            target_angle = max(78, min(90, max(target_angle, 78)))
                        else:
                            target_angle = max(78, min(94, target_angle))

                    rate_limit = STEERING_RATE_LIMIT_STRAIGHT if estado == "CENTRADO" else STEERING_RATE_LIMIT_CURVE
                    self.limitar_direccion(target_angle, rate_limit)
                    self.limitar_velocidad(target_speed)

                if self.finish_pending and self.curvas_superadas != self.finish_curve_count_target:
                    print("[WARN] Se perdio la recta objetivo de meta; deteniendo para evitar otra recta.")
                    self.limitar_velocidad(0)
                    self.current_angle = STEER_CENTER_ANGLE
                    self.running = False

                if (
                    self.finish_pending
                    and self.curvas_superadas == self.finish_curve_count_target
                    and current_time >= self.finish_ready_after
                    and self.distancia_us_filtrada is not None
                    and not self.finish_distance_armed
                ):
                    self.finish_distance_armed = True
                    self.finish_distance_prev = self.distancia_us_filtrada

                final_straight_stable = (
                    estado == "CENTRADO"
                    and self.centered_since is not None
                    and current_time - self.centered_since >= FINAL_STRAIGHT_MIN_CENTERED_SECONDS
                )
                final_signature_ready = (
                    self.finish_pending
                    and self.curvas_superadas == self.finish_curve_count_target
                    and current_time >= self.finish_ready_after
                    and final_straight_stable
                )
                if final_signature_ready:
                    visual_match = self.visual_signature_matches(
                        muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
                    )
                    straight_match = self.calibrated_straight_matches(
                        muro_izq, muro_der, ancho_visible, dark_ratio_centro, centro_pista_x
                    )
                    final_lane_centered = (
                        estado == "CENTRADO"
                        and muro_izq != -1
                        and muro_der != -1
                        and abs(160 - centro_pista_x) <= FINAL_LANE_CENTER_TOLERANCE_PX
                    )

                    if (
                        self.finish_distance_armed
                        and self.finish_distance_prev is not None
                        and self.distancia_us_inicio is not None
                        and self.distancia_us_filtrada is not None
                        and self.initial_visual_signature_captured
                    ):
                        distancia_objetivo = self.distancia_us_inicio
                        distancia_actual = self.distancia_us_filtrada
                        distancia_previa = self.finish_distance_prev
                        cruzo_objetivo = (
                            distancia_previa < (distancia_objetivo - FINAL_DISTANCE_TOLERANCE_CM)
                            and distancia_actual >= (distancia_objetivo - FINAL_DISTANCE_TOLERANCE_CM)
                        )
                        distancia_cercana = distancia_actual >= (distancia_objetivo - FINAL_DISTANCE_TOLERANCE_CM)
                        if visual_match and straight_match and final_lane_centered and (cruzo_objetivo or distancia_cercana):
                            print("\n[INFO] META FINAL CONFIRMADA: DETENIENDO MOTORES\n")
                            self.limitar_velocidad(0)
                            self.current_angle = STEER_CENTER_ANGLE
                            self.running = False
                        elif final_straight_stable and (
                            (visual_match and straight_match and final_lane_centered)
                            or distancia_actual >= (distancia_objetivo - FINAL_DISTANCE_BRAKE_MARGIN_CM)
                        ):
                            self.limitar_velocidad(min(self.current_speed, FINAL_APPROACH_SPEED))
                    elif self.initial_visual_signature_captured and visual_match and straight_match and final_lane_centered:
                        print("\n[INFO] META FINAL CONFIRMADA POR FIRMA VISUAL: DETENIENDO MOTORES\n")
                        self.limitar_velocidad(0)
                        self.current_angle = STEER_CENTER_ANGLE
                        self.running = False
                    else:
                        self.limitar_velocidad(min(self.current_speed, FINAL_APPROACH_SPEED))

                if self.finish_pending and self.finish_distance_armed and self.distancia_us_filtrada is not None:
                    self.finish_distance_prev = self.distancia_us_filtrada

                elapsed_seconds = 0.0 if self.start_time is None else (current_time - self.start_time)
                vista_principal = self.construir_vista_principal(frame, estado, elapsed_seconds)
                vision_binarizada = cv2.cvtColor(binarizada, cv2.COLOR_GRAY2BGR)

                if SHOW_DEBUG_WINDOWS:
                    cv2.imshow(WINDOW_MAIN, vista_principal)
                    cv2.imshow(WINDOW_GREEN, mask_green)
                    cv2.imshow(WINDOW_RED, mask_red)
                    cv2.imshow(WINDOW_BINARY, vision_binarizada)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        print("[SISTEMA] Detencion manual solicitada.")
                        self.running = False
                        break

                if int(current_time * 10) % 5 == 0:
                    print(
                        f"| Estado: {estado:<15} | Muros: [I:{muro_izq} D:{muro_der}] | "
                        f"Vel: {self.current_speed} | Angulo: {self.current_angle} |"
                    )

                time.sleep(0.01)
        finally:
            cap.release()
            if SHOW_DEBUG_WINDOWS:
                cv2.destroyAllWindows()

    def main_loop(self):
        if self.serial_enabled and self.ser is not None:
            self.ser.write(b"<0,120>\n")
            time.sleep(0.5)
            self.ser.write(b"<0,60>\n")
            time.sleep(0.5)
            self.ser.write(f"<0,{STEER_CENTER_ANGLE}>\n".encode("utf-8"))

            self.serial_reader_thread = threading.Thread(
                target=self.serial_reader_loop,
                name="serial_reader",
                daemon=True,
            )
            self.serial_reader_thread.start()

        vt = threading.Thread(target=self.process_vision)
        vt.start()

        try:
            print("\n[SISTEMA] INICIALIZADO. ESPERANDO PULSADOR PARA COMENZAR...\n")
            while self.running:
                if self.estado_general == "ESPERA":
                    self.current_speed = 0
                    self.current_angle = STEER_CENTER_ANGLE
                    if self.boton_presionado:
                        print("\n[!] BOTON PRESIONADO. INICIANDO CARRERA [!]\n")
                        self.estado_general = "CARRERA"
                        self.start_time = time.time()
                    else:
                        if self.serial_enabled and self.ser is not None:
                            self.ser.write(f"<0,{STEER_CENTER_ANGLE}>\n".encode("utf-8"))
                        time.sleep(0.05)
                        continue

                if self.serial_enabled and self.ser is not None:
                    paquete = f"<{self.current_speed},{self.current_angle}>\n"
                    self.ser.write(paquete.encode("utf-8"))
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.running = False
        finally:
            if self.serial_enabled and self.ser is not None:
                self.ser.write(f"<0,{STEER_CENTER_ANGLE}>\n".encode("utf-8"))
            vt.join()
            if self.serial_reader_thread is not None:
                self.serial_reader_thread.join(timeout=1.0)
            if self.ser is not None:
                self.ser.close()


if __name__ == "__main__":
    bot = WROAutonomousCar(serial_port=SERIAL_PORT)
    bot.main_loop()
