import cv2
import os
import numpy as np
import serial
import time
import threading

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=160): 
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class WROAutonomousCar:
    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
        time.sleep(2)
        
        # --- CONFIGURACIÓN ESTRATÉGICA ---
        self.SENTIDO_GIRO = "AUTO" # Cambiar a "DERECHA" si la pista lo requiere
        self.memoria_muro_exterior = "NINGUNO"
        self.MITAD_ANCHO_PISTA_PX = 140 # Píxeles estimados desde un muro hasta el centro
        
        # --- VARIABLES DE REGLAMENTO WRO ---
        self.vueltas_completadas = 0
        self.curvas_superadas = 0
        self.en_curva = False
        
        self.ultimo_tiempo_curva = time.time()
        
        # PID más agresivo en P para que reaccione rápido a un solo muro
        self.pid = PIDController(kp=0.06, ki=0.000, kd=0.20) 
        self.running = True
        
        self.centro_suavizado = 160.0
        
        self.current_speed = 0
        self.current_angle = 86 
        
        print(f"[SISTEMA] Algoritmo Raycasting Iniciado. Giro ciego: {self.SENTIDO_GIRO}")
        
    def process_vision(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)#medir esta tolerancia
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        last_time = time.time()

        while self.running:
            ret, frame = cap.read()
            if not ret: continue

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # =======================================================
            # 1. OPENCV: PROCESAMIENTO Y UMBRAL FIJO
            # =======================================================
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (7, 7), 0)
            
            # Recortamos una franja clave (ajusta esto si la cámara sube o baja)
            y_arriba = 80
            y_abajo = 140
            roi_blur = blur[y_arriba:y_abajo, 0:320]
            
            # Blanco brillante (Piso) = 255. Oscuro (Muro) = 0.
            _, binarizada = cv2.threshold(roi_blur, 95, 255, cv2.THRESH_BINARY)#medir esta tolerancia
            
            # =======================================================
            # 1.5 DETECCIÓN DE OBSTÁCULOS (ROJO Y VERDE)
            # =======================================================
            # Usamos una franja similar para que las coordenadas X coincidan
            roi_color = frame[80:160, 0:320] 
            hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
            
            # Rangos VERDE (Ajusta estos números viendo la ventana "Filtro VERDE")
            lower_green = np.array([40, 70, 50])
            upper_green = np.array([85, 255, 255])
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            
            # Rangos ROJO (Ajusta estos números viendo la ventana "Filtro ROJO")
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                                      cv2.inRange(hsv, lower_red2, upper_red2))
            
            obstaculo_tipo = "NINGUNO"
            obstaculo_cx = 160
            area_max_obs = 0
            x_obs, w_obs = 0, 0 # Guardaremos el ancho del pilar para borrarlo
            
            contornos_v, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contornos_v:
                area = cv2.contourArea(c)
                if area > 350 and area > area_max_obs:
                    area_max_obs = area
                    x, y, w, h = cv2.boundingRect(c)
                    x_obs, w_obs = x, w
                    obstaculo_cx = x + (w // 2)
                    obstaculo_tipo = "VERDE"
                        
            contornos_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contornos_r:
                area = cv2.contourArea(c)
                if area > 350 and area > area_max_obs:
                    area_max_obs = area
                    x, y, w, h = cv2.boundingRect(c)
                    x_obs, w_obs = x, w
                    obstaculo_cx = x + (w // 2)
                    obstaculo_tipo = "ROJO"

            # =======================================================
            # 1.6 LA CAPA DE INVISIBILIDAD (Hack de Muros)
            # =======================================================
            if obstaculo_tipo != "NINGUNO":
                # Si vemos un pilar, pintamos de BLANCO (255) esa zona en la imagen binarizada
                # Le damos un margen de 20 píxeles para borrar la sombra completa.
                # ¡Así el Raycasting ignorará el pilar y buscará los muros reales!
                x_inicio = max(0, x_obs - 20)
                x_fin = min(320, x_obs + w_obs + 20)
                binarizada[:, x_inicio:x_fin] = 255
            
            cv2.imshow("Filtro VERDE", mask_green)
            cv2.imshow("Filtro ROJO", mask_red)
            cv2.imshow("Vision Binarizada", binarizada)
            cv2.waitKey(1)
            
            # =======================================================
            # 2. ALGORITMO DE RAYCASTING (Escanear la línea media)
            # =======================================================
            alto_roi, ancho_roi = binarizada.shape
            linea_escaneo = binarizada[alto_roi // 2, :] # Extraer la fila central de la franja
            
            muro_izq = -1
            muro_der = -1
            
            # Disparar rayo hacia la izquierda desde el centro (160 a 0)

            # =======================================================
            # 3. MÁQUINA DE ESTADOS MATEMÁTICA
            # =======================================================
            centro_pista_x = 160
            estado = "DESCONOCIDO"

            if linea_escaneo[160] == 0:
                estado = "MURO_FRONTAL"
            else:
                for x in range(160, -1, -1):
                    if linea_escaneo[x] == 0: 
                        muro_izq = x; break
                
                for x in range(160, ancho_roi):
                    if linea_escaneo[x] == 0: 
                        muro_der = x; break

                # Máquina de estados normal
                if muro_izq != -1 and muro_der != -1:
                    centro_pista_x = (muro_izq + muro_der) // 2
                    estado = "CENTRADO"
                elif muro_izq != -1 and muro_der == -1:
                    centro_pista_x = muro_izq + self.MITAD_ANCHO_PISTA_PX
                    estado = "MURO_IZQ"
                elif muro_izq == -1 and muro_der != -1:
                    centro_pista_x = muro_der - self.MITAD_ANCHO_PISTA_PX
                    estado = "MURO_DER"
                else:
                    estado = "CEGUERA_BLANCA"

            # =======================================================
            # 3.5 AUTODETECCIÓN DE SENTIDO (Solo se ejecuta 1 vez)
            # =======================================================
            es_vertice_curva = estado in ["MURO_FRONTAL"]
            
            # =======================================================
            # 3.3 INYECCIÓN DE EVASIÓN (REPROGRAMACIÓN DEL PID)
            # =======================================================
            
            # Solo evadimos si vemos un obstáculo y NO estamos en plena curva peleando con la pared
            if obstaculo_tipo != "NINGUNO" and not es_vertice_curva:
                
                DISTANCIA_EVASION = 110 # Aumentamos la fuerza del desvío para asegurar
                
                if obstaculo_tipo == "ROJO":
                    # Regla WRO: Pasar por la DERECHA del bloque rojo
                    centro_pista_x = obstaculo_cx + DISTANCIA_EVASION
                    estado = "EVADIENDO_ROJO"
                    print(f"[EVASIÓN] Bloque ROJO detectado en {obstaculo_cx}. Forzando centro a {centro_pista_x}")
                    
                elif obstaculo_tipo == "VERDE":
                    # Regla WRO: Pasar por la IZQUIERDA del bloque verde
                    centro_pista_x = obstaculo_cx - DISTANCIA_EVASION
                    estado = "EVADIENDO_VERDE"
                    print(f"[EVASIÓN] Bloque VERDE detectado en {obstaculo_cx}. Forzando centro a {centro_pista_x}")
                    
                # ESCUDO ANTI-MURO: Evita chocar contra los bordes de la pista al esquivar
                if muro_der != -1:
                    centro_pista_x = min(centro_pista_x, muro_der - 45)
                if muro_izq != -1:
                    centro_pista_x = max(centro_pista_x, muro_izq + 45)
            
            if self.SENTIDO_GIRO == "AUTO" and es_vertice_curva:
                # Contamos matemáticamente los píxeles blancos (tapete libre) de cada lado
                blancos_izq = np.sum(binarizada[:, :160] == 255)
                blancos_der = np.sum(binarizada[:, 160:] == 255)
                
                # El lado con más píxeles blancos es el lado hacia donde se abre la pista
                if blancos_der > blancos_izq:
                    self.SENTIDO_GIRO = "DERECHA"
                    print("\n[!!!] ESPACIO LIBRE A LA DERECHA -> HORARIO CONFIRMADO [!!!]\n")
                else:
                    self.SENTIDO_GIRO = "IZQUIERDA"
                    print("\n[!!!] ESPACIO LIBRE A LA IZQ -> ANTIHORARIO CONFIRMADO [!!!]\n")
                    
            # =======================================================
            # 3.6 CONTEO DE VUELTAS (Reglamento WRO)
            # =======================================================
            # Consideramos que entró a una curva si perdió un muro o chocó de frente
            if es_vertice_curva:
                # El candado cronometrado: Deben haber pasado al menos 3.8 segundos desde la última esquina
                if not self.en_curva and (current_time - self.ultimo_tiempo_curva > 2.5):
                    self.en_curva = True
                    self.ultimo_tiempo_curva = current_time# Ponemos el candado
                    self.curvas_superadas += 1
                    
                    # Si ya pasó 4 esquinas, es 1 vuelta completa
                    if self.curvas_superadas % 4 == 0:
                        self.vueltas_completadas += 1
                        print(f"\n[] VUELTA {self.vueltas_completadas}/3 COMPLETADA []\n")
                        
                        if self.vueltas_completadas >= 3:
                            print("\n[] RETO SUPERADO: DETENIENDO MOTORES []\n")
                            self.current_speed = 0
                            self.current_angle = 86
                            self.running = False # Detiene el bucle principal
                            
            elif not es_vertice_curva and (current_time - self.ultimo_tiempo_curva > 2.5):
                # Al volver a la recta y ver ambos muros, quitamos el candado 3.8
                self.en_curva = False
                
            # =======================================================
            # 4. DIRECCIÓN Y VELOCIDAD (CON ZONA MUERTA)
            # =======================================================
            if estado == "MURO_FRONTAL":
                # Evadir a toda costa la colisión frontal
                #anadir orientacion dependiendo del sentido
                    
                if self.SENTIDO_GIRO == "DERECHA":
                    self.current_angle = 73
                else:
                    self.current_angle = 103
                self.current_speed = 220
                
            else:
                error_absoluto_real = 160 - centro_pista_x
                
                # Zona Muerta Generosa (15 píxeles)
                if abs(error_absoluto_real) < 150: #medir esta tolerancia
                    self.current_angle = 86
                    self.pid.integral = 0
                else:
                    if "EVADIENDO" in estado:
                        self.pid.kp = 0.12
                        self.current_speed = 190 # Frenamos un poco para maniobrar seguro
                    else:
                        self.pid.kp = 0.08 if estado == "CENTRADO" else 0.15
                        self.current_speed = 250
                    
                    correccion_pid = self.pid.compute(centro_pista_x, dt)
                    angulo_pid = int(86 + correccion_pid)
                    
                    # --- LA MAGIA DUAL-RATE ---
                    if estado == "CENTRADO":
                        # En recta: Topes físicos virtuales (Solo 10 grados de libertad)
                        # Esto destruye el zig-zag inmediatamente.
                        self.current_angle = max(76, min(96, angulo_pid))
                    else:
                        # Si perdió un muro, está entrando a curva: Liberamos el volante
                        self.current_angle = max(60, min(120, angulo_pid))
                
                # Velocidad
                if estado == "CENTRADO" and abs(error_absoluto_real) < 20:
                    self.current_speed = 250
                else:
                    self.current_speed = 250

            # Debug visual en consola
            if int(current_time * 10) % 5 == 0:
                print(f"| Estado: {estado: <15} | Muros: [I:{muro_izq} D:{muro_der}] | Vel: {self.current_speed} | Ángulo: {self.current_angle}° |")

            time.sleep(0.01)
        cap.release()
        
    def main_loop(self):
        vt = threading.Thread(target=self.process_vision)
        vt.start()

        try:
            while self.running:
                paquete = f"<{self.current_speed},{self.current_angle}>\n"
                self.ser.write(paquete.encode('utf-8'))
                time.sleep(0.05) 
        except KeyboardInterrupt:
            self.running = False
        finally:
            self.ser.write(b"<0,86>\n") 
            vt.join()
            self.ser.close()

if __name__ == "__main__":
    bot = WROAutonomousCar(serial_port='/dev/ttyUSB0') # Verifica el puerto
    bot.main_loop()