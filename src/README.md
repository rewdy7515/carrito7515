# Explanation of `MainCode.py`

This document explains how the `MainCode.py` file works. It runs on the Raspberry Pi and is responsible for the main decision-making of the autonomous car.

The program observes the track with OpenCV, calculates the driving direction, and sends a serial command to the Arduino with the speed and steering angle.

## General file structure

The file has two main components:

- **`PIDController`**: corrects the steering angle.
- **`WROAutonomousCar`**: contains wall-based vision, states, lap counting, and serial communication.

Base fragment:

```python
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=160):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0


class WROAutonomousCar:
    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
```

## 1. PID control

The PID takes a target position and a current position. The difference between them produces a correction.

- **P**: corrects proportionally to the current error.
- **I**: accumulates error over time.
- **D**: reacts to the change in error.

In this project, the target value is the horizontal center of the camera frame (`x = 160`), which represents the desired visual position of the track center.

```python
def compute(self, current_value, dt):
    error = self.setpoint - current_value
    self.integral += error * dt
    derivative = (error - self.prev_error) / dt if dt > 0 else 0
    self.prev_error = error
    return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
```

If the detected center moves away from `setpoint=160`, the PID returns a positive or negative correction to turn the servo.

## 2. Robot initialization

At startup, the program configures:

- the serial port connected to the Arduino,
- the turning direction in `AUTO` mode,
- corner and lap counters,
- the PID controller,
- the initial speed and steering angle.

```python
self.SENTIDO_GIRO = "AUTO"
self.memoria_muro_exterior = "NINGUNO"
self.MITAD_ANCHO_PISTA_PX = 140

self.vueltas_completadas = 0
self.curvas_superadas = 0
self.en_curva = False

self.pid = PIDController(kp=0.06, ki=0.000, kd=0.20)
self.running = True
self.current_speed = 0
self.current_angle = 86
```

## 3. Image capture and preprocessing

First, the camera is opened and the width and height are set. Then, for each frame:

1. it is converted to grayscale,
2. Gaussian blur is applied,
3. a relevant strip is cropped,
4. the image is converted to black and white.

```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (7, 7), 0)

y_arriba = 80
y_abajo = 140
roi_blur = blur[y_arriba:y_abajo, 0:320]

_, binarizada = cv2.threshold(
    roi_blur, 95, 255, cv2.THRESH_BINARY)
```

The `roi_blur` variable is essential. The code analyzes the specific part of the track used for fast decision-making instead of processing the entire image.

## 4. Raycasting and wall detection

Here, raycasting consists of taking a horizontal line from the binary image and scanning left and right from the center until the walls are found.

With those two points, the program estimates the center of the track.

```python
alto_roi, ancho_roi = binarizada.shape
linea_escaneo = binarizada[alto_roi // 2, :]

muro_izq = -1
muro_der = -1

for x in range(160, -1, -1):
    if linea_escaneo[x] == 0:
        muro_izq = x
        break

for x in range(160, ancho_roi):
    if linea_escaneo[x] == 0:
        muro_der = x
        break
```

## 5. State machine

Main states:

| State | Meaning | How it is produced |
|---|---|---|
| `CENTRADO` | Both walls are detected. | The center is calculated as the average of the left and right walls. |
| `MURO_IZQ` | Only the left wall is visible. | The center is estimated using the expected half-width of the track. |
| `MURO_DER` | Only the right wall is visible. | The center is estimated by shifting to the left. |
| `CEGUERA_BLANCA` | No wall is detected. | The scan line has no references. |
| `MURO_FRONTAL` | The center is blocked. | The center pixel of the scan line is black. |

```python
if linea_escaneo[160] == 0:
    estado = "MURO_FRONTAL"
else:
    if muro_izq != -1 and muro_der != -1:
        centro_pista_x = (muro_izq + muro_der) // 2
        estado = "CENTRADO"
    elif muro_izq != -1 and muro_der == -1:
        centro_pista_x = muro_izq + self.MITAD_ANCHO_PISTA_PX
        estado = "MURO_IZQ"
    elif muro_izq == -1 and muro_der != -1:
        centro_pista_x = muro_der - self.MITAD_ANCHO_PISTA_PX
        estado = "MURO_DER"
```

## 6. Automatic turning-direction detection

If the robot finds a frontal wall and still does not know whether the track is clockwise or counterclockwise, it compares the white free space on the left and right sides.

```python
if self.SENTIDO_GIRO == "AUTO" and es_vertice_curva:
    blancos_izq = np.sum(binarizada[:, :160] == 255)
    blancos_der = np.sum(binarizada[:, 160:] == 255)

    if blancos_der > blancos_izq:
        self.SENTIDO_GIRO = "DERECHA"
    else:
        self.SENTIDO_GIRO = "IZQUIERDA"
```

That logic is used to decide which side to choose for the blind turn during a frontal collision.

## 7. Corner and lap counting

The algorithm uses the appearance of a strong corner or frontal wall to count corners. To avoid counting the same corner multiple times, it applies a time-based lockout window.

- Every 4 corners = 1 lap.
- After 3 laps, the robot stops.

```python
if es_vertice_curva:
    if not self.en_curva and (current_time - self.ultimo_tiempo_curva > 2.5):
        self.en_curva = True
        self.ultimo_tiempo_curva = current_time
        self.curvas_superadas += 1

        if self.curvas_superadas % 4 == 0:
            self.vueltas_completadas += 1

            if self.vueltas_completadas >= 3:
                self.current_speed = 0
                self.current_angle = 86
                self.running = False
```

## 8. Steering and speed calculation

Main rules:

- If there is `MURO_FRONTAL`, the car turns with a fixed angle.
- If the error is small, it stays straight.
- On straight sections, the steering range is tighter to reduce zig-zag.
- In curves or when one wall is lost, the steering is allowed more freedom.

```python
if estado == "MURO_FRONTAL":
    if self.SENTIDO_GIRO == "DERECHA":
        self.current_angle = 73
    else:
        self.current_angle = 103
    self.current_speed = 220

else:
    error_absoluto_real = 160 - centro_pista_x

    if abs(error_absoluto_real) < 150:
        self.current_angle = 86
        self.pid.integral = 0
    else:
        correccion_pid = self.pid.compute(centro_pista_x, dt)
        angulo_pid = int(86 + correccion_pid)
```

This dead-zone condition defines when the car keeps the steering centered before applying PID correction.

## 9. Serial communication with Arduino

The main thread does not process vision. It takes the latest calculated command and packs it into a string:

```text
<speed,angle>
```

That packet is read by the Arduino to move the motor and servo.

```python
while self.running:
    paquete = f"<{self.current_speed},{self.current_angle}>\n"
    self.ser.write(paquete.encode('utf-8'))
    time.sleep(0.05)
```

Example:

- `<250,86>` means high speed with centered steering.

## 10. Visual summary of the complete flow

| Step | Action | Result |
|---|---|---|
| 1 | Frame capture | Current image of the track |
| 2 | Preprocessing | Binary image that is easier to analyze |
| 3 | Raycasting | Left and right walls |
| 4 | State machine | Decision about the current track context |
| 5 | PID + rules | Desired angle and speed |
| 6 | Serial | Command sent to the Arduino |

## Conclusion

`MainCode.py` works as the high-level brain of the car. The Raspberry Pi interprets the track and generates the decision, and the Arduino executes the received movement.

The most important part of the design is that it combines three layers:

- computer vision,
- state logic to understand the track,
- PID control to smooth steering.

The file integrates wall perception, decision-making, and control in the same driving loop.
