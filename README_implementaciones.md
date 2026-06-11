# Technical Documentation of the Implemented System

## 1. General Description

This document presents the implemented navigation system of an autonomous robot designed for the **WRO Future Engineers** competition, based on the following files:

- `src/MainCode.py`
- `src/Ino Code/Arduino_Code.ino`

According to the WRO Future Engineers 2026 rules, the vehicle operates in a self-driving car challenge in which it must drive autonomously on a track whose configuration varies between rounds. The official challenge includes Open Challenge rounds and Obstacle Challenge rounds, both based on autonomous track navigation.

The solution is distributed between a **Raspberry Pi**, responsible for vision processing and decision-making, and an **Arduino**, responsible for executing physical actions on the steering, traction, and ultrasonic sensing system.

This documentation remains aligned with the code currently available in the repository and describes the implemented logic only.

---

## 2. System Objective

The objective of the system is to allow the robot to:

- observe the track through a camera,
- detect walls and obstacles visually,
- adapt to a round direction that may be clockwise or counterclockwise,
- complete the required laps on the track autonomously,
- interpret red and green field references during obstacle navigation,
- determine a navigation state,
- calculate steering corrections using PID control,
- generate speed and steering-angle commands,
- and execute those commands through the Arduino.

In the WRO 2026 game description, the Open Challenge requires the vehicle to complete three laps on the track, while the Obstacle Challenge requires the vehicle to complete three laps while respecting the side indicated by red and green traffic signs. In the implemented system, these tasks are addressed through computer vision, state-based decision logic, PID control, serial communication, and ultrasonic-based mode selection.

---

## 3. Robot Architecture

### 3.1 Functional Distribution

| Module | File | Main Function |
|---|---|---|
| Raspberry Pi | `src/MainCode.py` | Vision processing, navigation states, PID control, mode selection, and serial command transmission |
| Arduino | `src/Ino Code/Arduino_Code.ino` | Command reception, servo control, motor control, ultrasonic sensing, and telemetry transmission |
| Camera | Accessed through OpenCV | Track image acquisition |
| Ultrasonic sensor | Processed by Arduino | Distance measurement used for telemetry and initial mode selection |

### 3.2 Physical System Flow

```text
Camera -> Raspberry Pi -> Serial -> Arduino -> Servo / Motor
                                     |
                                     -> Ultrasonic sensor -> Serial telemetry
```

---

## 4. System Structure by Operating Phase

For documentation purposes, the implemented system can be read in three main phases:

1. initialization,
2. Mode 1,
3. Mode 2.

This organization reflects the way the robot is prepared at startup and then operated according to the mode selected from the initial ultrasonic reading.

### 4.1 Initialization

During initialization, the system:

- opens serial communication with the Arduino,
- initializes the control variables,
- starts the serial-reading thread,
- receives ultrasonic telemetry,
- classifies the operating mode,
- and prepares the vision and control loop.

This phase defines whether the robot will begin operation in `Mode 1` or `Mode 2`.

### 4.2 Mode 1

Mode 1 corresponds to the speed-oriented operating condition:

- `modo_obstaculos = False`
- selected when `distancia_us >= 15`

In this mode, the robot prioritizes track-following through wall detection, orientation detection, lap counting, and PID-based steering corrections, while using the speed-mode turning profile defined in the control logic.

### 4.3 Mode 2

Mode 2 corresponds to the obstacle-oriented operating condition:

- `modo_obstaculos = True`
- selected when `distancia_us < 15`

In this mode, the system maintains the same base navigation architecture but applies the obstacle-mode turning profile and integrates the obstacle-handling states used during red and green sign interpretation.

Across both modes, the operating loop follows the same general cycle:

1. the camera provides a frame to the Raspberry Pi,
2. `MainCode.py` processes the image,
3. the system binarizes the track and detects visual references,
4. a navigation state is determined,
5. an estimated track center is calculated,
6. steering and speed logic is applied,
7. a serial packet `<speed,angle>` is generated,
8. the Arduino receives the packet and drives the servo and motor,
9. the Arduino measures the ultrasonic sensor and transmits telemetry as `US:distance`.

This cycle is repeated continuously during robot operation.

---

## 5. Python Implementation

This section is organized around the same operational sequence used by the robot:

- initialization,
- Mode 1,
- Mode 2,
- and the shared perception and communication layers that support both modes.

### 5.1 Initialization Stage in `MainCode.py`

`MainCode.py` contains the main navigation logic of the robot. Its function is to convert visual information from the track into movement commands.

#### Main Responsibilities During Initialization and Operation

- capture video,
- process image data,
- detect walls,
- detect obstacles,
- infer the effective driving direction of the round,
- execute the navigation state machine,
- calculate PID correction,
- count corners and laps,
- read ultrasonic telemetry from the Arduino,
- select the operating mode,
- send speed and steering commands through serial communication.

#### Imported Libraries

```python
import cv2
import os
import numpy as np
import serial
import time
import threading
```

| Library | Purpose |
|---|---|
| `cv2` | Image processing with OpenCV |
| `numpy` | Matrix operations and pixel counting |
| `serial` | Communication with the Arduino |
| `time` | Interval measurement and temporal control |
| `threading` | Concurrent execution of vision and serial reading |
| `os` | Auxiliary import currently present in the file |

#### Relationship with Hardware

`MainCode.py` interacts directly with:

- the USB camera through OpenCV,
- the Arduino through a serial port.

---

### 5.2 Shared PID Module

The `PIDController` class implements steering correction.

```python
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
```

#### Purpose

Convert the difference between the desired visual center and the detected center into an angular correction for the steering servo.

#### Main Variables

| Variable | Description |
|---|---|
| `kp` | Proportional gain |
| `ki` | Integral gain |
| `kd` | Derivative gain |
| `setpoint` | Target image center |
| `prev_error` | Previous error value |
| `integral` | Accumulated error |

#### Input and Output

| Element | Type | Description |
|---|---|---|
| `current_value` | numeric | Detected track center |
| `dt` | numeric | Time interval between iterations |
| output | numeric | Steering correction |

#### Relationship with Hardware

The PID output does not drive the servo directly. It determines the steering angle that is then sent to the Arduino for physical execution.

#### Implemented Safeguard

The derivative term prevents division by zero:

```python
derivative = (error - self.prev_error) / dt if dt > 0 else 0
```

---

### 5.3 Initialization of `WROAutonomousCar`

`WROAutonomousCar` represents the full logical unit of the autonomous vehicle.

#### Constructor

```python
class WROAutonomousCar:
    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
        time.sleep(2)
```

#### Program Instantiation

```python
if __name__ == "__main__":
    bot = WROAutonomousCar(serial_port='/dev/ttyUSB0')
    bot.main_loop()
```

#### Purpose

Integrate:

- robot configuration,
- vision processing,
- state-based navigation,
- lap counting,
- serial reading from Arduino,
- and command transmission.

#### Configuration Variables

```python
self.SENTIDO_GIRO = "AUTO"
self.memoria_muro_exterior = "NINGUNO"
self.MITAD_ANCHO_PISTA_PX = 140

self.vueltas_completadas = 0
self.curvas_superadas = 0
self.en_curva = False

self.ultimo_tiempo_curva = time.time()
self.pid = PIDController(kp=0.06, ki=0.000, kd=0.20)
self.running = True

self.centro_suavizado = 160.0
self.current_speed = 0
self.current_angle = 86

self.distancia_us = 200
self.modo_obstaculos = False
self.start_time = time.time()
```

#### Main Variables

| Variable | Description |
|---|---|
| `SENTIDO_GIRO` | Global track orientation |
| `MITAD_ANCHO_PISTA_PX` | Geometric estimate of half track width |
| `vueltas_completadas` | Lap counter |
| `curvas_superadas` | Corner counter |
| `en_curva` | Corner lockout state |
| `current_speed` | Last calculated speed |
| `current_angle` | Last calculated steering angle |
| `distancia_us` | Last ultrasonic distance received from Arduino |
| `modo_obstaculos` | Operating mode selected from ultrasonic reading |
| `start_time` | Reference time used by orientation detection |

#### Relationship with Hardware

- `serial_port` defines the physical link to the Arduino.
- `current_speed` and `current_angle` are the values that ultimately act on motor and servo.
- `distancia_us` stores telemetry received from the ultrasonic sensor via Arduino.

---

### 5.4 Initialization Telemetry Reading from Arduino

The file includes a dedicated serial-reading thread:

```python
def read_serial_data(self):
    while self.running:
        try:
            if self.ser.in_waiting > 0:
                linea = self.ser.readline().decode('utf-8').strip()
                if linea.startswith("US:"):
                    self.distancia_us = int(linea.split(":")[1])
        except:
            pass
        time.sleep(0.01)
```

#### Purpose

Read telemetry lines sent by the Arduino and update the latest ultrasonic distance.

#### Output

This routine updates:

- `self.distancia_us`

#### Relationship with Hardware

This block depends on the serial output generated by the Arduino ultrasonic subsystem.

---

### 5.5 Shared Video Capture for Both Modes

Video capture is performed inside `process_vision()`:

```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
```

#### Purpose

Acquire real-time images to feed the navigation logic.

#### Implemented Parameters

| Parameter | Current Value | Purpose |
|---|---:|---|
| Camera index | `0` | Main system camera |
| Width | `320` | Horizontal resolution |
| Height | `240` | Vertical resolution |

#### Capture Cycle

```python
ret, frame = cap.read()
if not ret:
    continue

current_time = time.time()
dt = current_time - last_time
last_time = current_time
```

#### Output

Each iteration produces:

- `frame`: current image,
- `dt`: time interval between frames.

#### Relationship with Hardware

This block depends directly on the camera connected to the Raspberry Pi.

---

### 5.6 Shared Wall-Detection Pipeline

Track detection is based on a specific horizontal strip of the image.

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (7, 7), 0)

y_arriba = 80
y_abajo = 140
roi_blur = blur[y_arriba:y_abajo, 0:320]

_, binarizada = cv2.threshold(
    roi_blur, 95, 255, cv2.THRESH_BINARY)
```

#### Purpose

Reduce the visual scene to a simple binary representation in which the track and the walls can be distinguished quickly.

#### Implemented Stages

| Stage | Purpose |
|---|---|
| Grayscale conversion | Simplify visual information |
| Gaussian blur | Reduce local noise |
| Region of interest | Concentrate analysis on the useful strip |
| Fixed threshold | Separate floor and walls |

#### Binary Convention

| Value | Interpretation |
|---:|---|
| `255` | Clear floor / free space |
| `0` | Dark wall / track boundary |

#### Binary Decision Logic

Navigation starts from a binary reading over the analyzed strip:

- `255` represents free space,
- `0` represents wall.

The system makes decisions based on this binary image rather than on the original RGB image.

```text
Original image -> grayscale -> blur -> fixed threshold -> binary image
```

The basic questions solved at this stage are:

- Is there a wall at the center?
- Is there a wall to the left of the center?
- Is there a wall to the right of the center?

#### Relationship with Hardware

This block processes the image supplied directly by the camera.

---

### 5.7 Mode 2 Obstacle Detection and Evasion

The implementation includes a color-based obstacle-detection stage.

```python
roi_color = frame[60:240, 0:320]
hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
```

#### Purpose

Detect green and red obstacles inside a region of interest compatible with the navigation geometry. Under the WRO obstacle rules, the red pillar indicates that the vehicle must pass on the right side of the sign, and the green pillar indicates that the vehicle must pass on the left side.

#### Additional Preprocessing

The implemented version applies morphological filtering with:

```python
kernel = np.ones((5, 5), np.uint8)
```

and then:

```python
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
```

#### Implemented Color Ranges

```python
lower_green = np.array([35, 60, 50])
upper_green = np.array([85, 255, 255])
mask_green = cv2.inRange(hsv, lower_green, upper_green)

lower_red1 = np.array([0, 100, 80])
upper_red1 = np.array([5, 255, 255])
lower_red2 = np.array([175, 100, 80])
upper_red2 = np.array([180, 255, 255])
mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
```

#### Detection Logic

The system:

1. builds color masks,
2. extracts contours,
3. filters by area and width,
4. stores detected obstacles in `lista_obstaculos`,
5. identifies the most relevant obstacle,
6. assigns `ROJO` or `VERDE`.

Relevant condition:

```python
if area > 400 and w < 260 and area > area_max_obs:
```

#### Obstacle Memory and Evasion Variables

The implemented logic defines and evaluates:

- `tiempo_ultimo_obstaculo`,
- `memoria_tipo_obstaculo`,
- `tiempo_ciego`,
- `en_memoria_evasion`,
- `tipo_evasion_activa`,
- `evadiendo`.

This logic is used to derive obstacle-related states such as:

- `EVADIENDO_ROJO`
- `EVADIENDO_VERDE`
- `MEMORIA_ROJO`
- `MEMORIA_VERDE`

#### Invisibility Layer

The implementation also modifies the binary image before raycasting:

```python
for obs in lista_obstaculos:
    cx_temp = obs["x"] + (obs["w"] // 2)
    x_inicio = max(0, obstaculo_cx - 20)
    x_fin = min(320, obstaculo_cx + 20)
    binarizada[:, x_inicio:x_fin] = 255
```

#### Output

This stage produces:

- `obstaculo_tipo`,
- `obstaculo_cx`,
- `lista_obstaculos`,
- evasion-related states and variables.

#### Relationship with Hardware

This detection depends directly on the camera and on the visual appearance of the track environment.

---

### 5.8 Shared Raycasting for Track Estimation

After binarization, the system analyzes the center line of the region of interest.

```python
alto_roi, ancho_roi = binarizada.shape
linea_escaneo = binarizada[alto_roi // 2, :]
```

#### Purpose

Find lateral track references and estimate the navigation center.

#### Implemented Scan

```python
for x in range(160, -1, -1):
    if linea_escaneo[x] == 0:
        muro_izq = x
        break

for x in range(160, ancho_roi):
    if linea_escaneo[x] == 0:
        muro_der = x
        break
```

#### Central Scan Logic

The raycasting sequence is:

1. take the center row of the binary strip,
2. inspect the center pixel (`x = 160`),
3. if that pixel is black, classify the situation as `MURO_FRONTAL`,
4. if the center is free, search for the first wall on the left,
5. then search for the first wall on the right,
6. estimate the track center from that combination.

#### Logical Result

```text
Blocked center         -> MURO_FRONTAL
Left and right walls   -> CENTRADO
Only left wall         -> MURO_IZQ
Only right wall        -> MURO_DER
None                   -> CEGUERA_BLANCA
```

#### Output

The raycasting stage produces:

- `muro_izq`,
- `muro_der`,
- `centro_pista_x`,
- `estado`.

---

### 5.9 Shared State Machine

The implementation uses the following states:

| State | Meaning |
|---|---|
| `CENTRADO` | Both walls are detected |
| `MURO_IZQ` | Only the left wall is detected |
| `MURO_DER` | Only the right wall is detected |
| `MURO_FRONTAL` | The center of the track is blocked |
| `CEGUERA_BLANCA` | No wall is detected |
| `EVADIENDO_ROJO` | The trajectory is forced by a red obstacle |
| `EVADIENDO_VERDE` | The trajectory is forced by a green obstacle |
| `MEMORIA_ROJO` | Memory-based red obstacle handling |
| `MEMORIA_VERDE` | Memory-based green obstacle handling |

#### Main Transition Logic

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
    else:
        estado = "CEGUERA_BLANCA"
```

#### State-Based Decision Logic

The state machine transforms binary observations into navigation decisions.

It first obtains:

- wall present or absent at the center,
- wall present or absent on the left,
- wall present or absent on the right.

Then it defines a state, and that state determines:

- the estimated track center,
- PID tuning behavior,
- steering limits,
- speed output,
- corner counting conditions.

```text
Level 1: binary perception of walls and obstacles
Level 2: state-based navigation decision
```

---

### 5.10 Shared Automatic Turning-Direction Detection

When the system enters `MURO_FRONTAL` and `SENTIDO_GIRO` is still `AUTO`, it evaluates the free space on both sides. In the implemented logic, the comparison uses the upper part of the binary image:

```python
if self.SENTIDO_GIRO == "AUTO" and es_vertice_curva and (current_time - self.start_time > 1.5):
    horizonte = binarizada[:30, :]

    blancos_izq = np.sum(horizonte[:, :160] == 255)
    blancos_der = np.sum(horizonte[:, 160:] == 255)

    if blancos_der > blancos_izq:
        self.SENTIDO_GIRO = "DERECHA"
    else:
        self.SENTIDO_GIRO = "IZQUIERDA"
```

#### Orientation Logic

| Condition | Result |
|---|---|
| More white pixels on the right | `DERECHA` |
| More white pixels on the left | `IZQUIERDA` |

#### Purpose

Define a global orientation used when resolving frontal collisions. This is consistent with the WRO round condition in which the driving direction may vary between clockwise and counterclockwise configurations.

---

### 5.11 Shared Corner and Lap Counting

Corner detection starts from:

```python
perdio_muro_interior = (self.SENTIDO_GIRO == "DERECHA" and muro_der == -1) or \
                       (self.SENTIDO_GIRO == "IZQUIERDA" and muro_izq == -1)
es_vertice_curva = estado == "MURO_FRONTAL" or perdio_muro_interior
```

Then a temporal lockout is applied:

```python
if not self.en_curva and (current_time - self.ultimo_tiempo_curva > 0.2):
```

#### Counting Rule

| Event | Result |
|---|---|
| 1 valid corner | `curvas_superadas += 1` |
| 4 valid corners | `vueltas_completadas += 1` |
| 3 laps | system stop |

This counting logic is consistent with the WRO challenge structure in which the vehicle is required to complete three laps autonomously.

#### Control Fragment

```python
if self.curvas_superadas % 4 == 0:
    self.vueltas_completadas += 1

    if self.vueltas_completadas >= 3:
        self.current_speed = 0
        self.current_angle = 86
        self.running = False
```

#### Temporal Logic

Corner counting depends on:

- a visual condition (`MURO_FRONTAL` or loss of the inner wall),
- a temporal condition (`0.2 s` lockout),
- an accumulated corner counter.

The reset condition uses:

```python
elif not es_vertice_curva and (current_time - self.ultimo_tiempo_curva > 0.5):
    self.en_curva = False
```

---

### 5.12 Mode-Dependent Steering and Speed Calculation

The implemented control separates two main scenarios.

#### Mode Summary

| Mode | Internal Condition | Main Control Profile |
|---|---|---|
| Mode 1 | `modo_obstaculos = False` | Speed-oriented wall-following and standard frontal turning profile |
| Mode 2 | `modo_obstaculos = True` | Obstacle-oriented wall-following and tighter frontal turning profile |

#### Case 1: Frontal wall without visible obstacle

```python
if estado == "MURO_FRONTAL" and obstaculo_tipo == "NINGUNO":
    if self.SENTIDO_GIRO == "DERECHA":
        self.current_angle = 60 if self.modo_obstaculos else 73
    else:
        self.current_angle = 115 if self.modo_obstaculos else 103
    self.current_speed = 220
```

#### Case 2: Normal navigation

```python
error_absoluto_real = 160 - centro_pista_x

zona_muerta = 45 if ("EVADIENDO" in estado or "MEMORIA" in estado) else 125

if abs(error_absoluto_real) < zona_muerta:
    self.current_angle = 86
    self.pid.integral = 0
else:
    if evadiendo:
        self.pid.kp = 0.15
        self.current_speed = 200
    else:
        self.pid.kp = 0.08 if estado == "CENTRADO" else 0.15
        self.current_speed = 250
```

#### PID Correction and Limits

```python
correccion_pid = self.pid.compute(centro_pista_x, dt)
angulo_pid = int(86 + correccion_pid)

if evadiendo:
    self.current_angle = max(60, min(120, angulo_pid))
elif estado == "CENTRADO":
    self.current_angle = max(76, min(96, angulo_pid))
else:
    self.current_angle = max(65, min(120, angulo_pid))
```

#### Final Speed in Normal Navigation

```python
if estado == "CENTRADO" and abs(error_absoluto_real) < 20:
    self.current_speed = 250
else:
    self.current_speed = 250
```

#### Control Logic

1. determine the current state,
2. calculate the track-center error,
3. evaluate the dead zone,
4. if required, apply PID,
5. limit the steering angle according to the state,
6. update `current_speed` and `current_angle`.

#### Binary Control Rule

| Condition | Action |
|---|---|
| Error inside dead zone | centered steering |
| Error outside dead zone | PID correction |

#### Relationship with Hardware

The result of this block translates directly into:

- a steering angle for the servo,
- a speed value for the motor.

---

### 5.13 Initialization-Based Operating Mode Selection

Before starting the main vision thread, the implementation reads ultrasonic telemetry for two seconds and selects an operating mode:

```python
rt = threading.Thread(target=self.read_serial_data, daemon=True)
rt.start()

time.sleep(2)
if self.distancia_us < 15:
    self.modo_obstaculos = True
else:
    self.modo_obstaculos = False
```

#### Purpose

Choose between:

- `modo_obstaculos = True`
- `modo_obstaculos = False`

based on the initial ultrasonic reading.

#### Decision Rule

| Condition | Mode |
|---|---|
| `distancia_us < 15` | obstacle mode |
| `distancia_us >= 15` | speed mode |

#### Mode Mapping

| Internal Variable | Documentation Name | Operational Meaning |
|---|---|---|
| `modo_obstaculos = False` | Mode 1 | Speed-oriented track navigation |
| `modo_obstaculos = True` | Mode 2 | Obstacle-oriented track navigation |

#### Relationship with Hardware

This decision depends on ultrasonic telemetry generated by the Arduino.

---

### 5.14 Shared Serial Transmission to the Arduino

`main_loop()` maintains the continuous transmission of commands:

```python
paquete = f"<{self.current_speed},{self.current_angle}>\n"
self.ser.write(paquete.encode('utf-8'))
```

#### Output Format

```text
<speed,angle>
```

Example:

```text
<250,86>
```

#### Shutdown Command

```python
self.ser.write(b"<0,86>\n")
```

#### Relationship with Hardware

This serial link constitutes the direct interface between the Raspberry Pi and the Arduino.

---

## 6. Arduino Implementation

### 6.1 General Purpose of `Arduino_Code.ino`

The Arduino receives the commands sent by the Raspberry Pi and converts them into physical actions on:

- the steering servo,
- the traction motor,
- and the ultrasonic sensor.

---

### 6.2 Libraries and Main Definition

```cpp
#include <Servo.h>
#include <stdlib.h>

Servo direccion;
```

| Element | Purpose |
|---|---|
| `Servo.h` | Servo angle control |
| `stdlib.h` | String-conversion utilities |
| `Servo direccion` | Object that drives the physical servo |

---

### 6.3 Pin Assignment

```cpp
const int pinServo = 8;
const int pinMotorPWM = 7;
const int pinMotorDir1 = 9;
const int pinMotorDir2 = 10;
const int pinTrig = 3;
const int pinEcho = 11;
```

| Pin | Associated Component | Function |
|---:|---|---|
| `8` | Servo | Steering |
| `7` | Motor driver | Speed PWM |
| `9` | Motor driver | Motor direction, input 1 |
| `10` | Motor driver | Motor direction, input 2 |
| `3` | Ultrasonic sensor | Trigger |
| `11` | Ultrasonic sensor | Echo |

---

### 6.4 Control Variables

```cpp
int distanciaUS = 200;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

int velocidadAuto = 0;
int anguloServo = 86;
unsigned long previousMillisUS = 0;
```

| Variable | Purpose |
|---|---|
| `distanciaUS` | Latest ultrasonic distance |
| `receivedChars` | Serial reception buffer |
| `tempChars` | Temporary copy for parsing |
| `newData` | Complete-packet flag |
| `velocidadAuto` | Current motor speed |
| `anguloServo` | Current servo angle |
| `previousMillisUS` | Timing reference for ultrasonic reading |

---

### 6.5 Initialization

```cpp
void setup() {
    Serial.begin(115200);
    direccion.attach(pinServo);
    direccion.write(86);

    pinMode(pinMotorPWM, OUTPUT);
    pinMode(pinMotorDir1, OUTPUT);
    pinMode(pinMotorDir2, OUTPUT);
    pinMode(pinTrig, OUTPUT);
    pinMode(pinEcho, INPUT);
}
```

#### Purpose

Initialize communication, servo, motor, and ultrasonic sensor.

#### Relationship with Hardware

This block establishes the physical base state of the system before command reception begins.

---

### 6.6 Serial Command Reception

The implemented input protocol uses start and end markers:

- start: `<`
- end: `>`

Reception is performed through:

```cpp
recvWithStartEndMarkers();
```

Expected format:

```text
<speed,angle>
```

Example:

```text
<250,86>
```

#### Reception Logic

The Arduino:

1. waits for the `<` marker,
2. stores incoming characters,
3. closes the packet when `>` is received,
4. activates `newData = true`.

---

### 6.7 Command Parsing

```cpp
strtokIndx = strtok(tempChars, ",");
int velTemp = atoi(strtokIndx);
strtokIndx = strtok(NULL, ",");
int angTemp = atoi(strtokIndx);
```

#### Implemented Limits

```cpp
if (velTemp < 0) velTemp = 0;
if (velTemp > 255) velTemp = 255;

if (angTemp >= 60 && angTemp <= 120) {
    anguloServo = angTemp;
}
```

| Variable | Applied Range |
|---|---|
| Speed | `0..255` |
| Angle | `60..120` |

#### Output

Parsing updates:

- `velocidadAuto`,
- `anguloServo`.

---

### 6.8 Movement Execution

```cpp
void ejecutarMovimiento() {
    direccion.write(anguloServo);

    if (velocidadAuto > 0) {
        digitalWrite(pinMotorDir1, HIGH);
        digitalWrite(pinMotorDir2, LOW);
        analogWrite(pinMotorPWM, velocidadAuto);
    } else {
        digitalWrite(pinMotorDir1, LOW);
        digitalWrite(pinMotorDir2, LOW);
        analogWrite(pinMotorPWM, 0);
    }
}
```

#### Purpose

Apply physically the steering angle and speed received from the Raspberry Pi.

#### Current Behavior

| Condition | Result |
|---|---|
| `velocidadAuto > 0` | forward motor operation |
| `velocidadAuto == 0` | motor stopped |

#### Relationship with Hardware

This block directly controls:

- the steering servo,
- the motor driver,
- the traction motor.

---

### 6.9 Ultrasonic Reading and Telemetry

The Arduino measures distance every 50 ms:

```cpp
if (currentMillis - previousMillisUS >= 50) {
    previousMillisUS = currentMillis;
    leerUltrasonido();
    Serial.print("US:");
    Serial.println(distanciaUS);
}
```

Reading logic:

```cpp
long duration = pulseIn(pinEcho, HIGH, 12000);
if (duration == 0) {
    distanciaUS = 200;
} else {
    distanciaUS = duration * 0.034 / 2;
}
```

#### Output Format

```text
US:123
```

#### Purpose

Generate a periodic distance measurement and transmit it as serial telemetry.

#### Relationship with Hardware

This block depends directly on the ultrasonic sensor connected to the Arduino.

---

## 7. Serial Communication Protocol

The communication uses two formats.

### 7.1 Raspberry Pi -> Arduino

| Format | Purpose |
|---|---|
| `<speed,angle>` | Motion control |

Examples:

```text
<250,86>
<220,103>
<0,86>
```

### 7.2 Arduino -> Raspberry Pi / Serial Monitor

| Format | Purpose |
|---|---|
| `US:distance` | Ultrasonic telemetry |

Example:

```text
US:57
```

---

## 8. Navigation Logic

The navigation logic is aligned with the core operational requirements of the WRO Future Engineers track challenge: autonomous lane-following, adaptation to variable driving direction, multi-lap execution, and obstacle-side interpretation based on colored field elements.

The navigation system combines several decision layers.

### 8.1 Initialization Logic

The navigation process begins with an initialization phase in which the robot:

- establishes serial communication,
- receives the first ultrasonic readings,
- classifies the operating mode,
- and starts the vision-processing thread.

This initialization stage determines whether the operating profile will be `Mode 1` or `Mode 2`.

### 8.2 Binary Perception Logic

The track is transformed into a binary image:

- `255` = free track,
- `0` = wall.

Based on this representation, the system answers binary questions:

- Is there a wall at the center?
- Is there a wall on the left?
- Is there a wall on the right?

This perception stage is shared by both operating modes.

### 8.3 Mode 1 Logic

Mode 1 corresponds to the condition:

- `modo_obstaculos = False`

In this mode, the navigation profile is centered on:

- wall-based track following,
- raycasting-based center estimation,
- automatic round-direction inference,
- lap counting across three laps,
- and the speed-mode steering profile used during frontal-wall resolution.

#### Code Blocks Used in Mode 1

Mode 1 is built from the following code blocks inside `MainCode.py`.

##### 1. Mode 1 Activation Block

This block leaves the robot in speed mode when the ultrasonic condition does not activate obstacle mode:

```python
time.sleep(2)
if self.distancia_us < 15:
    self.modo_obstaculos = True
else:
    self.modo_obstaculos = False
    print("\n[MODO] MODO VELOCIDAD (MODO 1) ACTIVADO\n")
```

##### 2. Wall Image Processing Block

Mode 1 navigation starts from the wall-detection strip:

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (7, 7), 0)

y_arriba = 80
y_abajo = 140
roi_blur = blur[y_arriba:y_abajo, 0:320]

_, binarizada = cv2.threshold(
    roi_blur, 95, 255, cv2.THRESH_BINARY)
```

##### 3. Raycasting Block

The wall references used in Mode 1 are extracted with raycasting over the center line:

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

##### 4. State Estimation Block

The robot converts the wall scan into a navigation state:

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
    else:
        estado = "CEGUERA_BLANCA"
```

##### 5. Orientation Detection Block

When the round direction is still unknown, Mode 1 uses this block:

```python
if self.SENTIDO_GIRO == "AUTO" and es_vertice_curva and (current_time - self.start_time > 1.5):
    horizonte = binarizada[:30, :]

    blancos_izq = np.sum(horizonte[:, :160] == 255)
    blancos_der = np.sum(horizonte[:, 160:] == 255)

    if blancos_der > blancos_izq:
        self.SENTIDO_GIRO = "DERECHA"
    else:
        self.SENTIDO_GIRO = "IZQUIERDA"
```

##### 6. Corner and Lap Counting Block

Mode 1 keeps lap progress with the following logic:

```python
perdio_muro_interior = (self.SENTIDO_GIRO == "DERECHA" and muro_der == -1) or \
                       (self.SENTIDO_GIRO == "IZQUIERDA" and muro_izq == -1)
es_vertice_curva = estado == "MURO_FRONTAL" or perdio_muro_interior

if es_vertice_curva:
    if not self.en_curva and (current_time - self.ultimo_tiempo_curva > 0.2):
        self.en_curva = True
        self.ultimo_tiempo_curva = current_time
        self.curvas_superadas += 1

        if self.curvas_superadas % 4 == 0:
            self.vueltas_completadas += 1
```

##### 7. Frontal-Turn Block for Mode 1

Mode 1 frontal turning uses the speed-mode branch:

```python
if estado == "MURO_FRONTAL" and obstaculo_tipo == "NINGUNO":
    if self.SENTIDO_GIRO == "DERECHA":
        self.current_angle = 60 if self.modo_obstaculos else 73
    else:
        self.current_angle = 115 if self.modo_obstaculos else 103
    self.current_speed = 220
```

Because `modo_obstaculos = False`, this branch applies:

- `73` for right-turn frontal resolution,
- `103` for left-turn frontal resolution.

##### 8. PID Steering Block for Mode 1

Normal navigation in Mode 1 continues through PID-based correction:

```python
if abs(error_absoluto_real) < zona_muerta:
    self.current_angle = 86
    self.pid.integral = 0
else:
    self.pid.kp = 0.08 if estado == "CENTRADO" else 0.15
    self.current_speed = 250
    correccion_pid = self.pid.compute(centro_pista_x, dt)
```

##### 9. Command Output Block

The resulting Mode 1 command is sent to the Arduino through:

```python
paquete = f"<{self.current_speed},{self.current_angle}>\n"
self.ser.write(paquete.encode('utf-8'))
```

#### Implementation in Mode 1

Mode 1 is the operating profile used for speed-oriented track completion. In practice, the robot:

1. receives the initial ultrasonic reading,
2. remains in `modo_obstaculos = False`,
3. captures the track image,
4. binarizes the wall region,
5. applies raycasting to estimate the track center,
6. determines a navigation state,
7. infers the global direction of the round when required,
8. counts corners and laps,
9. computes steering with PID,
10. sends `<speed,angle>` commands to the Arduino.

This mode is the direct implementation of the wall-following and three-lap logic used for autonomous track traversal.

#### Hardware Related to Mode 1

| Hardware Element | Role in Mode 1 |
|---|---|
| Camera | Provides the wall image used for binarization and raycasting |
| Raspberry Pi | Executes wall detection, state logic, lap counting, and PID |
| Arduino | Receives motion commands and actuates the servo and motor |
| Steering servo | Applies the steering angle generated in `MainCode.py` |
| Traction motor | Executes the requested speed |
| Ultrasonic sensor | Selects the initial operating profile before the run |

#### Functional Behavior of Mode 1

Mode 1 behaves as a wall-guided autonomous driving module. Its main objective is to keep the vehicle centered with respect to the track boundaries, detect directional changes at corners, preserve progress over three laps, and resolve frontal-wall conditions with the standard turning profile associated with speed-oriented navigation.

### 8.4 Mode 2 Logic

Mode 2 corresponds to the condition:

- `modo_obstaculos = True`

In this mode, the controller preserves the shared wall-following structure but applies:

- the obstacle-mode frontal turning profile,
- obstacle-related states such as `EVADIENDO_ROJO` and `EVADIENDO_VERDE`,
- and the memory-based continuation states `MEMORIA_ROJO` and `MEMORIA_VERDE`.

#### Code Blocks Used in Mode 2

Mode 2 is built from the following code blocks inside `MainCode.py`.

##### 1. Mode 2 Activation Block

This block activates obstacle mode from the initial ultrasonic reading:

```python
time.sleep(2)
if self.distancia_us < 15:
    self.modo_obstaculos = True
    print("\n[MODO] MODO OBSTÁCULOS (MODO 2) ACTIVADO POR ULTRASONIDO\n")
else:
    self.modo_obstaculos = False
```

##### 2. Shared Wall-Detection Block

Mode 2 preserves the same wall-processing pipeline used in track following:

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (7, 7), 0)

y_arriba = 80
y_abajo = 140
roi_blur = blur[y_arriba:y_abajo, 0:320]

_, binarizada = cv2.threshold(
    roi_blur, 95, 255, cv2.THRESH_BINARY)
```

##### 3. Obstacle-Detection Block

Mode 2 adds the HSV segmentation used to identify red and green signs:

```python
roi_color = frame[60:240, 0:320]
hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)

lower_green = np.array([35, 60, 50])
upper_green = np.array([85, 255, 255])
mask_green = cv2.inRange(hsv, lower_green, upper_green)

lower_red1 = np.array([0, 100, 80])
upper_red1 = np.array([5, 255, 255])
lower_red2 = np.array([175, 100, 80])
upper_red2 = np.array([180, 255, 255])
mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
```

##### 4. Morphological Cleaning Block

The obstacle masks are cleaned with morphology:

```python
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
```

##### 5. Obstacle Selection Block

The most relevant obstacle is selected by contour area:

```python
for c in contornos_v:
    area = cv2.contourArea(c)
    x, y, w, h = cv2.boundingRect(c)
    if area > 400 and w < 260 and area > area_max_obs:
        lista_obstaculos.append(
            {"x": x, "w": w, "tipo": "VERDE", "area": area})
        if area > area_max_obs:
            area_max_obs = area
            obstaculo_cx = x + (w // 2)
            obstaculo_tipo = "VERDE"
```

##### 6. Obstacle Memory and Evasion Block

Obstacle interpretation and evasion are handled by this block:

```python
tiempo_ciego = current_time - tiempo_ultimo_obstaculo
en_memoria_evasion = tiempo_ciego < 1
tipo_evasion_activa = obstaculo_tipo if obstaculo_tipo != "NINGUNO" else memoria_tipo_obstaculo
evadiendo = (obstaculo_tipo != "NINGUNO" or en_memoria_evasion)

if evadiendo:
    if tipo_evasion_activa == "ROJO":
        centro_pista_x = 240
        estado = "EVADIENDO_ROJO" if obstaculo_tipo != "NINGUNO" else "MEMORIA_ROJO"

    elif tipo_evasion_activa == "VERDE":
        centro_pista_x = 40
        estado = "EVADIENDO_VERDE" if obstaculo_tipo != "NINGUNO" else "MEMORIA_VERDE"

    centro_pista_x = max(45, min(350, centro_pista_x))
```

##### 7. Frontal-Turn Block for Mode 2

Mode 2 frontal turning uses the obstacle-mode branch:

```python
if estado == "MURO_FRONTAL" and obstaculo_tipo == "NINGUNO":
    if self.SENTIDO_GIRO == "DERECHA":
        self.current_angle = 60 if self.modo_obstaculos else 73
    else:
        self.current_angle = 115 if self.modo_obstaculos else 103
    self.current_speed = 220
```

Because `modo_obstaculos = True`, this branch applies:

- `60` for right-turn frontal resolution,
- `115` for left-turn frontal resolution.

##### 8. Obstacle-Mode Control Block

During obstacle handling, the control profile is also adjusted:

```python
if evadiendo:
    self.pid.kp = 0.15
    self.current_speed = 200
```

##### 9. Command Output Block

The resulting Mode 2 command is also sent to the Arduino through:

```python
paquete = f"<{self.current_speed},{self.current_angle}>\n"
self.ser.write(paquete.encode('utf-8'))
```

#### Implementation in Mode 2

Mode 2 is the obstacle-oriented operating profile. In practice, the robot:

1. activates `modo_obstaculos = True` after the initial ultrasonic classification,
2. captures the scene with the same camera used in wall navigation,
3. detects red and green obstacles in HSV space,
4. cleans the masks with morphological operations,
5. selects the most relevant obstacle by contour area,
6. changes the desired center of the track according to obstacle color,
7. maintains a short memory state when the obstacle is no longer visible,
8. applies a more aggressive obstacle-handling steering profile,
9. sends the resulting motion command to the Arduino.

This implementation allows the vehicle to maintain the same base navigation architecture while adding directional interpretation of the WRO red and green field elements.

#### Hardware Related to Mode 2

| Hardware Element | Role in Mode 2 |
|---|---|
| Camera | Captures both walls and colored obstacles |
| Raspberry Pi | Executes HSV segmentation, evasion-state logic, and obstacle-aware steering |
| Arduino | Receives obstacle-mode motion commands and actuates the vehicle |
| Steering servo | Applies the sharper turning profile used in obstacle mode |
| Traction motor | Executes reduced-speed obstacle maneuvers and normal forward motion |
| Ultrasonic sensor | Activates the obstacle-oriented profile at startup |

#### Functional Behavior of Mode 2

Mode 2 behaves as an obstacle-aware autonomous driving module. It preserves wall-based navigation as the structural reference, but superimposes color-based obstacle interpretation to force the robot toward the correct side of red and green signs. The result is a combined behavior in which the robot follows the track, keeps lap continuity, and modifies its trajectory according to obstacle position and obstacle color.

### 8.5 State-Based Logic

Binary observations are converted into navigation states:

| State | Function |
|---|---|
| `CENTRADO` | Navigation with both walls visible |
| `MURO_IZQ` | Estimation using the left-side reference |
| `MURO_DER` | Estimation using the right-side reference |
| `MURO_FRONTAL` | Response to a blocked front |
| `CEGUERA_BLANCA` | No lateral reference detected |
| `EVADIENDO_ROJO` | Forced adjustment due to a red obstacle |
| `EVADIENDO_VERDE` | Forced adjustment due to a green obstacle |
| `MEMORIA_ROJO` | Memory-based red obstacle state |
| `MEMORIA_VERDE` | Memory-based green obstacle state |

These states allow the controller to transform the visual interpretation of the WRO field into discrete navigation actions.

### 8.6 Global Orientation Logic

When a frontal block exists and the orientation remains in `AUTO`, the system compares free white space on both sides and assigns:

- `DERECHA`
- `IZQUIERDA`

This mechanism supports challenge rounds in which the vehicle may be required to drive either clockwise or counterclockwise.

### 8.7 Temporal Logic

Corner counting uses:

- a visual signal (`MURO_FRONTAL` or inner-wall loss),
- a time window,
- an accumulated counter.

By converting corner detections into lap progress, the controller maintains the sequence required for a three-lap autonomous run.

### 8.8 Control Logic

The control process follows this order:

1. visual perception,
2. state classification,
3. error calculation,
4. dead-zone evaluation,
5. PID application,
6. steering-angle limiting,
7. serial transmission.

### 8.9 Mode Selection Logic

The implementation also includes an initial logic that classifies the operating mode from ultrasonic telemetry:

- obstacle mode,
- speed mode.

---

## 9. Relationship Between Software and Physical Components

| Physical Component | Associated Software | Function |
|---|---|---|
| Camera | `cv2.VideoCapture(0)` in `MainCode.py` | Image capture |
| Raspberry Pi | `MainCode.py` | Visual processing and decision-making |
| Arduino | `Arduino_Code.ino` | Physical command execution |
| Servo | `direccion.write(anguloServo)` | Steering |
| Motor / driver | `analogWrite(pinMotorPWM, velocidadAuto)` | Traction |
| Ultrasonic sensor | `leerUltrasonido()` and `read_serial_data()` | Distance measurement and telemetry |

The relationship between software and hardware is direct: the Raspberry Pi generates logical commands and the Arduino translates them into physical motion and sensor reporting.

---

## 10. Conclusion

The implemented system uses computer vision on the Raspberry Pi to interpret the track, detect walls and obstacles, classify navigation states, compute corrections through PID control, select an operating mode, and transmit motion commands to the Arduino.

The Arduino receives those commands, controls the servo and motor, and measures distance using the ultrasonic sensor to generate serial telemetry.

Taken together, the system combines:

- binary visual perception,
- raycasting over the central strip,
- a navigation state machine,
- global orientation based on free space,
- PID control,
- ultrasonic-assisted mode selection,
- and structured serial communication.

The result is a functional navigation architecture distributed between high-level processing on the Raspberry Pi and physical execution on the Arduino, consistent with the actual implementation documented in the repository and with the main operational demands of the WRO Future Engineers track challenge.
