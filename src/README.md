# Technical Documentation of the Implemented System

## Table of Contents

<!-- toc -->

- [1. General Description](#1-general-description)
- [2. System Objective](#2-system-objective)
- [3. Robot Architecture](#3-robot-architecture)
  - [3.1 Functional Distribution](#31-functional-distribution)
  - [3.2 Physical System Flow](#32-physical-system-flow)
  - [3.3 Software-to-Hardware Connection](#33-software-to-hardware-connection)
- [4. Software Implementation](#4-software-implementation)
  - [4.1 Mode 1 Implementation](#41-mode-1-implementation)
  - [4.2 Mode 2 Implementation](#42-mode-2-implementation)
  - [4.3 Arduino Implementation](#43-arduino-implementation)
  - [4.4 Calibration Implementation](#44-calibration-implementation)
- [5. System Structure by Operating Phase](#5-system-structure-by-operating-phase)
  - [5.1 Initialization](#51-initialization)
  - [5.2 Mode 1: Adaptive Vision-Based Navigation](#52-mode-1-adaptive-vision-based-navigation)
  - [5.3 Mode 2: Choreography-Based Manual Sequence](#53-mode-2-choreography-based-manual-sequence)
  - [5.4 Vision Calibration Support](#54-vision-calibration-support)
  - [5.5 Operating Cycle](#55-operating-cycle)
- [6. Conclusion](#6-conclusion)

<!-- tocstop -->

---

## 1. General Description

This document presents the implemented navigation system of an autonomous robot designed for the **WRO Future Engineers** competition, based on the following files:

- `src/1st_mode.py` (Adaptive vision-based navigation)
- `src/2nd_mode.py` (Choreography-based manual sequence)
- `src/Calibration.py` (Vision calibration support tool)
- `src/Ino Code/Arduino_Code.ino` (Arduino motor, servo, and distance controller)

According to the WRO Future Engineers 2026 rules, the vehicle operates in a self-driving car challenge in which it must drive autonomously on a track whose configuration varies between rounds. The official challenge includes Open Challenge rounds and Obstacle Challenge rounds, both based on autonomous track navigation.

The solution is distributed between a **Raspberry Pi**, responsible for vision processing and decision-making, and an **Arduino**, responsible for executing physical actions on the steering and traction system.

This documentation remains aligned with the code currently available in the repository and describes the implemented logic only.

---

## 2. System Objective

The objective of the system is to allow the robot to:

- Observe the track through a camera.
- Detect the track and the walls visually.
- Adapt to a round direction that may be clockwise or counterclockwise.
- Complete the required laps on the track autonomously.
- Determine a navigation state.
- Generate speed and steering-angle commands.
- Calibrate camera thresholds before testing or competition runs.
- Execute the commands through the Arduino.

In the implemented system, these tasks are addressed through computer vision, time-based choreography, state-based decision logic, calibration tools, and serial communication between the Raspberry Pi and the Arduino.

The Raspberry Pi acts as the host computer executing the Python scripts depending on the selected challenge mode, while the Arduino operates as an embedded microcontroller executing the `src/Ino Code/Arduino_Code.ino` code. They exchange text data through serial communication using a synchronized speed setting called the **baud rate**.

---

## 3. Robot Architecture

### 3.1 Functional Distribution

| Module       | File                            | Main Function                                                                                            |
| ------------ | ------------------------------- | -------------------------------------------------------------------------------------------------------- |
| Raspberry Pi | `src/1st_mode.py`               | Real-time vision processing, adaptive navigation, automatic lap detection, and reactive steering control |
| Raspberry Pi | `src/2nd_mode.py`               | Choreographed sequence execution, pre-programmed motion commands, and time-based navigation              |
| Raspberry Pi | `src/Calibration.py`            | Camera calibration tool for track thresholding and HSV color ranges                                      |
| Arduino      | `src/Ino Code/Arduino_Code.ino` | Command reception, servo control, motor control, distance reading, and physical execution                |
| Camera       | Accessed through OpenCV         | Track image acquisition and real-time frame processing                                                   |

### 3.2 Physical System Flow

```text
Camera -> Raspberry Pi -> Serial -> Arduino -> Servo / Motor
```

The camera sends visual information to the Raspberry Pi. The Raspberry Pi processes the image and decides the movement command. The command is sent by serial communication to the Arduino. The Arduino then applies the received values to the steering servo and the traction motor.

### 3.3 Software-to-Hardware Connection

The current software connects with the physical components in the following way:

- **USB Camera -> Raspberry Pi:** `1st_mode.py` and `Calibration.py` open the camera with `cv2.VideoCapture(0)` and read live frames.
- **Raspberry Pi -> Arduino:** Both Python control modes open the serial port `/dev/ttyUSB0` at `115200` baud and send movement packets in `<speed,angle>` format.
- **Arduino-side serial input -> Raspberry Pi:** Both Python modes are prepared to listen for the `BTN:1` serial message used as the start signal.
- **Arduino -> Steering Servo:** The steering angle calculated or selected by the Python software is transmitted through the serial packet and then physically applied by the Arduino to the front steering servo.
- **Arduino -> Drive Motor:** The speed value calculated or selected by the Python software is transmitted through the same serial packet and then physically applied by the Arduino to the traction motor.
- **Arduino -> Raspberry Pi:** The Arduino also sends distance telemetry using the `DIST:<distance>` format.

---

## 4. Software Implementation

### 4.1 Mode 1 Implementation
### File: `src/1st_mode.py`

<img src="../resources/diagrama_modo_1_primitivo_blindado.png" alt="Flow diagram of Mode 1: adaptive vision-based navigation">

Mode 1 is the autonomous navigation mode. It uses the camera to analyze the track in real time, detect the walls, estimate the direction of the route, control the steering angle, and count laps.

#### Imported Libraries

```python
import cv2
import numpy as np
import serial
import time
import threading
```

| Library     | What it is                                                     | Why it is used in this project                                                                 |
| ----------- | -------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `cv2`       | OpenCV library for computer vision                             | Used to capture camera frames, convert images, apply blur, thresholding, morphology, and display |
| `numpy`     | Numerical computing library for arrays and matrices            | Used to count pixels, create kernels, calculate averages, and process image regions              |
| `serial`    | PySerial communication library                                 | Used to send `<speed,angle>` commands to Arduino and read button events                          |
| `time`      | Python timing library                                          | Used to control delays, debounce corners, measure race time, and pace control loops              |
| `threading` | Python library for parallel execution                          | Used to run serial reading and camera vision at the same time                                   |

#### Why These Libraries Are Needed

This mode requires the robot to react continuously while it is moving. The camera processing cannot block the serial communication, and the serial communication cannot stop the image analysis. For that reason, `threading` is used so the robot can process vision and listen for button events simultaneously.

`cv2` and `numpy` are used together because OpenCV returns images as numerical arrays. Every image is treated as a matrix of pixels, and the code analyzes those pixels to detect walls and make navigation decisions.

#### Main Methods

```python
class WROPrimitivoBlindado:
    def __init__(self):
    def read_serial_data(self):
    def process_vision(self):
    def main_loop(self):
```

#### Main Responsibilities

- Open serial communication with the Arduino.
- Capture the camera image.
- Process the track area using computer vision.
- Detect the front wall and side walls.
- Determine the turning direction automatically.
- Compute speed and steering angle.
- Count corners and completed laps.
- Send movement commands to the Arduino.

#### Configuration Parameters

```python
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
```

#### Important Values

```python
self.current_angle = 86
self.current_speed = 0
self.SENTIDO_GIRO = "AUTO"
self.VER_PANTALLAS = True
```

> [!IMPORTANT]
> The angle was set to `86` because at `90` the wheels were not completely straight on the physical robot.

#### General Flow of Mode 1

1. The program tries to open the serial connection with Arduino.
2. It initializes the robot state as `ESPERA`.
3. It starts one thread to read serial data from Arduino.
4. It starts another thread to process the camera image.
5. It waits for the `BTN:1` button signal.
6. When the button is pressed, it changes to `CARRERA`.
7. During the race, it continuously sends `<speed,angle>` packets.
8. When three laps are completed, it changes to `RETORNO_A_META`.
9. The robot performs a final stop sequence and the program ends.

---

### 4.2 Mode 2 Implementation
### File: `src/2nd_mode.py`

<img src="../resources/diagrama_modo_2_coreografia_manual.png" alt="Flow diagram of Mode 2: choreography-based manual sequence">

Mode 2 is a choreography-based manual sequence. It does not use camera feedback during execution. Instead, it follows a predefined list of movement commands where each step contains speed, steering angle, duration, and description.

#### Imported Libraries

```python
import serial
import time
```

| Library  | What it is                              | Why it is used in this project                                                    |
| -------- | --------------------------------------- | ---------------------------------------------------------------------------------- |
| `serial` | PySerial communication library          | Used to send timed `<speed,angle>` packets from the Raspberry Pi to the Arduino     |
| `time`   | Python timing library                   | Used to keep each movement active for the programmed number of seconds             |

#### Why These Libraries Are Needed

This mode depends on timing instead of camera feedback. The `time` library is necessary because every movement instruction must stay active for an exact duration. The `serial` library is required because each movement still needs to be sent to the Arduino using the same command format as Mode 1.

#### Main Methods

```python
class WROCoreografia:
    def __init__(self):
    def esperar_boton(self):
    def ejecutar_rutina(self):
    def run(self):
```

#### Main Responsibilities

- Open serial communication with the Arduino.
- Keep the robot stopped while waiting for the start button.
- Wait for the `BTN:1` signal.
- Execute `RUTINA_MANUAL` step by step.
- Send `<speed,angle>` packets to Arduino.
- Keep each command active for its assigned time.
- Stop the robot at the end.

#### Configuration Parameters

```python
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
RUTINA_MANUAL = [
    (velocity, angle, duration, "description"),
]
```

#### Choreography Tuple Format

Each movement step uses this structure:

```python
(speed, angle, time_in_seconds, "comment")
```

Example:

```python
(250, 86, 0.6, "Aceleración en recta inicial")
```

This means:

- `250`: motor speed command.
- `86`: steering angle, used as the neutral straight position.
- `0.6`: duration in seconds.
- `"Aceleración en recta inicial"`: description printed in the terminal.

#### Servo Handshake Sequence

Before the main routine starts, the script performs a steering handshake:

```python
self.ser.write(b"<0,120>\n")
time.sleep(0.3)
self.ser.write(b"<0,60>\n")
time.sleep(0.3)
self.ser.write(b"<0,86>\n")
```

This moves the wheels to one side, then to the other side, and finally centers them. The objective is to visually confirm that the steering servo is responding correctly.

#### General Flow of Mode 2

1. The program opens serial communication with Arduino.
2. It waits 2 seconds so the Arduino can reset after the serial connection opens.
3. It performs the servo handshake.
4. It waits for the `BTN:1` start signal.
5. It reads the next tuple from `RUTINA_MANUAL`.
6. It converts the tuple into a `<speed,angle>` command.
7. It sends the command to Arduino.
8. It waits for the duration defined in the tuple.
9. It continues until all instructions are completed.
10. It sends `<0,86>` to stop and center the robot.

---

### 4.3 Arduino Implementation
### File: `src/Ino Code/Arduino_Code.ino`

<img src="../resources/diagrama_arduino_controlador.png" alt="Flow diagram of the Arduino controller: motor, servo, and distance sensor">

The Arduino code is the physical execution layer of the robot. It receives commands from the Raspberry Pi, parses the speed and angle values, applies the steering angle to the servo, controls the traction motor, and periodically sends distance sensor readings.

#### Imported Libraries

```cpp
#include <Servo.h>
#include <stdlib.h>
```

| Library    | What it is                                      | Why it is used in this project                                                |
| ---------- | ----------------------------------------------- | ------------------------------------------------------------------------------ |
| `Servo.h`  | Arduino library for controlling servo motors    | Used to control the steering servo connected to pin `8`                        |
| `stdlib.h` | Standard C library with utility functions       | Used to convert text values from the serial packet into numeric values         |

#### Why These Libraries Are Needed

The Arduino receives the command as text, for example `<250,86>`. Before applying it to the motor and servo, the Arduino must separate the values and convert them into numbers. `stdlib.h` is used for this conversion. `Servo.h` is used because the steering system depends on a servo motor that receives angle instructions.

#### Main Hardware Connections

```cpp
const int pinServo = 8;
const int pinMotorPWM = 7;
const int pinMotorDir1 = 9;
const int pinMotorDir2 = 10;
const int pinTrig = 3;
const int pinEcho = 11;
```

| Pin  | Component         | Function                |
| ---- | ----------------- | ----------------------- |
| `8`  | Steering servo    | Steering angle output   |
| `7`  | Motor driver PWM  | Motor speed control     |
| `9`  | Motor driver IN1  | Motor direction line 1  |
| `10` | Motor driver IN2  | Motor direction line 2  |
| `3`  | Distance Trig     | Trigger pulse output    |
| `11` | Distance Echo     | Echo pulse input        |

#### Main Responsibilities

- Initialize serial communication at `115200`.
- Receive `<speed,angle>` packets from the Raspberry Pi.
- Parse and validate the received values.
- Apply the angle to the steering servo.
- Apply the speed to the traction motor.
- Read the distance sensor periodically.
- Send distance telemetry as `DIST:<distance>`.

#### Main Variables

```cpp
int distanciaSensor = 200;
int velocidadAuto = 0;
int anguloServo = 86;
unsigned long previousMillisSensor = 0;
```

These variables store:

- the last measured distance,
- the current motor speed,
- the current steering angle,
- the timing reference for distance sampling.

#### Setup Sequence

In `setup()`, the Arduino:

1. Starts serial communication with `Serial.begin(115200)`.
2. Attaches the steering servo.
3. Centers the servo at `86`.
4. Configures the motor pins as outputs.
5. Configures the distance trigger pin as output.
6. Configures the distance echo pin as input.

#### Serial Protocol Received by Arduino

The Arduino expects packets with start and end markers:

```text
<speed,angle>
```

Example:

```text
<250,86>
```

The function `recvWithStartEndMarkers()` reads the packet between `<` and `>`. The function `parseData()` separates the values using the comma and converts them into:

- `velocidadAuto`
- `anguloServo`

#### Applied Limits

The Arduino constrains the parsed values as follows:

- speed is limited to `0..255`
- angle is accepted only in the range `60..120`

> [!NOTE]
> In the current Arduino implementation, negative speed values sent from Python are constrained to `0`. This means that reverse commands require Arduino-side support if reverse movement is intended.

#### Movement Execution

The function `ejecutarMovimiento()`:

- writes `anguloServo` to the steering servo,
- drives the motor forward when `velocidadAuto > 0`,
- stops the motor when `velocidadAuto == 0`.

#### Distance Telemetry

Every 50 ms, the Arduino executes `leerDistancia()` and sends the result through serial:

```text
DIST:<distance>
```

If no echo is received within the timeout, the stored distance falls back to `200`.

This makes the Arduino file the physical execution layer that turns the Raspberry Pi commands into real steering and motor movement while also publishing distance telemetry.

---

### 4.4 Calibration Implementation
### File: `src/Calibration.py`

<img src="../resources/diagrama_calibration_calibrador_vision.png" alt="Flow diagram of Calibration.py: vision calibrator with trackbars">

`Calibration.py` is a support script used to calibrate the camera thresholds before using the robot on the track. It helps adjust the values used to detect the floor, green objects, red objects, and magenta objects.

This file is not the main autonomous control loop. Its purpose is to make vision testing easier by showing the region of interest, the generated mask, and the filtered result in real time.

#### Imported Libraries

```python
import cv2
import numpy as np
```

| Library | What it is                                          | Why it is used in this project                                                                 |
| ------- | --------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `cv2`   | OpenCV library for computer vision                  | Used to capture frames, create control windows, trackbars, masks, thresholding, and visualization |
| `numpy` | Numerical computing library for arrays and matrices | Used to create lower and upper HSV range arrays for color filtering                              |

#### Why These Libraries Are Needed

The calibrator needs to display camera output and update image-processing values live. `cv2` provides the camera capture, windows, trackbars, thresholding, HSV conversion, masks, and display functions. `numpy` provides the array format needed to define HSV lower and upper bounds.

#### Main Functions and Process

```python
def nada(x):
    pass
```

The `nada()` function is used as a callback for the OpenCV trackbars. The function does not need to execute code because the main loop reads the trackbar positions continuously.

#### Calibration Modes

The script includes four calibration modes:

| Mode | Name    | Purpose                                           |
| ---- | ------- | ------------------------------------------------- |
| `0`  | Piso    | Adjusts the black/white threshold for the track   |
| `1`  | Verde   | Adjusts HSV limits for green object detection     |
| `2`  | Rojo    | Adjusts HSV limits for red object detection       |
| `3`  | Magenta | Adjusts HSV limits for magenta object detection   |

#### Trackbars

The script creates interactive sliders for:

- `MODO`
- `H Min`
- `H Max`
- `S Min`
- `S Max`
- `V Min`
- `V Max`
- `Umbral Piso`

These sliders allow the team to test values directly with the camera instead of changing the code manually every time.

#### Region of Interest

The script uses two different ROI configurations:

```python
roi = frame[80:140, 0:320]
```

Used for:

- floor detection,
- magenta detection.

```python
roi = frame[60:240, 0:320]
```

Used for:

- green object detection,
- red object detection.

The ROI reduces the amount of image that is processed and focuses the analysis on the parts of the frame that are useful for the robot.

#### Floor Calibration

For floor calibration, the script:

1. Converts the ROI to grayscale.
2. Applies Gaussian blur with a `7x7` kernel.
3. Applies a binary threshold using the selected `Umbral Piso`.
4. Displays the mask as the detected result.

#### Color Calibration

For green, red, and magenta calibration, the script:

1. Converts the ROI from BGR to HSV.
2. Reads the selected minimum and maximum HSV values.
3. Creates lower and upper HSV arrays.
4. Uses `cv2.inRange()` to create a mask.
5. Uses `cv2.bitwise_and()` to show only the detected color area.

#### Keyboard Controls

| Key | Action                                      |
| --- | ------------------------------------------- |
| `p` | Prints the current calibration values        |
| `q` | Exits the calibration program                |

#### General Flow of Calibration.py

1. Open the camera.
2. Set the resolution to `320x240`.
3. Create the control window.
4. Create all trackbars.
5. Read a frame from the camera.
6. Read the selected mode.
7. If the mode changed, load the default values for that mode.
8. Crop the proper ROI.
9. Apply floor thresholding or HSV color filtering.
10. Show the ROI, mask, and filtered result.
11. Print values if `p` is pressed.
12. Exit if `q` is pressed.

---

## 5. System Structure by Operating Phase

The current implementation can be read in five main phases:

1. Serial and hardware initialization.
2. Mode 1 autonomous execution.
3. Mode 2 choreography execution.
4. Vision calibration and parameter adjustment.
5. Arduino-side physical execution.

### 5.1 Initialization

Both Python control modes begin by opening the serial connection with the Arduino:

```python
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
```

After opening the port, each script waits 2 seconds so the Arduino can stabilize after the serial reset.

In both control modes, the physical connection at startup is:

- Raspberry Pi -> serial port -> Arduino
- Arduino-side start signal -> Raspberry Pi

Both control modes also use an initial steering handshake before the main action starts:

```python
self.ser.write(b"<0,120>\n") # Turn right the wheels
time.sleep(0.3)
self.ser.write(b"<0,60>\n") # Turn left the wheels
time.sleep(0.3)
self.ser.write(b"<0,86>\n") # Straighten the wheels
```

This confirms that the steering servo responds correctly before the robot begins the selected routine.

### 5.2 Mode 1: Adaptive Vision-Based Navigation

**Class:** `WROPrimitivoBlindado`

Mode 1 is the autonomous camera-based mode. It uses the USB camera as the main physical input and continuously sends steering and speed commands to the Arduino.

#### How Mode 1 Works

The script starts two parallel tasks:

- `read_serial_data()`: waits for the `BTN:1` start signal from the Arduino side.
- `process_vision()`: captures and processes frames from the USB camera.

The main loop sends `<speed,angle>` packets to the Arduino while the robot is in race mode.

#### Vision Processing Steps

The vision pipeline follows these steps:

1. Capture one frame from the camera with `cv2.VideoCapture(0)`.
2. Set the resolution to `320x240`.
3. Convert the frame to grayscale.
4. Apply Gaussian blur with a `5x5` kernel.
5. Crop the track band using rows `60:150`.
6. Apply threshold `95` to create a binary image.
7. Apply morphological opening with a `3x3` kernel.
8. Split the processed image into:
   - `horizonte` for far-track analysis,
   - `linea_escaneo` for near wall detection.
9. Analyze the central box to detect a front wall.
10. Scan from the center to both sides to estimate side walls.

#### Grayscale Image

A grayscale image is an image with only one intensity channel instead of three color channels. In this project, the camera originally captures a BGR image, but the code converts it to grayscale because the robot mainly needs to separate dark walls from bright floor areas.

This makes the processing faster and simpler because the algorithm only needs to compare brightness values instead of analyzing full color information.

#### Gaussian Blur

Gaussian blur is used to smooth the image before thresholding. Small camera noise, reflections, or texture changes can create isolated pixels that confuse wall detection.

The code applies:

```python
blur = cv2.GaussianBlur(gray, (5, 5), 0)
```

The `(5,5)` value is the kernel size. It means the blur operation looks at a 5 by 5 pixel neighborhood around each pixel to calculate a smoother result.

#### Kernel

A kernel is a small matrix used by image-processing operations. The code uses kernels in two important parts:

```python
cv2.GaussianBlur(gray, (5, 5), 0)
```

and:

```python
kernel = np.ones((3, 3), np.uint8)
binarizada = cv2.morphologyEx(binarizada, cv2.MORPH_OPEN, kernel)
```

The `5x5` kernel is used for blur, while the `3x3` kernel is used for morphology. The morphology kernel tells OpenCV how large the neighborhood should be when removing small noise from the binary image.

#### Binary Image

A binary image is an image that only contains two values:

- `0`: black
- `255`: white

The code creates it using:

```python
_, binarizada = cv2.threshold(roi_pista, 95, 255, cv2.THRESH_BINARY)
```

This means that pixels brighter than the threshold become white, and pixels darker than the threshold become black. In this robot, this helps separate the light floor from the dark track walls.

#### Morphological Opening

Morphological opening is an image-cleaning operation. It is useful after thresholding because the binary image may contain small isolated noise points.

Opening is normally understood as:

1. Erosion: removes small white noise.
2. Dilation: restores the remaining valid shapes.

In this project, the operation is used to remove small white specks that could be confused with track openings or wall boundaries.

#### Horizon and Scan Line

After cleaning the binary image, the code separates two useful areas:

```python
horizonte = binarizada[0:25, :]
linea_escaneo = binarizada[65, :]
```

- `horizonte` is the upper part of the processed ROI. It is used to estimate whether the path is more open to the left or to the right.
- `linea_escaneo` is a lower row of the ROI. It is used to scan for the nearest visible left and right walls.

#### Direction Detection

At the beginning of the race, the code compares the white pixels on the left and right halves of the horizon:

- more white on the right -> `DERECHA`
- more white on the left -> `IZQUIERDA`

This establishes the corner direction that will be used for the rest of the run.

#### Front and Side Wall Detection

The code detects the front wall from a central box:

```python
caja_central = binarizada[30:70, 120:200]
ratio_oscuro = np.mean(caja_central == 0)
```

The corner is considered detected when the dark-pixel ratio is greater than `0.55`.

The side walls are detected by scanning from the image center outward:

- left side -> `muro_izq`
- right side -> `muro_der`

These values are used to estimate the track center.

#### State Machine

Mode 1 uses three states:

| State            | Meaning                                                                  |
| ---------------- | ------------------------------------------------------------------------ |
| `ESPERA`         | The robot is idle and waiting for the start button                       |
| `CARRERA`        | The robot is actively navigating the track                               |
| `RETORNO_A_META` | The robot has completed the required laps and executes the stop sequence |

#### Steering and Speed Logic

When a front wall is detected:

- angle `70` for right corners,
- angle `104` for left corners,
- speed `180`.

When no front wall is detected:

- speed `250`,
- the robot estimates the center of the track,
- the steering is corrected with proportional control,
- angle `86` is kept if the error is inside the `22` pixel dead zone,
- straight-line steering is clamped between `74` and `98`.

#### Lap Counting

The code counts one corner only if the robot is not already flagged as being inside a corner and at least 2 seconds have passed since the previous corner.

Every 4 corners:

```python
vueltas_completadas += 1
```

After 3 completed laps:

- the state changes to `RETORNO_A_META`,
- the robot performs a short stop sequence,
- the program ends.

#### Physical Connections Used in Mode 1

- The USB camera provides the visual input.
- The Raspberry Pi processes the image and computes the steering and speed values.
- The serial cable carries commands from Raspberry Pi to Arduino.
- The Arduino applies the received values to the steering servo and drive motor.
- The start signal is read in Python as `BTN:1`.

#### Key Parameters in Mode 1

| Variable               | Value     | Purpose                             |
| ---------------------- | --------- | ----------------------------------- |
| ROI Rows               | 60-150    | Track-focused image band            |
| Binarization Threshold | 95        | White/black separation              |
| Gaussian Kernel        | 5x5       | Image smoothing                     |
| Morphology Kernel      | 3x3       | Binary noise removal                |
| Scan Line Position     | Row 65    | Wall detection line                 |
| Front Wall Ratio       | 0.55      | Dark-pixel threshold for front wall |
| Straight Speed         | 250       | High-speed on open track            |
| Curve Speed            | 180       | Safe speed through corners          |
| Right Turn Angle       | 70        | Fixed steering angle right          |
| Left Turn Angle        | 104       | Fixed steering angle left           |
| Straight Angle         | 86        | Neutral steering position           |
| Dead Zone              | 22 px     | Straight stability                  |
| Corner Debounce        | 2 seconds | Prevents multiple corner detections |

### 5.3 Mode 2: Choreography-Based Manual Sequence

**Class:** `WROCoreografia`

Mode 2 is the fully timed choreography mode. It does not use the camera for control. Instead, it sends a predefined sequence of commands stored in `RUTINA_MANUAL`.

#### How Mode 2 Works

The sequence is:

1. Open serial communication with Arduino.
2. Perform the steering handshake.
3. Wait for `BTN:1`.
4. Execute each tuple in `RUTINA_MANUAL`.
5. Stop the robot with `<0,86>`.

#### Physical Connections Used in Mode 2

- The physical start button is received through Arduino as `BTN:1`.
- The start signal is received in Python as `BTN:1`.
- The Raspberry Pi sends only timed serial commands in this mode.
- The serial link carries each `<speed,angle>` packet.
- The Arduino applies each command to the steering servo and drive motor.

#### Choreography Structure

Each instruction uses this format:

```python
(velocity, angle, duration_seconds, "description")
```

Meaning:

- `velocity`: motor command sent by Python.
- `angle`: steering command.
- `duration_seconds`: command duration.
- `description`: console label.

#### Choreography Phases

The current routine contains:

1. Parking exit.
2. First lap.
3. Second lap.
4. Third lap.
5. Parking entry.
6. Final motor stop.

#### Key Parameters in Mode 2

| Parameter               | Example Values | Purpose                               |
| ----------------------- | -------------- | ------------------------------------- |
| Python Velocity Command | -140 to 250    | Motor values sent by the Python list  |
| Arduino Applied Speed   | 0 to 255       | Speed range currently accepted        |
| Right Turn Angle        | 60             | Programmed right steering             |
| Center Angle            | 86             | Neutral steering                      |
| Left Turn Angle         | 120            | Programmed left steering              |
| Turn Duration           | 1.58-1.8s      | Curve execution time                  |
| Straight Duration       | 0.6-2.8s       | Straight segments                     |

### 5.4 Vision Calibration Support

**File:** `src/Calibration.py`

The calibration script supports the vision process by allowing the team to tune threshold and HSV values visually. This is important because lighting conditions, floor reflection, wall contrast, and camera position can change the values that work correctly.

#### Relation With Mode 1

Mode 1 uses thresholding to separate the track floor and walls. `Calibration.py` helps test the threshold value before using the robot because it displays:

- the original ROI,
- the binary or color mask,
- the filtered result.

This makes it easier to choose values that produce stable detection.

#### HSV Color Calibration

HSV separates color information into:

- `H`: Hue, the type of color.
- `S`: Saturation, the intensity of the color.
- `V`: Value, the brightness of the color.

This is useful because detecting a color such as green, red, or magenta is easier in HSV than in the original BGR camera format. The code creates a range with minimum and maximum values, and everything inside that range becomes white in the mask.

#### Mask

A mask is an image that marks the pixels that match a condition. In this project:

- white pixels represent detected areas,
- black pixels represent ignored areas.

For floor mode, the mask comes from a brightness threshold. For color modes, the mask comes from HSV filtering.

#### Calibration Output

When the `p` key is pressed, the program prints the current calibration values in the terminal. These values can then be transferred to the navigation code if the team needs to adjust detection thresholds.

### 5.5 Operating Cycle

#### Mode 1 Cycle

1. The camera provides a frame.
2. The Raspberry Pi processes the image.
3. The track walls are estimated.
4. Speed and steering are calculated.
5. A `<speed,angle>` packet is sent to Arduino.
6. The Arduino drives the servo and motor.
7. The process repeats until three laps are completed.

#### Mode 2 Cycle

1. The Raspberry Pi reads the next tuple from `RUTINA_MANUAL`.
2. The command is converted into `<speed,angle>`.
3. The packet is sent to Arduino.
4. The Arduino drives the servo and motor.
5. The Raspberry Pi waits for the configured duration.
6. The process repeats until the routine is complete.

#### Calibration Cycle

1. The camera provides a frame.
2. The selected calibration mode is read.
3. The correct ROI is selected.
4. The image is processed using thresholding or HSV filtering.
5. The ROI, mask, and result are displayed.
6. The user adjusts the trackbars.
7. The current values can be printed with `p`.

From the hardware perspective, the software currently reads from:

- USB camera,
- start button event through Arduino,
- distance sensor through Arduino telemetry.

And writes to:

- steering servo,
- drive motor.

---

## 6. Conclusion

The current software documented in this repository is centered on two Python control modes, one calibration support script, and one Arduino execution layer:

- `src/1st_mode.py` for autonomous camera-based navigation.
- `src/2nd_mode.py` for manual time-based choreography.
- `src/Calibration.py` for camera threshold and HSV calibration.
- `src/Ino Code/Arduino_Code.ino` for receiving commands and controlling the physical hardware.

Both control modes use the same physical communication path from the Raspberry Pi to the Arduino and from there to the steering and traction hardware. Mode 1 reacts to live camera input, Mode 2 follows a predefined timed routine, and the calibration script helps adjust the vision parameters before testing the robot on the track.
