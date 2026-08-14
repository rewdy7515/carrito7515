# WRO2026 Future Engineers – Ingenieros Paralelos

## About us

>Team members
- Samuel Guaimacuto
- Andrés Villareal
- David Xu

_We are a Venezuelan team conformed by Informatic Engineering students of Universidad Gran Mariscal de Ayacucho (UGMA), núcleo Barcelona, being our first time participating in a WRO competition, competing in the Future Engineers category. Our inspiration to be part of this tournament was the desire to learn about robotics' world, wanting to face this challenge in order to achieve it. We are grateful with all our family, professors and classmates, without their support it would not have been possible to achieve what we set out to do._

<img src="./t-photos/igshiix0ajvkbmse051z.jpeg" alt="Photo of Us" width="500">

<hr>

## Table of Contents

<!-- toc -->

- [Preview of the Car](#preview-of-the-car)
- [Preview of the Car Performance](#preview-of-the-car-performance)
- [Components Used and Estimated Budget](#components-used-and-estimated-budget)
- [Vision Management](#vision-management)
  - [Logitech C922 Web Camera](#logitech-c922-web-camera)
  - [Raspberry Pi 4](#raspberry-pi-4)
- [Mobility Management](#mobility-management)
  - [Arduino Uno](#arduino-uno)
  - [L298N Driver](#l298n-driver)
  - [Fischertechnik Maker Kit Car](#fischertechnik-maker-kit-car)
  - [Ackermann Mechanism](#ackermann-mechanism)
  - [Ackermann Principle](#ackermann-principle)
  - [Ackermann in our project](#ackermann-in-our-project)
- [Power Management](#power-management)
  - [LX-2BUPS UPS](#lx-2bups-ups)
  - [Ultrafire TR 18650 Batteries](#ultrafire-tr-18650-batteries)
- <a href="src"> Obstacle Management </a>

<!-- tocstop -->

<hr>

## Preview of the Car

<table>
  <tr>
    <td align="center"><b>Top</b><br><img src="./v-photos/IMG_5101.JPG" width="300"></td>
    <td align="center"><b>Front</b><br><img src="./v-photos/IMG_5102.JPG" width="300"></td>
    <td align="center"><b>Left</b><br><img src="./v-photos/IMG_5103.JPG" width="300"></td>
  </tr>
  <tr>
    <td align="center"><b>Bottom</b><br><img src="./v-photos/IMG_5104.JPG" width="300"></td>
    <td align="center"><b>Back</b><br><img src="./v-photos/IMG_5105.JPG" width="300"></td>
    <td align="center"><b>Right</b><br><img src="./v-photos/IMG_5100.JPG" width="300"></td>
  </tr>
</table>

<hr>

## Preview of the Car Performance

<img src="/resources/Car Preview.gif" alt="Car Preview" width="80%">

<a href="https://www.youtube.com/watch?v=6vZ5giluS2M"> Click to See the Complete Performance </a>

<hr>

## Components Used and Estimated Budget

| Component | Quantity | Estimated Unit Price | Estimated Subtotal | Reference |
|---|---:|---:|---:|---|
| Raspberry Pi 4 Model B 4GB Kit | 1 | $200.00 | $200.00 | MercadoLibre Venezuela |
| Arduino Uno R3 | 1 | $9.99 | $9.99 | MercadoLibre Venezuela |
| L298N Motor Driver | 1 | $6.99 | $6.99 | MercadoLibre Venezuela |
| Logitech C922 Camera | 1 | $70.00 | $70.00 | MercadoLibre Venezuela |
| LX-2BUPS UPS Module | 1 | $17.80 | $17.80 | MercadoLibre Venezuela |
| 18650 3.7V Battery | 4 | $5.00 | $20.00 | MercadoLibre Venezuela |
| Fischertechnik Maker Kit Car | 1 | $115.33 | $115.33 | eBay |

### Estimated Total: $443.10

<hr>

## Vision Management

- #### Logitech C922 Web Camera

<table>
  <tr>
    <td align="center" width="300" >
      <img src="./resources/Logitech C922.png" alt="Logitech C922 Webcam" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Max resolution: 1080p at 30fps (Full HD) or 720p at 60fps (HD) </li>
        <li>Field of View (FoV): 78° diagonal </li>
        <li>Focus Type: Autofocus </li>
        <li>Lens Technology: Full HD glass lens with automatic light correction</li>
        <li>Audio: Dual omnidirectional stereo microphones </li>
        <li>Connectivity: Wired USB 2.0 (includes a 5-foot / 1.5m cable)</li>
      </ul>
    </td>
  </tr>
</table>

The Logitech C922 Pro Stream is a popular, high-definition webcam designed specifically for content creators, streamers, and professionals. It offers sharp video resolution, smooth frame rates for fluid motion, and a convenient low-light correction feature. In our project, we used it as the eye of the car, catching the view of the environment.

<hr>

- #### Raspberry Pi 4

<table>
  <tr>
    <td align="center" width="300" >
      <img src="./resources/Raspberry Pi 4.png " alt="Raspberry Pi 4" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Processor: Quad-Core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5–1.8 GHz </li>
        <li>Memory: 4GB LPDDR4-3200 SDRAM </li>
        <li>Video: Dual micro-HDMI ports supporting 4K @ 60fps </li>
        <li>Connectivity: Gigabit Ethernet, 2.4/5.0 GHz Wi-Fi, and Bluetooth 5.0 </li>
        <li>USB: 2x USB 3.0, 2x USB 2.0 ports </li>
        <li>Power: USB-C (5V/3A) or Power over Ethernet (PoE) supported </li>
      </ul>
    </td>
  </tr>
</table>

The Raspberry Pi 4 Model B is a credit card-sized, single-board computer. It functions as a fully operational, low-cost computer capable of desktop computing, media streaming, home automation, and robotics, while using only a fraction of the power of a standard desktop. <b>This piece of hardware acts as the brain of the car, with software capable to process the view of the web camera, deciding which is the most appropriate action to execute, according the scenery, to later let our microcontroller perform it</b>.

<a href="src"> See the implemented code in the Raspberry </a>

<hr>

## Mobility Management

- #### Arduino Uno

<table>
  <tr>
    <td align="center" >
      <img src="./resources/Arduino_Uno.png " alt="Arduino Uno" width="300" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li> Microcontroller: ATmega328P </li>
        <li> Operating Voltage: 5V </li>
        <li> Input Voltage (Recommended): 7V to 12V </li>
        <li> Input Voltage (Limit): 6V to 20V </li>
        <li> Digital I/O Pins: 14 (6 provide PWM output) </li>
        <li> Analog Input Pins: 6 </li>
        <li> DC Current per I/O Pin: 20mA </li>
        <li> Clock Speed: 16MHz </li>
        <li> Flash Memory: 32KB (of which 0.5KB is used by the bootloader) </li>
        <li> SRAM: 2KB </li>
        <li> EEPROM: 1KB </li>
      </ul>
    </td>
  </tr>
</table>

The Arduino Uno is a beginner-friendly, open-source microcontroller board used for building digital devices and interactive projects. It acts as the brain of our project, it allows to read inputs such as a sensor, button, or temperature reading and turn them into outputs, like moving a motor or turning on an LED. <b>This hardware acts as the nervous system of our car, sending small electric pulses to the driver. Since it is our first time participating in this kind of tournaments, we decided to begin trying this model of Arduino</b>. 

<a href="src"> See the implemented code in the Arduino </a>

<hr>

- #### L298N Driver

<table>
  <tr>
    <td align="center" width="300" >
      <img src="./resources/Driver_L298N.png " alt="Driver L298N" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li> Driver IC: STMicroelectronics L298N </li>
        <li> Motor Supply Voltage (Vs): 5V to 35V </li>
        <li> Peak Output Current: 2A per channel (4A total max) </li>
        <li> Logic Supply Voltage (Vss): 5V to 7V </li>
        <li> Maximum Power Dissipation: 20W at 75°C </li>
        <li> Control Signal Level: Low (-0.3V to 1.5V), High (2.3V to Vss) </li>
      </ul>
    </td>
  </tr>
</table>

The L298N is a dual H-Bridge motor driver module used to control the direction and speed of DC or stepper motors. It acts as a bridge between the microcontroller, the Arduino Uno, and high-power motors (in our case the servo motor and the encoder motor), supplying the necessary current and voltage. <b>This component acts as the muscles of the car, suplying the necessary energy to the motors, but since the Arduino and Raspberry manage a low voltage (not enough to supply the driver), it is necessary the implementation of an additional power source to this component</b>.

<hr>

- #### Fischertechnik Maker Kit Car

<img src="./resources/Fischertechnik_Maker_Kit_Car.png " alt="Fischertechnik Maker Kit Car" width="300px" >

The Fischertechnik Maker Kit Car is an advanced construction kit designed for makers, hobbyists, and robotic enthusiasts to build a highly customizable, mobile robotic vehicle chassis. Includes pieces for building sturdy structural superstructures and custom mounts, so we took advantage of this by using the blocks as the base or skeleton of our car to later assembly the other components around it.

<h3> Other components the kit contains </h3>

> Servomotor

<a href="schemes/Fistchertechnik Maker Kit Car/BA_DATENBLATT_MAKER_KIT_CAR_ENCODERMOTOR.pdf">Check specifications</a>

Is a specialized motor designed to turn to a specific angle (in this case between 60° and 120°) and hold that position. It connects directly to the front steering knuckles of the chassis and it controls the steering mechanism. Unlike the drive motor, it is not programmed to spin continuously. Instead, it is commanded to change degrees, giving the car precise navigation capabilities.

> Encoder Motor or C Motor

<a href="schemes/Fistchertechnik Maker Kit Car/BA_DATENBLATT_MAKER_KIT_CAR_ENCODERMOTOR.pdf">Check specifications</a>

Is the primary drive engine of the vehicle. It does not just spin; it counts its own rotations. It provides the driving power (traction) to move the car forward and backward. The built-in encoder sends digital pulses back to our controller, the Arduino Uno. This allows us to measure exactly how far the car has traveled, calculate its speed, and program precise movements.

> Differential Gear

Is a mechanical gearbox located between the two driven wheels. It allows the left and right wheels to rotate at different speeds while still receiving power from the motor. When the car turns, the outside wheel has to travel a longer distance than the inside wheel. Without a differential, the wheels would lock up, slip, or skid during turns. This component ensures smooth, realistic cornering and prevents our car from losing traction.

<hr>

- #### Ackermann Mechanism

<img src="./resources/Ackermann_Turning.png " alt="Fischertechnik Maker Kit Car" width="300px" >

When a vehicle takes a turn, the front wheels follow paths with different radii. The inner wheel follows a tighter circle (smaller radius) while the outer wheel describes a wider arc (larger radius). If both wheels point in exactly the same direction (parallel to each other), the inner wheel tends to drag or slip sideways because it is geometrically forced to follow a path that does not correspond to it. This generates: Premature tire wear, Greater steering effort, Loss of stability and grip, Larger turning radius of the vehicle. The Ackermann mechanism solves this problem by making the wheels adopt different angles automatically when the steering wheel is turned.

<hr>

- #### Ackermann principle

The Ackermann principle is based on a geometric condition known as the "Ackermann condition":

In a perfect turn, the axes of all wheels must intersect at a single common point located on the extension of the rear axle. That point is the instantaneous center of rotation of the vehicle.

This implies that:

Inner front wheel → must turn at a larger angle (αᵢ)

Outer front wheel → must turn at a smaller angle (αₑ)

The relationship between both angles is given by the formula:

```text
cot(αₑ) - cot(αᵢ) = d / L
```

Where:

d = distance between the wheel pivot points (track width)

L = distance between axles (wheelbase)

This relationship ensures that, for any steering angle, the center of curvature remains on the line of the rear axle, preventing lateral dragging of the wheels.

<hr>

- #### Ackermann in our project

In our car there is not presence of this mechanism, or it is called a 0% Ackermann, this does not affect a lot the performance since is a small vehicle, but if there was presence of this in the project it would help us improve the times. There are some reasons the kit does not includes it:

1. It is a basic or "entry-level" kit – The Maker Kit Car is designed for the maker market as a base chassis, robust and easy to expand, not as a high-performance scale model.

2. Priority on educational functionality – Its main objective is to serve as a platform for integrating development boards (Arduino, Raspberry Pi) and learning about robotics and programming. A simpler steering system, such as a steering knuckle with a servo motor, is easier to build and program for a beginner user.

3. Product differentiation – fischertechnik reserves the Ackermann mechanism for its more advanced kits focused on competition, such as the STEM Coding Competition, which have a much higher price and complexity. The Maker Kit Car, with its 119 pieces, is a more affordable and versatile option for creative projects.

4. Cost and manufacturing simplicity – A complete Ackermann mechanism requires more parts (angled steering arms, additional track rods, precise geometry) than a simple steering knuckle with a servo, which increases production cost and assembly complexity.

5. Target audience – The kit is aimed at makers and hobbyists who want to experiment with electronics and programming, not necessarily at automotive engineering students who require an exact reproduction of vehicle dynamics.

<hr>

## Power Management

- #### LX-2BUPS UPS

<table>
  <tr>
    <td align="center" width="300" >
      <img src="./resources/LX-2BUPS.png " alt="LX-2BUPS" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li> Battery Type: Two parallel 18650 lithium-ion batteries (3.7V) </li>
        <li> Output Voltage: Typically available in 5V, 9V, or 12V versions </li>
        <li> Max Output Current: Up to 3A </li>
        <li> Max Output Power: 15W to 24W </li>
        <li> Input Voltage: Standard DC 5V (via Micro USB or USB Type-C depending on the board variant) </li>
      </ul>
    </td>
  </tr>
</table>

The LX-2BUPS is a popular DIY-style universal uninterruptible power supply (UPS) module. It runs on two parallel-connected 18650 lithium-ion batteries and provides seamless, zero-delay switching between mains power and battery backup, making it ideal for keeping low-power devices like internet routers and modems running during outages. We employed two pieces of this component, one of 5V to the Raspberry and another of 12V for the driver.

<hr>

- #### Ultrafire TR 18650 Batteries

<table>
  <tr>
    <td align="center" width="300" >
      <img src="./resources/Ultrafire_TR18650_9800mAh_3.7V.png " alt="Ultrafire TR 18650 9800mAh 3.7V" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li> Form Factor: Standard 18650 cylindrical cell. </li>
        <li> Diameter: 18 mm. </li>
        <li> Length: 65 mm (can reach up to 68mm if it includes a button-top or an unlisted protection circuit). </li>
        <li> Chemistry: Lithium-ion (Li-ion). </li>
        <li> Terminal Type: Flat top or Button top (varies by distributor). </li>
        <li> Nominal Voltage: 3.7V advertised (Standard Li-ion curve: 4.2V fully charged, ~2.75V cut-off). </li>
        <li> Stated Capacity: 9800 mAh. </li>
      </ul>
    </td>
  </tr>
</table>

In the project we used four of these batteries, two for each UPS. They are rechargeable, we recharge them by plugging in the UPS with a USB-C charger of 20W (admitting 9V / 2.22A).

<hr>

### End of the main section, <a href="src"> click here to see implemented software details </a>.
