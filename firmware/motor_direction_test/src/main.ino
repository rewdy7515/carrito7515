// Prueba manual del servo de dirección.
// El motor queda detenido durante toda la prueba.
//
// Serial Monitor: 115200 baudios
// Comandos:
//   c       -> centro provisional, 86 grados
//   l       -> prueba izquierda moderada, 75 grados
//   r       -> prueba derecha moderada, 97 grados
//   60-120 -> escribir un ángulo específico dentro del rango documentado

#include <Servo.h>

const int pinServo = 8;
const int pinMotorEnable = 7;
const int pinMotorIN1 = 9;
const int pinMotorIN2 = 10;

const int centroProvisional = 86;
const int limiteMinimo = 60;
const int limiteMaximo = 120;

Servo direccion;

void stopMotor() {
  digitalWrite(pinMotorEnable, LOW);
  digitalWrite(pinMotorIN1, LOW);
  digitalWrite(pinMotorIN2, LOW);
}

void writeServoAngle(int angle) {
  if (angle < limiteMinimo || angle > limiteMaximo) {
    Serial.println("Angulo fuera del rango 60-120; no se ejecuta");
    return;
  }

  direccion.write(angle);
  Serial.print("Servo: ");
  Serial.print(angle);
  Serial.println(" grados");
}

void setup() {
  pinMode(pinMotorEnable, OUTPUT);
  pinMode(pinMotorIN1, OUTPUT);
  pinMode(pinMotorIN2, OUTPUT);
  stopMotor();

  direccion.attach(pinServo);
  direccion.write(centroProvisional);

  Serial.begin(115200);
  Serial.println("Prueba de direccion lista");
  Serial.println("Comandos: c=86, l=75, r=97, o 60-120");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command == "c" || command == "C") {
    writeServoAngle(centroProvisional);
  } else if (command == "l" || command == "L") {
    writeServoAngle(75);
  } else if (command == "r" || command == "R") {
    writeServoAngle(97);
  } else {
    int angle = command.toInt();
    if (angle >= limiteMinimo && angle <= limiteMaximo) {
      writeServoAngle(angle);
    } else {
      Serial.println("Comando invalido. Usa c, l, r o un angulo entre 60 y 120");
    }
  }
}

