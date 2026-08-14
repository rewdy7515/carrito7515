// Prueba manual del servo de dirección.
// El motor queda detenido durante toda la prueba.
//
// Serial Monitor: 115200 baudios
// Comandos:
// Los ángulos recibidos son lógicos: 90 significa dirección recta.
// El programa aplica el offset físico: ángulo del servo = ángulo lógico - 4.
//   c       -> centro lógico, 90 (envía 86 al servo)
//   l       -> prueba izquierda moderada, 79 (envía 75 al servo)
//   r       -> prueba derecha moderada, 101 (envía 97 al servo)
//   64-124  -> escribir un ángulo lógico dentro del rango físico 60-120

#include <Servo.h>

const int pinServo = 8;
const int pinMotorEnable = 7;
const int pinMotorIN1 = 9;
const int pinMotorIN2 = 10;

const int centroLogico = 90;
const int centroFisicoServo = 86;
const int limiteFisicoMinimo = 60;
const int limiteFisicoMaximo = 120;

Servo direccion;

void stopMotor() {
  digitalWrite(pinMotorEnable, LOW);
  digitalWrite(pinMotorIN1, LOW);
  digitalWrite(pinMotorIN2, LOW);
}

int logicalToServoAngle(int logicalAngle) {
  return centroFisicoServo + (logicalAngle - centroLogico);
}

void writeLogicalAngle(int logicalAngle) {
  int servoAngle = logicalToServoAngle(logicalAngle);

  if (servoAngle < limiteFisicoMinimo || servoAngle > limiteFisicoMaximo) {
    Serial.println("Angulo fuera del rango fisico 60-120; no se ejecuta");
    return;
  }

  direccion.write(servoAngle);
  Serial.print("Logico: ");
  Serial.print(logicalAngle);
  Serial.print(" grados -> servo fisico: ");
  Serial.print(servoAngle);
  Serial.println(" grados");
}

void setup() {
  pinMode(pinMotorEnable, OUTPUT);
  pinMode(pinMotorIN1, OUTPUT);
  pinMode(pinMotorIN2, OUTPUT);
  stopMotor();

  direccion.attach(pinServo);
  direccion.write(centroFisicoServo);

  Serial.begin(115200);
  Serial.println("Prueba de direccion lista");
  Serial.println("Comandos logicos: c=90, l=79, r=101, o 64-124");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command == "c" || command == "C") {
    writeLogicalAngle(centroLogico);
  } else if (command == "l" || command == "L") {
    writeLogicalAngle(79);
  } else if (command == "r" || command == "R") {
    writeLogicalAngle(101);
  } else {
    int logicalAngle = command.toInt();
    int servoAngle = logicalToServoAngle(logicalAngle);
    if (servoAngle >= limiteFisicoMinimo && servoAngle <= limiteFisicoMaximo) {
      writeLogicalAngle(logicalAngle);
    } else {
      Serial.println("Comando invalido. Usa c, l, r o un angulo logico entre 64 y 124");
    }
  }
}
