// Firmware base de movimiento para Arduino Uno.
//
// Protocolo serial a 115200 baudios:
//   <velocidad,angulo_logico>\n
// Ejemplos:
//   <120,92>    avance, dirección recta
//   <-120,92>   reversa, dirección recta
//   <120,75>    avance con dirección izquierda
//   <0,92>      parada y dirección recta
//
// El ángulo lógico 92 corresponde al centro físico recto verificado de 92 grados.

#include <Servo.h>

const int pinServo = 8;
const int pinMotorEnable = 7;
const int pinMotorIN1 = 9;
const int pinMotorIN2 = 10;

const int centroLogico = 92;
const int centroFisicoServo = 92;
// Modo temporal de prueba mecánica. Dejar en 0 para operación normal.
// En 1 se permite explorar el rango Servo.write() 0..180; no es un límite
// mecánico seguro y requiere supervisión inmediata con el robot detenido.
#define SERVO_LIMIT_TEST_MODE 1
const int limiteFisicoMinimo = SERVO_LIMIT_TEST_MODE ? 0 : 60;
const int limiteFisicoMaximo = SERVO_LIMIT_TEST_MODE ? 180 : 120;
const int velocidadMaxima = 255;

const unsigned long communicationTimeoutMs = 500;
// D7 no tiene PWM por hardware en Arduino Uno. Se usa un PWM por software
// sobre el mismo pin para que <45,...> entregue menos potencia que <255,...>
// sin modificar el cableado actual del L298N.
const unsigned long motorPwmPeriodUs = 5000;

Servo direccion;
unsigned long lastValidCommandMs = 0;
int motorTargetDuty = 0;

int logicalToServoAngle(int logicalAngle) {
  return centroFisicoServo + (logicalAngle - centroLogico);
}

bool writeLogicalAngle(int logicalAngle) {
  int servoAngle = logicalToServoAngle(logicalAngle);

  if (servoAngle < limiteFisicoMinimo || servoAngle > limiteFisicoMaximo) {
    return false;
  }

  direccion.write(servoAngle);
  return true;
}

void stopMotor() {
  motorTargetDuty = 0;
  digitalWrite(pinMotorEnable, LOW);
  digitalWrite(pinMotorIN1, LOW);
  digitalWrite(pinMotorIN2, LOW);
}

void updateMotorEnable() {
  if (motorTargetDuty <= 0) {
    digitalWrite(pinMotorEnable, LOW);
    return;
  }

  unsigned long highTimeUs = (motorPwmPeriodUs * motorTargetDuty) / velocidadMaxima;
  unsigned long cyclePositionUs = micros() % motorPwmPeriodUs;
  digitalWrite(pinMotorEnable, cyclePositionUs < highTimeUs ? HIGH : LOW);
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -velocidadMaxima, velocidadMaxima);

  if (speed == 0) {
    stopMotor();
    return;
  }

  if (speed > 0) {
    digitalWrite(pinMotorIN1, HIGH);
    digitalWrite(pinMotorIN2, LOW);
  } else {
    digitalWrite(pinMotorIN1, LOW);
    digitalWrite(pinMotorIN2, HIGH);
  }

  motorTargetDuty = abs(speed);
}

bool parseMotionCommand(const String& command, int& speed, int& logicalAngle) {
  if (!command.startsWith("<") || !command.endsWith(">")) {
    return false;
  }

  int commaIndex = command.indexOf(',');
  if (commaIndex < 2 || commaIndex >= command.length() - 1) {
    return false;
  }

  String speedText = command.substring(1, commaIndex);
  String angleText = command.substring(commaIndex + 1, command.length() - 1);

  speed = speedText.toInt();
  logicalAngle = angleText.toInt();

  int servoAngle = logicalToServoAngle(logicalAngle);
  return servoAngle >= limiteFisicoMinimo && servoAngle <= limiteFisicoMaximo;
}

void processCommand(String command) {
  command.trim();

  int speed = 0;
  int logicalAngle = centroLogico;

  if (parseMotionCommand(command, speed, logicalAngle)) {
    writeLogicalAngle(logicalAngle);
    setMotorSpeed(speed);
    lastValidCommandMs = millis();
    Serial.println("OK");
    return;
  }

  // Comandos manuales conservados para pruebas desde el monitor serial.
  if (command == "f" || command == "F") {
    writeLogicalAngle(centroLogico);
    setMotorSpeed(120);
    lastValidCommandMs = millis();
    Serial.println("AVANCE");
  } else if (command == "r" || command == "R") {
    writeLogicalAngle(centroLogico);
    setMotorSpeed(-120);
    lastValidCommandMs = millis();
    Serial.println("REVERSA");
  } else if (command == "s" || command == "S") {
    stopMotor();
    lastValidCommandMs = millis();
    Serial.println("PARADA");
  } else if (command == "c" || command == "C") {
    writeLogicalAngle(centroLogico);
    lastValidCommandMs = millis();
    Serial.println("CENTRO");
  } else {
    stopMotor();
    Serial.println("Comando invalido");
  }
}

void setup() {
  pinMode(pinMotorEnable, OUTPUT);
  pinMode(pinMotorIN1, OUTPUT);
  pinMode(pinMotorIN2, OUTPUT);
  stopMotor();

  direccion.attach(pinServo);
  writeLogicalAngle(centroLogico);

  Serial.begin(115200);
  Serial.setTimeout(10);
  lastValidCommandMs = millis();
  Serial.println("Firmware de movimiento listo");
}

void loop() {
  updateMotorEnable();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  if (millis() - lastValidCommandMs > communicationTimeoutMs) {
    stopMotor();
  }

  updateMotorEnable();
}
