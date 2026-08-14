// Prueba manual del motor mediante el L298N.
// No controla el servo, el HC-SR04 ni el encoder.
//
// IMPORTANTE:
// - Mantener el carrito elevado y las ruedas libres.
// - Esta prueba usa los pines actualmente documentados.
// - D7 se usa solo como enable ON/OFF durante esta prueba; no se prueba PWM.
// - En el Serial Monitor enviar: f = avance, r = reversa, s = parada.

const int pinMotorEnable = 7;
const int pinMotorIN1 = 9;
const int pinMotorIN2 = 10;

void stopMotor() {
  digitalWrite(pinMotorEnable, LOW);
  digitalWrite(pinMotorIN1, LOW);
  digitalWrite(pinMotorIN2, LOW);
}

void moveForward() {
  digitalWrite(pinMotorIN1, HIGH);
  digitalWrite(pinMotorIN2, LOW);
  digitalWrite(pinMotorEnable, HIGH);
}

void moveReverse() {
  digitalWrite(pinMotorIN1, LOW);
  digitalWrite(pinMotorIN2, HIGH);
  digitalWrite(pinMotorEnable, HIGH);
}

void setup() {
  pinMode(pinMotorEnable, OUTPUT);
  pinMode(pinMotorIN1, OUTPUT);
  pinMode(pinMotorIN2, OUTPUT);

  stopMotor();
  Serial.begin(115200);
  Serial.println("Motor test listo: f=avance, r=reversa, s=parada");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  char command = Serial.read();

  if (command == 'f' || command == 'F') {
    moveForward();
    Serial.println("AVANCE");
  } else if (command == 'r' || command == 'R') {
    moveReverse();
    Serial.println("REVERSA");
  } else if (command == 's' || command == 'S') {
    stopMotor();
    Serial.println("PARADA");
  } else if (command != '\n' && command != '\r') {
    stopMotor();
    Serial.println("Comando invalido: motor detenido");
  }
}

