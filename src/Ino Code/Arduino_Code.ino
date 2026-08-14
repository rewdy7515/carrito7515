#include <Servo.h>
#include <stdlib.h> 

Servo direccion;

// ================= PINES =================
const int pinServo = 8;
const int pinMotorPWM = 7;  
const int pinMotorDir1 = 9; // Motor IN1
const int pinMotorDir2 = 10; // Motor IN2

// Sensor de distancia funcional para estacionamiento
const int pinTrig = 3;
const int pinEcho = 11;

// ================= VARIABLES =================
int distanciaSensor = 200; 
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

int velocidadAuto = 0;
int anguloServo = 86;
unsigned long previousMillisSensor = 0;

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

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseData();
        ejecutarMovimiento();
        newData = false;
    }
    
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisSensor >= 50) {
        previousMillisSensor = currentMillis;
        leerDistancia();
        Serial.print("DIST:");
        Serial.println(distanciaSensor);
    }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) { ndx = numChars - 1; }
            } else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char * strtokIndx;
    strtokIndx = strtok(tempChars, ",");
    if(strtokIndx != NULL) {
        int velTemp = atoi(strtokIndx);     
        strtokIndx = strtok(NULL, ","); 
        if(strtokIndx != NULL) {
            int angTemp = atoi(strtokIndx); 
            
            if (velTemp < 0) velTemp = 0; 
            if (velTemp > 255) velTemp = 255;
            velocidadAuto = velTemp;
            
            if (angTemp >= 60 && angTemp <= 120) {
                anguloServo = angTemp;
            }
        }
    }
}

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

void leerDistancia() {
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    
    long duration = pulseIn(pinEcho, HIGH, 12000); 
    if (duration == 0) {
        distanciaSensor = 200; 
    } else {
        distanciaSensor = duration * 0.034 / 2;
    }
}
