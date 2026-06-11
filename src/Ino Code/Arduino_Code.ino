#include <Servo.h>
#include <stdlib.h> 

Servo direccion;

// ================= PINES =================
const int pinServo = 8;
const int pinMotorPWM = 7;  
const int pinMotorDir1 = 9; // Motor IN1
const int pinMotorDir2 = 10; // Motor IN2

// Único Ultrasonido Funcional (Estacionamiento)
const int pinTrig = 3;
const int pinEcho = 11;

// ================= VARIABLES =================
int distanciaUS = 200; 
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

int velocidadAuto = 0;
int anguloServo = 86;
unsigned long previousMillisUS = 0;

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
    if (currentMillis - previousMillisUS >= 50) {
        previousMillisUS = currentMillis;
        leerUltrasonido();
        Serial.print("US:");
        Serial.println(distanciaUS);
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

void leerUltrasonido() {
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    
    long duration = pulseIn(pinEcho, HIGH, 12000); 
    if (duration == 0) {
        distanciaUS = 200; 
    } else {
        distanciaUS = duration * 0.034 / 2;
    }
}