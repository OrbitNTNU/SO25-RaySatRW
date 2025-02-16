#include <Arduino.h>
#include "MPU6050.h"
#include "motor.h"
#include "PID.h"
#include "clock.h"

// Definer pinner til motoren
#define PWM_PIN 18
#define DIR_PIN 19
#define ENABLE_PIN 5
#define MAX_PWM 255  

// PID-konstanter (juster disse for å finstemme styringen)
#define Kp 0.0
#define Ki 0.0
#define Kd 0.0

MPU6050Sensor sensor;
Motor motor(PWM_PIN, DIR_PIN,ENABLE_PIN, MAX_PWM,1);
PID pid(Kp, Ki, Kd);
Klokke klokke(1);

void setup() {
    Serial.begin(115200);
    // sensor.begin();  // Start og kalibrer gyroskopet
    // motor.begin();   // Sett opp motorpinner
}

void loop() {
    // Hent sensorverdi
    float gyroZ = sensor.getGyroZ();  // Gyrodata for rotasjon
    uint64_t dt = klokke.getDT();
    // Beregn PID-utgang
    float u = pid.compute(gyroZ,dt);  // PID-kontroller beregner pådrag
    Serial.print("Gyro: ");
    Serial.print(gyroZ);
    Serial.print(" PID: ");
    Serial.println(u);
    // Kjør motor basert på PID-utgang
    // motor.setSpeed(pidOutput);
    // motor.setSpeed(125);
    // delay(7000);
    // motor.setSpeed(-125);


    // Debugging i Serial Monitor
    // Serial.print("Gyro Z: ");
    // Serial.print(gyroZ);
    // Serial.print(" | PID Output: ");
    // Serial.println(pidOutput);

    delay(1000);  // Kjør loopen raskt for responsiv kontroll
}