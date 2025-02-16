#include <Arduino.h>
#include "MPU6050.h"
#include "motor.h"
#include "PID.h"

// Definer pinner til motoren
#define PWM_PIN 5
#define DIR_PIN 6
#define MAX_PWM 255  

// PID-konstanter (juster disse for å finstemme styringen)
#define Kp 0.0
#define Ki 0.0
#define Kd 0.0

MPU6050Sensor sensor;
Motor motor(PWM_PIN, DIR_PIN, MAX_PWM);
PID pid(Kp, Ki, Kd);

void setup() {
    Serial.begin(115200);
    sensor.begin();  // Start og kalibrer gyroskopet
    motor.begin();   // Sett opp motorpinner
    pid.setSetpoint(0);  // Ønsket stabil posisjon er 0 rad/s
}

void loop() {
    // Hent sensorverdi
    float gyroZ = sensor.getGyroZ();  // Gyrodata for rotasjon

    // Beregn PID-utgang
    float pidOutput = pid.compute(gyroZ);  // PID-kontroller beregner korreksjon

    // Kjør motor basert på PID-utgang
    motor.setSpeed(pidOutput);

    // Debugging i Serial Monitor
    Serial.print("Gyro Z: ");
    Serial.print(gyroZ);
    Serial.print(" | PID Output: ");
    Serial.println(pidOutput);

    delay(10);  // Kjør loopen raskt for responsiv kontroll
}