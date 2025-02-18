#include <Arduino.h>
#include "MPU6050.h"
#include "motor.h"
#include "PID.h"
#include "clock.h"

// Definer pinner til motoren
#define PWM_PIN 18
#define DIR_PIN 19
#define ENABLE_PIN 5
#define MAX_PWM 50  

// PID-konstanter (juster disse for å finstemme styringen)
#define Kp 100.0
#define Ki 20.0
#define Kd 10.0

MPU6050Sensor sensor;
Motor motor(PWM_PIN, DIR_PIN,ENABLE_PIN, MAX_PWM,0);
PID pid(Kp, Ki, Kd,MAX_PWM);
Klokke klokke(1);

void setup() {
    Serial.begin(115200);
    motor.begin();   // Sett opp motorpinner
    sensor.begin();  // Start og kalibrer gyroskopet
    delay(100);
}

void loop() {
    // Hent sensorverdi
    float gyroZ = sensor.getGyroZ();  // [rad/s]
    uint64_t dt = klokke.getDT(); 
    // Beregn PID-utgang
    float u = pid.compute(gyroZ,dt);  // PID-kontroller beregner pådrag
    // Serial.print("Gyro: ");
    // Serial.print(gyroZ);
    // Serial.print(" PID: ");
    // Serial.print(int(u));
    // Serial.print("\t"); Serial.println(u);
    // Kjør motor basert på PID-utgang
    motor.setSpeed(int(u));
    // motor.setSpeed(1); delay(5000); motor.setSpeed(10); delay(5000); motor.setSpeed(20); delay(5000); motor.setSpeed(30); delay(5000); motor.setSpeed(40); delay(5000); motor.setSpeed(50); delay(5000); motor.setSpeed(0); delay(5000);
    // motor.setSpeed(125);
    // Serial.println("Posetiv :)");
    // delay(7000);
    // motor.setSpeed(-125);
    // Serial.println("negativ :(");
    // delay(7000);


    // Debugging i Serial Monitor
    // Serial.print("Gyro Z: ");
    // Serial.print(gyroZ);
    // Serial.print(" | PID Output: ");
    // Serial.println(pidOutput);

    delay(50);  // Kjør loopen raskt for responsiv kontroll
}