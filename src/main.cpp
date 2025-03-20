#include <Arduino.h>
#include "MPU6050.h"
#include "motor.h"
#include "PID.h"
#include "clock.h"

// Definer pinner til motoren
#define PWM_PIN 18
#define DIR_PIN 4
#define ENABLE_PIN 5
#define MAX_PWM 250  
#define OBC_PIN 13 

// PID-konstanter //kan fintunes mer? 
#define Kp 80.0
#define Ki 5.0
#define Kd 1.0

MPU6050Sensor sensor;
Motor motor(PWM_PIN, DIR_PIN, ENABLE_PIN, MAX_PWM, 0, OBC_PIN);
PID pid(Kp, Ki, Kd,MAX_PWM);
Klokke klokke(1);

void setup() {
    Serial.begin(115200);
    motor.begin();   // Sett opp motorpinner
    sensor.begin();  // Start og kalibrer gyroskopet
    delay(100);
}

void loop() {
    // sjekker om makstid er nådd eller motor er i metning
    if (motor.checkAndStop()) {
        motor.stop(); // overfladisk, kan fjerne
        return;
    }
    

    //sjekker signal fra OBC, høyt signal skrur av motoren (også overflatisk, gjøres allerede i checkAndStop())
    if (digitalRead(OBC_PIN) == HIGH) {
        motor.stop();
        return;
    }
    

    float gyroZ = sensor.getGyroZ(); // [rad/s]
    uint64_t dt = klokke.getDT(); 
    float u = pid.compute(gyroZ, dt);  // PID-kontroller beregner pådrag
   
    // Kjør motor basert på PID-utgang
    motor.setSpeed(int(u));
    Serial.print("pådrag: "); Serial.print(u);


    delay(50);  
}


