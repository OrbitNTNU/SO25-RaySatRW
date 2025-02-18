#include "PID.h"
#include "MPU6050.h"


PID::PID(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
    reference_ = 0;
    prevError = 0;
    integral = 0;
}

float PID::compute(float measured, uint64_t dt) {
    float newdt = dt / 1e6;
    // Serial.print(newdt); Serial.print(" ");
    float error = reference_ - measured;
    integral += error * newdt;
    float derivative = (error - prevError) / newdt;
    prevError = error;
    // Serial.print(" pe: "); Serial.print(prevError);
    return Kp * error + Ki * integral + Kd * derivative;
}
void PID::setReference(float reference) {
    reference_ = reference;
}