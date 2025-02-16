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
    float error = reference_ - measured;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}
void PID::setReference(float reference) {
    reference_ = reference;
}