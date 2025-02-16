#include "PID.h"

PID::PID(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
    prevError = 0;
    integral = 0;
}

float PID::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}