#include "PID.h"
#include "MPU6050.h"


PID::PID(float p, float i, float d, int outputMax) {
    Kp = p;
    Ki = i;
    Kd = d;
    reference_ = 0;
    prevError = 0;
    integral = 0;
    outputMax = outputMax;
    outputMin = -outputMax;    
}
#define LOW_PASS_ALPHA 0.5
float PID::lowPass(float e) {
    return LOW_PASS_ALPHA*e+(1-LOW_PASS_ALPHA)*prevError;
}
float PID::compute(float measured, uint64_t dt) {
    float newdt = dt / 1e6;
    // Serial.print(newdt); Serial.print(" ");
    float error = lowPass(reference_ - measured);
    // If not in saturation, integrate
    if (prevOutput < outputMax && prevOutput > outputMin) {
        integral += error * newdt;
    }
    float derivative = (error - prevError) / newdt;
    prevError = error;
    // Serial.print(" pe: "); Serial.print(prevError);
    return Kp * error + Ki * integral + Kd * derivative;
}
void PID::setReference(float reference) {
    reference_ = reference;
}