#ifndef PID_H
#define PID_H
#include "arduino.h"


class PID {
private:
    float Kp, Ki, Kd;
    float prevError, integral, reference_;
    int outputMax_, outputMin;
    float prevOutput;
    float lowPass(float e);

public:
    PID(float p, float i, float d, int outputMax);
    float compute(float measured, uint64_t dt);
    void setReference(float reference);
};

#endif