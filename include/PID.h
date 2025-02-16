#ifndef PID_H
#define PID_H
#include "arduino.h"


class PID {
private:
    float Kp, Ki, Kd;
    float prevError, integral, reference_;
    

public:
    PID(float p, float i, float d);
    float compute(float measured, uint64_t dt);
    void setReference(float reference);
};

#endif