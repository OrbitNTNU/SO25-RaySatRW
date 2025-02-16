#ifndef PID_H
#define PID_H


class PID {
private:
    float Kp, Ki, Kd;
    float prevError, integral;

public:
    PID(float p, float i, float d);
    float compute(float setpoint, float measured, float dt);
};

#endif