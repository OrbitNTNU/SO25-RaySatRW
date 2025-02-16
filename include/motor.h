
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  int pwmPin;
  int dirPin; 
  int maxPwm; 
  int pwmChannel;
  int enablePin;

public:
  Motor(int pwm, int dir, int enable, int max, int channel);
  void begin();
  void setSpeed(int speed);
};


#endif

