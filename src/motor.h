
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  int pwmPin;
  int dirPin; 
  int maxPwm; 
  int pwmChannel;

public:
  Motor(int pwm, int dir, int max, int channel) : pwmPin(pwm), dirPin(dir), maxPwm(max), pwmChannel(channel) {}
  void begin() {
        pinMode(dirPin, OUTPUT);
        ledcSetup(pwmChannel, 25000, 8); // 25 kHz, 8-bit
        ledcAttachPin(pwmPin, pwmChannel);
    }
  void setSpeed(int speed) {
        speed = constrain(speed, -maxPwm, maxPwm);
        ledcWrite(pwmChannel, abs(speed));
        digitalWrite(dirPin, (speed >= 0) ? HIGH : LOW);
    }
};


#endif

