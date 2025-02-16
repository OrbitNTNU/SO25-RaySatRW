
#include "Motor.h"

void Motor::begin() {
    pinMode(dirPin, OUTPUT);
    ledcSetup(pwmChannel, 25000, 8); 
    ledcAttachPin(pwmPin, pwmChannel);
}

void Motor::setSpeed(int speed) {
    speed = constrain(speed, -maxPwm, maxPwm);
    ledcWrite(pwmChannel, abs(speed));
    digitalWrite(dirPin, (speed >= 0) ? HIGH : LOW);
}

