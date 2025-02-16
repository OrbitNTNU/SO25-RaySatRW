#include "motor.h"

Motor::Motor(int pwm, int dir,int enable, int max, int channel) {
    pwmPin = pwm;
    dirPin = dir;
    maxPwm = max;
    enablePin = enable;
    pwmChannel = channel;
}

void Motor::begin() {
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    // Setup PWM channel
    ledcSetup(pwmChannel, 5000, 8); 
    ledcAttachPin(pwmPin, pwmChannel);
    // Enable motor (HIGH = Enabled, LOW = Disabled)
    digitalWrite(enablePin,HIGH);
}

void Motor::setSpeed(int speed) {
    speed = constrain(speed, -maxPwm, maxPwm);
    ledcWrite(pwmChannel, abs(speed));
    uint8_t direction = LOW*(speed < 0) + HIGH*(speed >= 0);
    digitalWrite(dirPin,direction);
    analogWrite(pwmPin, abs(speed));
}

