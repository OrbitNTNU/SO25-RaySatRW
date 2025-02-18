#include "motor.h"

Motor::Motor(int pwm, int dir,int enable, int max, int channel) {
    pwmPin = pwm;
    dirPin = dir;
    maxPwm = max;
    enablePin = enable;
    pwmChannel = channel;
}

void Motor::begin() {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin,LOW);
    
    pinMode(dirPin, OUTPUT);
    // Setup PWM channel
    ledcSetup(pwmChannel, 5000, 8); 
    ledcAttachPin(pwmPin, pwmChannel);
    // Enable motor (HIGH = Enabled, LOW = Disabled)
    // stop();
}

void Motor::stop() {
    ledcWrite(pwmChannel, 255); // Send MAX value instead of 0 (inverted logic)
    digitalWrite(dirPin, LOW);
    digitalWrite(enablePin,LOW);
    return;
}

void Motor::setSpeed(int speed) {
    speed = constrain(speed, -maxPwm, maxPwm);
    if (abs(speed) == 0) {
        stop();
        return;
    }
    ledcWrite(pwmChannel, 255-abs(speed));
    // FIXME: < must be switched to >, this is like this for testing
    digitalWrite(dirPin,speed < 0 ? LOW : HIGH); 
    // analogWrite(pwmPin, int(abs(speed)));
    digitalWrite(enablePin,HIGH);
}

