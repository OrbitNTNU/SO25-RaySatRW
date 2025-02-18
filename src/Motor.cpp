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

void Motor::stop() {
    ledcWrite(pwmChannel, 255); // Send MAX value instead of 0 (inverted logic)
    digitalWrite(dirPin, LOW);
    digitalWrite(enablePin,LOW);
    return;
}

void Motor::setSpeed(int speed) {
    speed = constrain(speed, -maxPwm, maxPwm);
    if (abs(speed) == 0) {
        Serial.println("stop");
        stop();
        return;
    }
    Serial.print("speed "); Serial.println(speed);
    digitalWrite(enablePin,HIGH);
    ledcWrite(pwmChannel, 255-abs(speed));
    digitalWrite(dirPin,speed > 0 ? LOW : HIGH);
    analogWrite(pwmPin, int(abs(speed)));
}

