#include "motor.h"
#include "clock.h"

Motor::Motor(int pwm, int dir, int enable, int max, int channel, int obc) : klokke(0) {
    pwmPin = pwm;
    dirPin = dir;
    maxPwm = max;
    obcPin = obc;
    enablePin = enable;
    pwmChannel = channel;
    maxTime = 60; // makstid i minutter, motor skrur seg av etter maxTime, og går ikke på igjen
    startTid = 0;
    stoppTid = 0; //sjekker når motor stopper
    satStart = 0;
    state = RUNNING;

}

void Motor::begin() {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);
    pinMode(dirPin, OUTPUT);
    pinMode(obcPin, INPUT);
    // Setup PWM channel
    ledcSetup(pwmChannel, 5000, 8); 
    ledcAttachPin(pwmPin, pwmChannel);
    startTid = klokke.getTime();
    
}

void Motor::stop() {
    ledcWrite(pwmChannel, 255); // Sender MAX value isteden for for 0 (invertert logikk)
    digitalWrite(dirPin, LOW);
    digitalWrite(enablePin,LOW);
    // stoppTid = klokke.getTime();
    
}

void Motor::setSpeed(int speed) {
    if (state == PERMANENTLY_STOPPED || state == SATURATED) {
        return;
    }

    speed = constrain(speed, -maxPwm, maxPwm);

    if (abs(speed) == 0) {
        stop();
        return;
    }

    if (abs(speed) == maxPwm) {
        if (satStart == 0) {
            satStart = klokke.getTime();
        }
        if (klokke.getTime() - satStart >= 5e6) {
            state = SATURATED;
            stop();
            stoppTid = klokke.getTime();
            Serial.println("State: SATURATED (maxpwm reached for over 10 secs)");
            return;
        } 
    } else {
        satStart = 0;
        
    }

    ledcWrite(pwmChannel, 255-abs(speed));
    digitalWrite(dirPin, speed < 0 ? LOW : HIGH); 
    // analogWrite(pwmPin, int(abs(speed)));
    digitalWrite(enablePin, HIGH);
}

bool Motor::checkAndStop() {

    if (state == PERMANENTLY_STOPPED) {
        return true;
    }
    double naatid= klokke.convertTime();
   // Serial.printf("Naatid: %f, maxTid: %i\n", naatid, maxTime);
    if (naatid >= maxTime) {
        stop();
        state = PERMANENTLY_STOPPED;
        Serial.println("\tstate: PERMANENTLY STOPPED, maxtime reached");
        return true;
    }
    
    // sjekker om signal fra OBC er høyt
    if (digitalRead(obcPin) == HIGH) {
        if (state != TEMPORARILY_STOPPED) {
            stop();
            state = TEMPORARILY_STOPPED;
            Serial.println("\tstate: TEMPORARILY STOPPED, obc signal detected");
        }
       
        return true;
    // skrur på motor igjen dersom signal fra OBC blir lavt igjen
    } else if (state == TEMPORARILY_STOPPED) {
        state = RUNNING;
        Serial.println("state: running again, low signal from obc detected");
        begin();
        return false;
    }
   
    // sjekker om motor er i metning
    if (state == SATURATED) {
        if (klokke.getTime() - stoppTid >= 120e6) {
            state = RUNNING;
            Serial.println("\tState: RUNNING -> recovered from saturation");
           begin();
           return false;
       }
      return true;
   } 
    return false;
}

