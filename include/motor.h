
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <clock.h>

enum MotorState {
  RUNNING,
  SATURATED,
  PERMANENTLY_STOPPED,
};

class Motor {
private:
  int pwmPin;
  int dirPin; 
  int enablePin;
  int maxPwm; 
  int maxTime;
  int pwmChannel;
  int obcPin;
  double startTid;
  double stoppTid; 
  double satStart;
  

public:
  Motor(int pwm, int dir, int enable, int max, int channel, int obc);
  void begin();
  void stop();
  void setSpeed(int speed);
  bool checkAndStop();
  Klokke klokke;
  //bool isStopped;
  
  MotorState state;
  
};


#endif

