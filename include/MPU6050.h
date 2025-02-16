
#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "clk.h"

class MPU6050Sensor {
private:
    Adafruit_MPU6050 mpu;
    float gyroZ_offset;

public:
    MPU6050Sensor();
    void begin();
    void calibrate();
    float getGyroZ();
    void printSensorData();
};

#endif
