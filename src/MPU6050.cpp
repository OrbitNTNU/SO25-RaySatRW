#include "MPU6050.h"

MPU6050Sensor::MPU6050Sensor() : gyroZ_offset(0) {}  // Initialiser offset til 0

void MPU6050Sensor::begin() {
    Serial.begin(115200);
    Serial.println("Starter MPU6050...");

    if (!mpu.begin()) {
        Serial.println("Feil: Fant ikke MPU6050!");
        while (1) delay(10);
    }

    Serial.println("MPU6050 funnet!");
    calibrate(); // Kjør kalibrering én gang

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void MPU6050Sensor::calibrate() {
    Serial.println("Kalibrerer MPU6050...");
    int numSamples = 1000;
    gyroZ_offset = 0;

    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyroZ_offset += g.gyro.z;
        delay(1);
    }

    gyroZ_offset /= numSamples;
    Serial.print("Gyro Z offset: ");
    Serial.println(gyroZ_offset, 6);
    Serial.println("Kalibrering fullført!");
}

float MPU6050Sensor::getGyroZ() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return g.gyro.z - gyroZ_offset;  // Kompenserer for offset
}

void MPU6050Sensor::printSensorData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z - gyroZ_offset);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("");
}