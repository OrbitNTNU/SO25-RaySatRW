#include <Wire.h>
// PID Control Parameters
float Kp = 1.0;  // Reduced proportional gain
float Ki = 0.0;  // Keep integral zero for now
float Kd = 0.1;  // Lower derivative gain
float previous_error = 0;
float integral = 0;
#define PWM_PIN 18     // Motor PWM signal
#define DIR_PIN 19     // Motor direction control
#define ENABLE_PIN 5   // Motor enable pin
const int MPU6050_ADDR = 0x68; // MPU6050 I2C address
float gyroX_offset = -2.30;    // X-axis offset
float gyroY_offset = 1.00;     // Y-axis offset
float gyroZ_offset = -3.40;    // Z-axis offset
const int threshold = 2;  // Ignore movement below 2°/s
const int maxPWM = 150;   // Limit motor speed for safety
const int minPWM = 30;    // Lower minimum PWM to avoid unwanted motion
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
    // Setup Motor Control
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    ledcSetup(0, 5000, 8);  // PWM Channel 0, 5kHz, 8-bit resolution
    ledcAttachPin(PWM_PIN, 0);
    digitalWrite(ENABLE_PIN, HIGH); // Enable motor
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Set to 0 to wake up the sensor
    Wire.endTransmission();
    Serial.println(":rocket: System Initialized.");
    Serial.println(":satellite_antenna: Reading Gyro Data...");
}
void loop() {
    float gyroX = getGyroX(); // Read X-axis angular velocity
    // Step 1: Print Gyroscope Data
    Serial.print("Gyro X: ");
    Serial.print(gyroX);
    Serial.println(" °/s");
    // SAFETY CHECK: Stop motor if gyro values are too high
    if (abs(gyroX) > 100) {
        Serial.println(":warning: WARNING: Gyro data too high! Stopping motor.");
        stopMotor();
        delay(1000);
        return;
    }
    // Step 2: Ignore small disturbances
    if (abs(gyroX) < threshold) {
        Serial.println(":white_check_mark: Gyro stable - No correction needed");
        stopMotor();
        return;
    }
    // Step 3: Compute PID Correction
    float correction = computePID(gyroX);
    // **NEW FIX:** Ignore small PID corrections
    if (abs(correction) < 10) {
        Serial.println(":white_check_mark: PID correction too small - Stopping motor");
        stopMotor();
        return;
    }
    // Step 4: Print PID Output
    Serial.print(":arrows_counterclockwise: PID Output (Correction): ");
    Serial.println(correction);
    // Step 5: Control Motor
    controlMotor(correction);
    delay(50); // Short delay for stability
}
// Function to read gyro X-axis from MPU6050
float getGyroX() {
    int16_t rawX;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // Gyroscope X-axis register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    rawX = Wire.read() << 8 | Wire.read();
    float gyroX_DPS = (rawX / 131.0) - gyroX_offset;
    // Clamp gyro values to prevent instability
    if (gyroX_DPS > 100) gyroX_DPS = 100;
    if (gyroX_DPS < -100) gyroX_DPS = -100;
    return gyroX_DPS;
}
// PID Controller Function
float computePID(float error) {
    float dt = 0.05; // Loop time (50ms)
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previous_error = error;
    return output;
}
// Motor Control Function (NOW INVERTS PWM SIGNAL)
void controlMotor(float speed) {
    int pwmValue = constrain(abs(speed), minPWM, maxPWM); // Ensure PWM is within safe range
    Serial.print(":zap: Motor Direction: ");
    Serial.print(speed > 0 ? "Counteract Clockwise" : "Counteract Counterclockwise");
    Serial.print(" | PWM Speed (Inverted): ");
    Serial.println(255 - pwmValue);
    digitalWrite(DIR_PIN, speed > 0 ? LOW : HIGH); // Set direction
    ledcWrite(0, 255 - pwmValue); // Invert PWM signal
}
// Stop the Motor Function
void stopMotor() {
    Serial.println(":black_square_for_stop: Motor Stopped.");
    ledcWrite(0, 255); // Send MAX value instead of 0 (inverted logic)
    digitalWrite(DIR_PIN, LOW);
}