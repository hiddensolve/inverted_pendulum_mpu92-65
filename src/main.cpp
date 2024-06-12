#include <Wire.h>
#include "MPU9250.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define MOTOR_PIN1 14    // Connect to IN1 of L298N
#define MOTOR_PIN2 12    // Connect to IN2 of L298N
#define ENA_PIN 32       // Connect to ENA of L298N

MPU9250 IMU(Wire, 0x68); // MPU9250 object
int status;

const float ACCEL_THRESHOLD = 0.5; // Threshold for motor activation based on accelerometer reading
const float ANGLE_THRESHOLD = 5.0;  // Threshold for distinguishing between inclined and horizontal
const int MOTOR_SPEED = 255;       // Motor speed (0-255)

float angle = 0; // Initial angle estimate
unsigned long lastTime = 0; // Last time gyro data was read

void setup() {
  Serial.begin(921600);

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C communication with specified pins
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
}

void loop() {
  IMU.readSensor();

   float gyroX = IMU.getGyroX_rads(); // Get angular velocity around X-axis (in radians per second)

  float accelY = IMU.getAccelY_mss();

 

  static float angleX = 0.0; // Initialize the angle variable and make it static to retain its value between function calls

  // Integrate angular velocity to get change in angle (in radians) over a small time interval (here, 100 milliseconds)
  angleX += gyroX * 0.1; // 0.1 seconds = 100 milliseconds (time interval)
 if(accelY < 1 && accelY > -1){
  angleX = 0;
  }
  // Convert angle from radians to degrees for easier interpretation
  float angle = angleX * (180.0 / PI);

  // Get accelerometer reading along the Y-axis
  

  // Determine motor direction based on angle estimate and accelerometer reading
  if (abs(angle) > ANGLE_THRESHOLD && accelY > ACCEL_THRESHOLD) {
    // If angle is above threshold and accelerometer indicates inclination
    // Rotate motor in one direction
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
  } else if (abs(angle) > ANGLE_THRESHOLD && accelY < -ACCEL_THRESHOLD) {
    // If angle is above threshold and accelerometer indicates opposite inclination
    // Rotate motor in the other direction
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
  } else {
    // Stop the motor
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
  }

  // Set motor speed
  analogWrite(ENA_PIN, MOTOR_SPEED);

  // Print sensor data
  Serial.print("Angle: "); Serial.print(angle);
  Serial.print("\tAccX: "); Serial.print(accelY);
  Serial.print("\tMotor Direction: ");
  if (abs(angle) > ANGLE_THRESHOLD && accelY > ACCEL_THRESHOLD) {
    Serial.println("Forward");
  } else if (abs(angle) > ANGLE_THRESHOLD && accelY < -ACCEL_THRESHOLD) {
    Serial.println("Backward");
  } else {
    Serial.println("Stop");
  }

  delay(100);
}
