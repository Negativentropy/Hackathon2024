#include "IMU.h"
#include <Servo.h>

// Loop time and complementary filter constant
#define DT  0.02
#define AA  0.99

// PID Variables
float previousErrorX = 0, integralX = 0;
float previousErrorY = 0, integralY = 0;
float SetpointX = 90; // Default target angle for X, assuming mid-range as default
float SetpointY = 90; // Default target angle for Y, assuming mid-range as default
float Kp = 1; 
float Ki = 0; 
float Kd = 0; 


// IMU data variables
byte buff[6];
int accRaw[3], magRaw[3], gyrRaw[3];
float rate_gyr_x = 0.0, rate_gyr_y = 0.0, rate_gyr_z = 0.0;
float gyroXangle = 0.0, gyroYangle = 0.0, gyroZangle = 0.0;
float AccXangle = 0.0, AccYangle = 0.0;
float CFangleX = 0.0, CFangleY = 0.0;

// Servo objects for X and Y axes
Servo servoX;
Servo servoY;
Servo motorSpeed;

String inputString = ""; // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

float computePID(float setpoint, float actualPosition, float &integral, float &previousError) {
    float error = setpoint - actualPosition;
    integral += error * DT;
    float derivative = (error - previousError) / DT;
    float output = -Kp * error + -Ki * integral + -Kd * derivative;
    previousError = error;
    return output;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  detectIMU();
  enableIMU();
  
  // Attach servos to their pins
  servoX.attach(5);
  servoY.attach(6);
  motorSpeed.attach(3, 500, 2400);  // Assuming motorSpeed is a servo controlling speed

  inputString.reserve(200); // reserve 200 bytes for the inputString
}

void loop() {
  // Read the measurements from sensors
  readIMU(); // Continues as previously defined

  // Update PID constants from analog inputs
  Kp = analogRead(A0) / 200.0;
  Ki = analogRead(A1) / 200.0;
  Kd = analogRead(A2) / 200.0;

  int MotorSpeedValue = map(analogRead(A3), 0, 1023, 0, 100);
  motorSpeed.write(MotorSpeedValue);

  // Check for new serial input and process it
  if (Serial.available() > 0) {
    // Read the incoming string until a newline is received
    inputString = Serial.readStringUntil('\n');

    // Check if the string is meant for setting the X angle
    if (inputString.startsWith("x")) {
      SetpointX = inputString.substring(1).toInt();
      SetpointX = constrain(SetpointX, 45, 135); // Ensure the target angle is within bounds
    }
    // Check if the string is meant for setting the Y angle
    else if (inputString.startsWith("y")) {
      SetpointY = inputString.substring(1).toInt();
      SetpointY = constrain(SetpointY, 70, 110); // Ensure the target angle is within bounds
    }
    // Clear the input string to get ready for the next command
    inputString = "";
  }

  // Compute PID output for X and Y axes
  float pidOutputX = computePID(SetpointX, AccXangle, integralX, previousErrorX);
  float pidOutputY = computePID(SetpointY, AccYangle, integralY, previousErrorY);

  // Map the PID output to the servo range and write to servos
  int servoOutputX = map(pidOutputX, -180, 180, 45, 135);
  int servoOutputY = map(pidOutputY, -180, 180, 70, 110);
  int invServoOutputY = map(servoOutputY, 0, 180, 180, 0);
    
  servoX.write(constrain(servoOutputX, 45, 135));
  servoY.write(constrain(invServoOutputY, 70, 110));

  // Print PID constants
  Serial.print("Kp: "); Serial.print(Kp);
  Serial.print(", Ki: "); Serial.print(Ki);
  Serial.print(", Kd: "); Serial.print(Kd);
  // Print the updated target angles
  Serial.print("  Target Angle X: "); Serial.print(SetpointX);
  Serial.print(", Target Angle Y: "); Serial.print(SetpointY);
  // Print current angles
  Serial.print("  Current Angle X: "); Serial.print(AccXangle);
  Serial.print(", Current Angle Y: "); Serial.print(AccYangle);
  // Print motor speed
  Serial.print("  Motor Speed: "); Serial.println(MotorSpeedValue);

  // Delay to ensure loop timing as per DT
  delay(DT*1000 + 30);
}

void readIMU() {
  // Read accelerometer, gyroscope, and magnetometer
  readACC(buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));

  //readGYR(buff);
  //gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));
  //gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
  //gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));

  // Convert Gyro raw to degrees per second
  //rate_gyr_x = (float)gyrRaw[0] * G_GAIN;
  //rate_gyr_y = (float)gyrRaw[1] * G_GAIN;
  //rate_gyr_z = (float)gyrRaw[2] * G_GAIN;

  // Calculate the angles from the gyro
  //gyroXangle += rate_gyr_x * DT;
  //gyroYangle += rate_gyr_y * DT;
  //gyroZangle += rate_gyr_z * DT;

  // Convert Accelerometer values to degrees
  AccXangle = (float)(atan2(accRaw[1], accRaw[2]) + M_PI) * RAD_TO_DEG;
  AccYangle = (float)(atan2(accRaw[2], accRaw[0]) + M_PI) * RAD_TO_DEG;

  // Adjust angles based on orientation
  AccXangle -= 180.0;
  if (AccYangle > 90)
    AccYangle -= 270;
  else
    AccYangle += 90;

  // hack dont look closely
  if (AccXangle < 0){
    AccXangle += 360;
  }
  if (AccYangle < 0){
    AccYangle += 360;
  }
  AccXangle -= 90;
  AccYangle -= 90;

  AccXangle = constrain(AccXangle, 45, 135);
  AccYangle = constrain(AccYangle, 45, 135);

  // Complementary filter to combine accelerometer and gyro values
  //CFangleX = AA * (CFangleX + rate_gyr_x * DT) + (1 - AA) * AccXangle;
  //CFangleY = AA * (CFangleY + rate_gyr_y * DT) + (1 - AA) * AccYangle;
}