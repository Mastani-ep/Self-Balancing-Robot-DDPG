/* Source code of Mahony filter by jremington,
 https://github.com/jremington/MPU-6050-Fusion/blob/main/MPU6050_MahonyIMU.ino */

#include <Wire.h>
#include "Motor.h"
#define AIN1 13
#define AIN2 12
#define BIN1 8
#define BIN2 7
#define PWMA 6
#define PWMB 3

const int MPU = 0x68; // MPU6050 I2C address
float GyroX, GyroY, GyroZ;

int motor_torque = 0;
Motor *motorA, *motorB;

/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/

float kp = 22.0;
float ki = 0.0;
float kd = 0.8;

float desired_angle = 0;

float PID, error, previous_error, previous_previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
String str;
bool manual_control = true; // false - PID; true - model

// Anti-windup limits for the integral term
const float pid_i_min = -100.0;
const float pid_i_max = 100.0;

float G_off[3] = {-18.91, -65.53, 7.26}; // raw offsets, determined for gyro at rest

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 5.0;
float Ki = 0.0;

float yaw, pitch, roll; // Euler angle output

void setup() {
  Wire.begin();                      // Initialize communication
  Serial.begin(115200);

  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x18);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x18);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);

  motorA = new Motor(AIN1, AIN2, PWMA);
  motorB = new Motor(BIN2, BIN1, PWMB);
}

void loop() {

  static unsigned int i = 0; // loop counter
  static float deltat = 0;  // loop time in seconds
  static unsigned long now = 0, last = 0; // micros() timers

  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  Axyz[0] = (Wire.read() << 8 | Wire.read()) / 2048.0; // X-axis value
  Axyz[1] = (Wire.read() << 8 | Wire.read()) / 2048.0; // Y-axis value
  Axyz[2] = (Wire.read() << 8 | Wire.read()) / 2048.0; // Z-axis value

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

  GyroX = (Wire.read() << 8 | Wire.read()) / 16.4;
  GyroY = (Wire.read() << 8 | Wire.read()) / 16.4;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 16.4;

  Gxyz[0] = (((float) GyroX) - G_off[0]) * (PI/180.0);
  Gxyz[1] = (((float) GyroY) - G_off[1]) * (PI/180.0);
  Gxyz[2] = (((float) GyroZ) - G_off[2]) * (PI/180.0);

  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  // conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
  yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
  // to degrees
  yaw   *= 180.0 / PI;
  if (yaw < 0) yaw += 360.0; //compass circle
  // correct for local magnetic declination here
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

    roll += 7.2;

    if (roll > 0) {
      roll = roll * 90 / 113;
    }
    else {
      roll = roll * 90 / 82;
    }
//    Manual changes in calculated degrees due to problems stemming from those sensors lack of accuracy
//    and their position not being in the middle of the robot


  if (manual_control == false) {
    error = desired_angle - roll; // Error calculation

    // Proportional Error
    pid_p = kp * error;

    // Integral Error (with anti-windup)
    pid_i += ki * error * deltat;
    pid_i = constrain(pid_i, pid_i_min, pid_i_max);

    // Differential Error
    pid_d = kd * ((error - previous_error) / deltat);

    // Total PID value
    PID = pid_p + pid_i + pid_d;

    // Update the error value
    previous_previous_error = previous_error;
    previous_error = error;

    motor_torque = constrain(PID, -255, 255); // Ensure the speed is within motor limits
  }
  else {
    Serial.println(String(roll) + " " + String(motor_torque));
    str = Serial.readStringUntil('\n');
    motor_torque = str.toInt();
  }
  set_motors_torque();
}

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;

  // ignore accelerometer if false (tested OK, SJR)
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

void set_motors_torque() {
  motorA->set(motor_torque);
  motorB->set(-motor_torque);
}