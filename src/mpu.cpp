#include <Arduino.h>
#include "mpumine.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define alpha 0.95

Mpumine::Mpumine():mpu(Adafruit_MPU6050()){}

int eror;float roll,pitch,x,y,z,gyro_x,gyro_y,gyro_z;

void Mpumine::begin() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float Mpumine::readacc_x() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  x = accel.acceleration.x;
  return accel.acceleration.x;
}
float Mpumine::readacc_y() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  y = accel.acceleration.y;
  return accel.acceleration.y;
}
float Mpumine::readacc_z() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  z = accel.acceleration.z;
  return accel.acceleration.z;
}
float Mpumine::readgyro_x() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  gyro_x = gyro.gyro.x;
  return gyro.gyro.x;
}
float Mpumine::readgyro_y() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  gyro_y = gyro.gyro.y;
  return gyro.gyro.y;
}
float Mpumine::readgyro_z() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  gyro_z = gyro.gyro.z;
  return gyro.gyro.z;
}

float Mpumine::readGforce() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  gForce = sqrt(accel.acceleration.x * accel.acceleration.x +
                accel.acceleration.y * accel.acceleration.y +
                accel.acceleration.z * accel.acceleration.z) / 9.80665;
  return gForce;
}

int Mpumine::error_cek() {
  Wire1.beginTransmission(0x68);  //buat baca mpu nyambung atau tidak pakai address mpu 0x68
  return Wire1.endTransmission();
}

float Mpumine::read_roll() {
  roll = atan2(x, y) * RAD_TO_DEG;
  float gyroRoll = roll + gyro_x * 0.01;
  roll = alpha * gyroRoll + (1 - alpha) * roll;
  return roll;
}

float Mpumine::read_pitch() {
  pitch = atan2(-x, sqrt(y * y + z * z)) * RAD_TO_DEG;
  float gyroPitch = pitch + gyro_y * 0.01;
  pitch = alpha * gyroPitch + (1 - alpha) * pitch;
  return pitch;
}
