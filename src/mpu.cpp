#include <Arduino.h>
#include "mpumine.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Mpumine::Mpumine():mpu(Adafruit_MPU6050()){}

void Mpumine::begin() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float Mpumine::readacc_x() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  return accel.acceleration.x;
}
float Mpumine::readacc_y() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  return accel.acceleration.y;
}
float Mpumine::readacc_z() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  return accel.acceleration.z;
}

float Mpumine::readGforce() {
  sensors_event_t accel,gyro,tempe;
  mpu.getEvent(&accel, &gyro, &tempe);
  gForce = sqrt(accel.acceleration.x * accel.acceleration.x +
                accel.acceleration.y * accel.acceleration.y +
                accel.acceleration.z * accel.acceleration.z) / 9.80665;
  return gForce;
}