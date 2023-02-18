#ifndef mpumine_h
#define mpumine_h

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class Mpumine {
  private:
    Adafruit_MPU6050 mpu;
    float gForce;
  public:
    Mpumine();
    void begin();
    float readacc_x(); 
    float readacc_y();
    float readacc_z();
    float readGforce();

};

#endif

