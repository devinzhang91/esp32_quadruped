#ifndef _GYRO_H
#define _GYRO_H

#include <Arduino.h>
#include <Wire.h>

int initGyro(bool use_dmp=true);
int getYPRDMP(float *ypr);
int getYPRKalman(float *ypr);

#endif
