#pragma once
#include <Arduino.h>

bool compassInit(int sda, int scl);
float readHeading();
void compassCalibrate();