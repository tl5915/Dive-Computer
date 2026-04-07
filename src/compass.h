#pragma once
#include <Arduino.h>

enum CompassCalibrationStage : uint8_t {
	COMPASS_CAL_STAGE_COLLECTING = 0,
	COMPASS_CAL_STAGE_COMPUTING = 1,
	COMPASS_CAL_STAGE_DONE = 2,
	COMPASS_CAL_STAGE_FAILED = 3
};

const char* compassLastCalibrationMethod();

using CompassCalibrationStageCallback = void (*)(CompassCalibrationStage stage);

void compassSetCalibrationStageCallback(CompassCalibrationStageCallback callback);

bool compassInit(int sda, int scl);

float readAccelMagnitude();

float readHeading();

void compassCalibrate();