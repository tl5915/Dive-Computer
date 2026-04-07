#pragma once

#include <stddef.h>
#include <stdint.h>

struct CompassCalibrationMatrices {
    float hard_iron[3];
    float soft_iron[9];
    float reference_field_gauss;
    float lsb_per_gauss;
    float fitted_field_lsb;
    bool is_valid;
};

enum class CompassCalibrationFitMethod : uint8_t {
    None = 0,
    Ellipsoid = 1,
    MinMax = 2,
    Error = 3,
};

void compassConfigureReferenceField(float reference_field_gauss, float lsb_per_gauss);

float compassReferenceFieldGauss();

float compassLsbPerGauss();

bool compassCalibrateFromSamples(const float* mag_samples_xyz,
                                 size_t sample_count,
                                 CompassCalibrationMatrices* out_matrices);

bool compassCalibrateFromSamplesWithFallback(const float* mag_samples_xyz,
                                             size_t sample_count,
                                             CompassCalibrationMatrices* out_matrices,
                                             CompassCalibrationFitMethod* out_method);

bool compassSetCalibrationMatrices(const CompassCalibrationMatrices* matrices);

bool compassGetCalibrationMatrices(CompassCalibrationMatrices* out_matrices);

void compassApplyCalibration(const float raw_xyz[3], float calibrated_xyz[3]);

void compassApplyCalibrationAndTiltCompensation(const float raw_xyz[3],
                                                const float accel_xyz[3],
                                                float compensated_xyz[3]);

float compassHeadingFromMagneticVector(const float magnetic_xyz[3]);
