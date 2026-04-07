#include <math.h>
#include <string.h>
#include "magnetometer_calibration.h"
#include "minmax.h"
#include "ellipsoid.h"

namespace {
constexpr float kDefaultReferenceFieldGauss = 0.49f;
constexpr float kDefaultLsbPerGauss = 3000.0f;

CalibrationCoefficients g_calibration = {};
float g_reference_field_gauss = kDefaultReferenceFieldGauss;
float g_lsb_per_gauss = kDefaultLsbPerGauss;

void applySoftAndHardIron(const float raw_xyz[3], float calibrated_xyz[3]) {
    if (!g_calibration.is_valid) {
        calibrated_xyz[0] = raw_xyz[0];
        calibrated_xyz[1] = raw_xyz[1];
        calibrated_xyz[2] = raw_xyz[2];
        return;
    }

    const float corrected_x = raw_xyz[0] - g_calibration.hard_iron[0];
    const float corrected_y = raw_xyz[1] - g_calibration.hard_iron[1];
    const float corrected_z = raw_xyz[2] - g_calibration.hard_iron[2];

    calibrated_xyz[0] = g_calibration.soft_iron[0] * corrected_x +
                        g_calibration.soft_iron[1] * corrected_y +
                        g_calibration.soft_iron[2] * corrected_z;
    calibrated_xyz[1] = g_calibration.soft_iron[3] * corrected_x +
                        g_calibration.soft_iron[4] * corrected_y +
                        g_calibration.soft_iron[5] * corrected_z;
    calibrated_xyz[2] = g_calibration.soft_iron[6] * corrected_x +
                        g_calibration.soft_iron[7] * corrected_y +
                        g_calibration.soft_iron[8] * corrected_z;
}
}


// ----- Public ----- //

// Calibration parameters configuration
void compassConfigureReferenceField(float reference_field_gauss, float lsb_per_gauss) {
    if (reference_field_gauss > 0.0f) {
        g_reference_field_gauss = reference_field_gauss;
    }
    if (lsb_per_gauss > 0.0f) {
        g_lsb_per_gauss = lsb_per_gauss;
    }
}

// Get reference magnetic field intensity in Gauss
float compassReferenceFieldGauss() {
    return g_reference_field_gauss;
}

// Get magnetometer scaling factor in LSB per Gauss
float compassLsbPerGauss() {
    return g_lsb_per_gauss;
}

// Compass calibration with ellipsoid method
bool compassCalibrateFromSamples(const float* mag_samples_xyz,
                                 size_t sample_count,
                                 CompassCalibrationMatrices* out_matrices) {
    if (mag_samples_xyz == nullptr || out_matrices == nullptr || sample_count < 50U) {
        return false;  // Minimum 50 samples for reliable ellipsoid fit
    }

    MagSample* samples = new MagSample[sample_count];
    if (samples == nullptr) {
        return false;
    }

    for (size_t i = 0; i < sample_count; ++i) {
        samples[i].x = mag_samples_xyz[(i * 3U) + 0U];
        samples[i].y = mag_samples_xyz[(i * 3U) + 1U];
        samples[i].z = mag_samples_xyz[(i * 3U) + 2U];
    }

    CalibrationCoefficients fitted = {};
    const float reference_field_lsb = g_reference_field_gauss * g_lsb_per_gauss;
    const int fit_result = calibrate_ellipsoid(samples,
                                               static_cast<int>(sample_count),
                                               reference_field_lsb,
                                               &fitted);
    delete[] samples;

    if (fit_result != 0 || !fitted.is_valid) {
        return false;
    }

    memcpy(out_matrices->hard_iron, fitted.hard_iron, sizeof(fitted.hard_iron));
    memcpy(out_matrices->soft_iron, fitted.soft_iron, sizeof(fitted.soft_iron));
    out_matrices->reference_field_gauss = g_reference_field_gauss;
    out_matrices->lsb_per_gauss = g_lsb_per_gauss;
    out_matrices->fitted_field_lsb = fitted.field_magnitude;
    out_matrices->is_valid = true;

    return true;
}

// Compass calibration with Min/Max method (fallback)
bool compassCalibrateFromSamplesWithFallback(const float* mag_samples_xyz,
                                             size_t sample_count,
                                             CompassCalibrationMatrices* out_matrices,
                                             CompassCalibrationFitMethod* out_method) {
    if (out_method != nullptr) {
        *out_method = CompassCalibrationFitMethod::Error;
    }

    if (mag_samples_xyz == nullptr || out_matrices == nullptr || sample_count < 50U) {
        return false;
    }

    if (compassCalibrateFromSamples(mag_samples_xyz, sample_count, out_matrices)) {
        if (out_method != nullptr) {
            *out_method = CompassCalibrationFitMethod::Ellipsoid;
        }
        return true;
    }

    if (calibrate_minmax(mag_samples_xyz,
                         sample_count,
                         g_reference_field_gauss,
                         g_lsb_per_gauss,
                         out_matrices)) {
        if (out_method != nullptr) {
            *out_method = CompassCalibrationFitMethod::MinMax;
        }
        return true;
    }

    return false;
}

// Set calibration matrices to be applied on raw magnetometer readings
bool compassSetCalibrationMatrices(const CompassCalibrationMatrices* matrices) {
    if (matrices == nullptr || !matrices->is_valid) {
        g_calibration = {};
        return false;
    }

    g_calibration = {};
    memcpy(g_calibration.hard_iron, matrices->hard_iron, sizeof(g_calibration.hard_iron));
    memcpy(g_calibration.soft_iron, matrices->soft_iron, sizeof(g_calibration.soft_iron));
    g_calibration.reference_magnitude = matrices->reference_field_gauss * matrices->lsb_per_gauss;
    g_calibration.field_magnitude = matrices->fitted_field_lsb;
    g_calibration.is_valid = true;

    return true;
}

// Get current calibration matrices
bool compassGetCalibrationMatrices(CompassCalibrationMatrices* out_matrices) {
    if (out_matrices == nullptr || !g_calibration.is_valid) {
        return false;
    }

    memcpy(out_matrices->hard_iron, g_calibration.hard_iron, sizeof(g_calibration.hard_iron));
    memcpy(out_matrices->soft_iron, g_calibration.soft_iron, sizeof(g_calibration.soft_iron));
    out_matrices->reference_field_gauss = g_reference_field_gauss;
    out_matrices->lsb_per_gauss = g_lsb_per_gauss;
    out_matrices->fitted_field_lsb = g_calibration.field_magnitude;
    out_matrices->is_valid = true;

    return true;
}

// Apply calibration to raw magnetometer readings
void compassApplyCalibration(const float raw_xyz[3], float calibrated_xyz[3]) {
    applySoftAndHardIron(raw_xyz, calibrated_xyz);
}

// Apply calibration and tilt compensation to raw magnetometer readings
void compassApplyCalibrationAndTiltCompensation(const float raw_xyz[3],
                                                const float accel_xyz[3],
                                                float compensated_xyz[3]) {
    float calibrated_xyz[3] = {0.0f, 0.0f, 0.0f};
    applySoftAndHardIron(raw_xyz, calibrated_xyz);

    const float ax = accel_xyz[0];
    const float ay = accel_xyz[1];
    const float az = accel_xyz[2];
    const float accel_norm_sq = (ax * ax) + (ay * ay) + (az * az);
    if (accel_norm_sq < 1e-8f) {
        compensated_xyz[0] = calibrated_xyz[0];
        compensated_xyz[1] = calibrated_xyz[1];
        compensated_xyz[2] = calibrated_xyz[2];
        return;
    }

    const float pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
    const float roll = atan2f(ay, az);

    const float mx = calibrated_xyz[0];
    const float my = calibrated_xyz[1];
    const float mz = calibrated_xyz[2];

    compensated_xyz[0] = (mx * cosf(pitch)) + (mz * sinf(pitch));
    compensated_xyz[1] = (mx * sinf(roll) * sinf(pitch)) +
                         (my * cosf(roll)) -
                         (mz * sinf(roll) * cosf(pitch));
    compensated_xyz[2] = (-mx * cosf(roll) * sinf(pitch)) +
                         (my * sinf(roll)) +
                         (mz * cosf(roll) * cosf(pitch));
}

// Calculate compass heading in degrees
float compassHeadingFromMagneticVector(const float magnetic_xyz[3]) {
    float heading = atan2f(-magnetic_xyz[1], magnetic_xyz[0]) * (180.0f / static_cast<float>(M_PI));
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}
