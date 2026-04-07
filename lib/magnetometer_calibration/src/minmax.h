#pragma once

#include <stddef.h>
#include "magnetometer_calibration.h"

// Min/Max diagonal calibration fallback
static inline bool calibrate_minmax(const float* mag_samples_xyz,
                                    size_t sample_count,
                                    float reference_field_gauss,
                                    float lsb_per_gauss,
                                    CompassCalibrationMatrices* out_matrices) {
    if (mag_samples_xyz == nullptr || out_matrices == nullptr || sample_count == 0U) {
        return false;
    }

    float min_x = mag_samples_xyz[0];
    float max_x = mag_samples_xyz[0];
    float min_y = mag_samples_xyz[1];
    float max_y = mag_samples_xyz[1];
    float min_z = mag_samples_xyz[2];
    float max_z = mag_samples_xyz[2];

    for (size_t i = 1U; i < sample_count; ++i) {
        const float x = mag_samples_xyz[(i * 3U) + 0U];
        const float y = mag_samples_xyz[(i * 3U) + 1U];
        const float z = mag_samples_xyz[(i * 3U) + 2U];
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
    }

    const float x_offset = (min_x + max_x) * 0.5f;
    const float y_offset = (min_y + max_y) * 0.5f;
    const float z_offset = (min_z + max_z) * 0.5f;
    const float x_half_span = (max_x - min_x) * 0.5f;
    const float y_half_span = (max_y - min_y) * 0.5f;
    const float z_half_span = (max_z - min_z) * 0.5f;

    const float average_half_span = (x_half_span + y_half_span + z_half_span) / 3.0f;
    if (average_half_span <= 0.0f) {
        return false;
    }

    *out_matrices = {};
    out_matrices->hard_iron[0] = x_offset;
    out_matrices->hard_iron[1] = y_offset;
    out_matrices->hard_iron[2] = z_offset;

    out_matrices->soft_iron[0] = (x_half_span > 0.0f) ? (average_half_span / x_half_span) : 1.0f;
    out_matrices->soft_iron[4] = (y_half_span > 0.0f) ? (average_half_span / y_half_span) : 1.0f;
    out_matrices->soft_iron[8] = (z_half_span > 0.0f) ? (average_half_span / z_half_span) : 1.0f;

    out_matrices->reference_field_gauss = reference_field_gauss;
    out_matrices->lsb_per_gauss = lsb_per_gauss;
    out_matrices->fitted_field_lsb = average_half_span;
    out_matrices->is_valid = true;
    return true;
}