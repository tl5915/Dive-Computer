# Magnetometer Hard and Soft Iron Calibration Library

A lightweight library for magnetometer calibration and compass heading computation suitable for microcontrollers. Supports ellipsoid fitting and Min/Max diagonal calibration with optional tilt compensation.

This library calibrates for hard iron and soft iron distortions using ellipsoid fitting, based on the work of [sailboatinstruments1](https://sites.google.com/view/sailboatinstruments1).

## Features

- **Dual calibration methods**
  
  - **Ellipsoid fitting**: accurate multi-axis calibration using least-squares optimisation
  - **Min/Max diagonal**: simple fallback method for resource-constrained scenarios
- **Tilt compensation**: optional accelerometer-based tilt compensation

## Hardware Requirements

- Magnetometer (QMC5883L, AK8963, HMC5883L, etc.)
- Optional: accelerometer or 6-axis IMU (for tilt compensation)
- MCU: Arduino, ESP32, etc

## Testing

This library gives very similar result to the [Magneto 1.2 app ]([https://](https://sites.google.com/view/sailboatinstruments1/a-download-magneto-v1-2)). See `ellipsoid.test` folder

## Functions

### Configuration

```cpp
void compassConfigureReferenceField(float reference_field_gauss, float lsb_per_gauss);
```

Set the reference magnetic field strength and magnetometer sensitivity.

**Parameters:**

- `reference_field_gauss`: expected magnetic field magnitude. Find it [here](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm).
- `lsb_per_gauss`: sensor sensitivity (e.g. for QMC5883L at +/- 8 Gauss range: 3000 LSB/Gauss). Only affects global gain, not direction, for compass use can leave at default.

### Calibration

```cpp
bool compassCalibrateFromSamplesWithFallback(
    const float* mag_samples_xyz,      // Flattened [x, y, z, x, y, z, ...]
    size_t sample_count,
    CompassCalibrationMatrices* out_matrices,
    CompassCalibrationFitMethod* out_method
);
```

Run calibration on collected samples. Automatically tries ellipsoid fitting first; falls back to Min/Max if ellipsoid fails.
Rotate in all directions during data collection.

**Parameters:**

- `mag_samples_xyz`: Array of raw magnetometer readings (3 floats per sample, minimum 50 samples required)
- `sample_count`: Number of (x, y, z) triplets
- `out_matrices`: Output struct containing hard_iron, soft_iron, reference field, and validity flag
- `out_method`: Enum indicating which method succeeded (Ellipsoid or Min/Max)

**Returns:** `true` if calibration succeeded, `false` otherwise

### Set/Get Calibration

```cpp
bool compassSetCalibrationMatrices(const CompassCalibrationMatrices* matrices);
bool compassGetCalibrationMatrices(CompassCalibrationMatrices* out_matrices);
```

Store/retrieve calibration matrices globally for use in heading calculations.

### Apply Calibration

```cpp
void compassApplyCalibration(const float raw_xyz[3], float calibrated_xyz[3]);
```

Apply hard iron + soft iron correction to raw magnetometer readings.

```cpp
void compassApplyCalibrationAndTiltCompensation(
    const float raw_xyz[3],    // Raw magnetometer
    const float accel_xyz[3],  // Accelerometer
    float compensated_xyz[3]
);
```

Apply calibration plus tilt compensation (pitch + roll) using accelerometer data.

### Heading Computation

```cpp
float compassHeadingFromMagneticVector(const float magnetic_xyz[3]);
```

Convert calibrated/tilt compensated magnetic vector to compass heading (0–360 degrees).

**Returns:** Heading in degrees

## Data Types

### CompassCalibrationMatrices

```cpp
struct CompassCalibrationMatrices {
    float hard_iron[3];     // Bias offsets: [Bx, By, Bz]
    float soft_iron[9];     // 3×3 scale/rotation matrix (row-major)
    float reference_field_gauss;
    float lsb_per_gauss;
    float fitted_field_lsb;
    bool is_valid;
};
```

### CompassCalibrationFitMethod

```cpp
enum class CompassCalibrationFitMethod : uint8_t {
    None = 0,       // Calibration not run
    Ellipsoid = 1,  // Ellipsoid fitting succeeded
    MinMax = 2,     // Min/Max fallback used
    Error = 3       // Calibration failed
};
```

## Usage Example

**Quick workflow:**

1. **Initialise with calibration parameters**
   
   ```cpp
   compassConfigureReferenceField(0.49f, 3000.0f);
   ```
2. **Collect samples**
   
   ```cpp
   std::vector<float> samples;
   // Read magnetometer while rotating device
   for (int i = 0; i < 10000; i++) {
       samples.push_back(readMagX());
       samples.push_back(readMagY());
       samples.push_back(readMagZ());
   }
   ```
3. **Calibrate**
   
   ```cpp
   CompassCalibrationMatrices matrices = {};
   CompassCalibrationFitMethod method = CompassCalibrationFitMethod::Error;
   bool success = compassCalibrateFromSamplesWithFallback(
       samples.data(), samples.size() / 3, &matrices, &method
   );
   ```
   
   Output format (`CompassCalibrationMatrices`):
   
   - `hard_iron[3]`: bias offsets `[bx, by, bz]`
   - `soft_iron[9]`: row-major 3x3 correction matrix
     `[m00, m01, m02, m10, m11, m12, m20, m21, m22]`
   - `reference_field_gauss`: configured local magnetic field in Gauss
   - `lsb_per_gauss`: configured sensor sensitivity in LSB/G
   - `fitted_field_lsb`: fitted field magnitude in sensor counts
   - `is_valid`: calibration validity flag
   
   Save calibration result to NVS after success.
   
   ```cpp
   #include <Preferences.h>
   
   if (success && matrices.is_valid) {
       Preferences prefs;
       prefs.begin("compass", false);
       prefs.putBytes("hard_iron", matrices.hard_iron, sizeof(matrices.hard_iron));
       prefs.putBytes("soft_iron", matrices.soft_iron, sizeof(matrices.soft_iron));
       prefs.putFloat("ref_gauss", matrices.reference_field_gauss);
       prefs.putFloat("lsb_per_g", matrices.lsb_per_gauss);
       prefs.putFloat("fit_lsb", matrices.fitted_field_lsb);
       prefs.putUChar("fit_method", static_cast<uint8_t>(method));
       prefs.end();
   }
   ```
4. **Apply calibration**
   
   ```cpp
   if (success) {
       compassSetCalibrationMatrices(&matrices);
       float raw[3] = {readMagX(), readMagY(), readMagZ()};
       float calibrated[3];
       compassApplyCalibration(raw, calibrated);
       float heading = compassHeadingFromMagneticVector(calibrated);
   }
   ```

