#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <SensorQMI8658.hpp>
#include <QMC5883LCompass.h>
#include "compass.h"
#include "ellipsoid.h"

static SensorQMI8658 qmi;
static QMC5883LCompass mag;
static Preferences prefs;
static CalibrationCoefficients cal_coeffs;
static CompassCalibrationStageCallback calibration_stage_callback = nullptr;
static const char* last_calibration_method = "None";

constexpr const char *Compass_NVS_Namespace = "compass";
constexpr const char *Calibration_Valid_Key = "cal_valid";

// QMC5883L registers
constexpr uint8_t QMC5883L_I2C_Address = 0x0D;
constexpr uint8_t QMC5883L_Status_Register = 0x06;
constexpr uint8_t QMC5883L_Status_DRDY_Mask = 0x01;

// Ellipsoid calibration parameters
constexpr const char *Hard_Iron_X_Key = "hi_x";
constexpr const char *Hard_Iron_Y_Key = "hi_y";
constexpr const char *Hard_Iron_Z_Key = "hi_z";
constexpr const char *Soft_Iron_Key = "soft_iron";
constexpr const char *Reference_Mag_Key = "ref_mag";

// Min/Max calibration parameters (fall back)
constexpr const char *Offset_X_Key = "off_x";
constexpr const char *Offset_Y_Key = "off_y";
constexpr const char *Offset_Z_Key = "off_z";
constexpr const char *Scale_X_Key = "scale_x";
constexpr const char *Scale_Y_Key = "scale_y";
constexpr const char *Scale_Z_Key = "scale_z";

constexpr uint32_t Calibration_Duration_MS = 120000;  // 120 second data collection

// UK magnetic field: 49000 nT = 0.49 Gauss
// 8 Gauss range: 2048 LSB/Gauss
// Expected reading: 0.49 * 2048 = 1003.5 LSB
constexpr float QMC5883L_Reference_LSB = 1003.5f;


// Load calibration parameters
void loadCalibrationFromNVS() {
  if (!prefs.begin(Compass_NVS_Namespace, true)) {
    return;
  }
  const bool valid = prefs.getBool(Calibration_Valid_Key, false);
  if (!valid) {
    prefs.end();
    return;
  }
  // Prioritise Ellipsoid
  if (prefs.isKey(Hard_Iron_X_Key)) {
    cal_coeffs.hard_iron[0] = prefs.getFloat(Hard_Iron_X_Key, 0.0f);
    cal_coeffs.hard_iron[1] = prefs.getFloat(Hard_Iron_Y_Key, 0.0f);
    cal_coeffs.hard_iron[2] = prefs.getFloat(Hard_Iron_Z_Key, 0.0f);
    cal_coeffs.reference_magnitude = prefs.getFloat(Reference_Mag_Key, QMC5883L_Reference_LSB);
    size_t si_size = prefs.getBytesLength(Soft_Iron_Key);
    if (si_size == 9 * sizeof(float)) {
      prefs.getBytes(Soft_Iron_Key, (uint8_t*)cal_coeffs.soft_iron, si_size);
      cal_coeffs.is_valid = true;
    } else {
      cal_coeffs.is_valid = false;
    }
  } else {
    // Fall back to Min/Max
    mag.setCalibrationOffsets(
        prefs.getFloat(Offset_X_Key, 0.0f),
        prefs.getFloat(Offset_Y_Key, 0.0f),
        prefs.getFloat(Offset_Z_Key, 0.0f));
    mag.setCalibrationScales(
        prefs.getFloat(Scale_X_Key, 1.0f),
        prefs.getFloat(Scale_Y_Key, 1.0f),
        prefs.getFloat(Scale_Z_Key, 1.0f));
    cal_coeffs.is_valid = false;
  }
  prefs.end();
}


// Save calibration parameters
void saveCalibrationToNVS() {
  if (!prefs.begin(Compass_NVS_Namespace, false)) {
    return;
  }
  if (cal_coeffs.is_valid) {
    // Save Ellipsoid parameters
    prefs.putFloat(Hard_Iron_X_Key, cal_coeffs.hard_iron[0]);
    prefs.putFloat(Hard_Iron_Y_Key, cal_coeffs.hard_iron[1]);
    prefs.putFloat(Hard_Iron_Z_Key, cal_coeffs.hard_iron[2]);
    prefs.putFloat(Reference_Mag_Key, cal_coeffs.reference_magnitude);
    prefs.putBytes(Soft_Iron_Key, (uint8_t*)cal_coeffs.soft_iron, 9 * sizeof(float));
  } else {
    // Save Min/Max parameters
    prefs.putFloat(Offset_X_Key, mag.getCalibrationOffset(0));
    prefs.putFloat(Offset_Y_Key, mag.getCalibrationOffset(1));
    prefs.putFloat(Offset_Z_Key, mag.getCalibrationOffset(2));
    prefs.putFloat(Scale_X_Key, mag.getCalibrationScale(0));
    prefs.putFloat(Scale_Y_Key, mag.getCalibrationScale(1));
    prefs.putFloat(Scale_Z_Key, mag.getCalibrationScale(2));
  }
  prefs.putBool(Calibration_Valid_Key, true);
  prefs.end();
}


// Calibration method used: Ellipsoid, Min/Max, None, Error)
const char* compassLastCalibrationMethod() {
  return last_calibration_method;
}


// Calibration stage callback
void compassSetCalibrationStageCallback(CompassCalibrationStageCallback callback) {
  calibration_stage_callback = callback;
}


// Ellipsoid: Calibrated = A * (Raw - B)
static inline void apply_calibration(float& mx, float& my, float& mz) {
  if (!cal_coeffs.is_valid) {
    return;
  }
  // Hard iron correction: subtract bias
  float corrected[3];
  corrected[0] = mx - cal_coeffs.hard_iron[0];
  corrected[1] = my - cal_coeffs.hard_iron[1];
  corrected[2] = mz - cal_coeffs.hard_iron[2];
  // Soft iron correction: matrix transformation
  mx = cal_coeffs.soft_iron[0] * corrected[0] + 
       cal_coeffs.soft_iron[1] * corrected[1] + 
       cal_coeffs.soft_iron[2] * corrected[2];
  my = cal_coeffs.soft_iron[3] * corrected[0] + 
       cal_coeffs.soft_iron[4] * corrected[1] + 
       cal_coeffs.soft_iron[5] * corrected[2];
  mz = cal_coeffs.soft_iron[6] * corrected[0] + 
       cal_coeffs.soft_iron[7] * corrected[1] + 
       cal_coeffs.soft_iron[8] * corrected[2];
}


// Sensor initialisation
bool compassInit(int sda, int scl) {
  // Initialize QMI8658
  qmi.begin(Wire, 0x6B, sda, scl);
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_2);
  qmi.enableAccelerometer();
  qmi.disableGyroscope();
  // Initialize QMC5883L
  mag.init();
  // Mode: continuous; ODR: 50Hz; Range: 8G; OSR: 512
  mag.setMode(0x01, 0x04, 0x10, 0x00);
  // Load calibration parameters
  loadCalibrationFromNVS();
  return true;
}


// QMI8658 reading for tapping detection
float readAccelMagnitude() {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);
  return sqrtf(ax * ax + ay * ay + az * az);
}


// QMC5883L data ready check
static bool qmc5883lDataReady() {
  Wire.beginTransmission(QMC5883L_I2C_Address);
  Wire.write(QMC5883L_Status_Register);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t count = Wire.requestFrom(
      static_cast<uint8_t>(QMC5883L_I2C_Address),
      static_cast<uint8_t>(1));
  if (count != 1 || Wire.available() <= 0) {
    return false;
  }
  const uint8_t status = static_cast<uint8_t>(Wire.read());
  return (status & QMC5883L_Status_DRDY_Mask) != 0;
}


// Read compensated and calibrated heading
float readHeading() {
  // Accelerometer readings
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);
  // Raw magnetometer readings
  mag.read();
  float mx = static_cast<float>(mag.getX());
  float my = static_cast<float>(mag.getY());
  float mz = static_cast<float>(mag.getZ());
  // Apply calibration
  apply_calibration(mx, my, mz);
  // Compute tilt angles
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float roll  = atan2f(ay, az);
  // Rotate magnetometer vector to horizontal plane
  const float mx2 =  mx * cosf(pitch)
                   + mz * sinf(pitch);
  const float my2 =  mx * sinf(roll) * sinf(pitch)
                   + my * cosf(roll)
                   - mz * sinf(roll) * cosf(pitch);
  // Heading in degrees [0, 360)
  float heading = atan2f(-my2, mx2) * (180.0f / static_cast<float>(M_PI));
  if (heading < 0.0f) {
    heading += 360.0f;
  }
  return heading;
}


// Compass calibration
void compassCalibrate() {
  if (calibration_stage_callback != nullptr) {
    calibration_stage_callback(COMPASS_CAL_STAGE_COLLECTING);
  }

  mag.clearCalibration();

  // Allocate sample buffer
  constexpr int MAX_SAMPLES = 10000;
  MagSample* samples = (MagSample*)malloc(MAX_SAMPLES * sizeof(MagSample));
  // No samples allocated, failed calibration
  if (samples == nullptr) {
    last_calibration_method = "Error";
    if (calibration_stage_callback != nullptr) {
      calibration_stage_callback(COMPASS_CAL_STAGE_FAILED);
    }
    return;
  }
  
  // Collect data
  int sample_count = 0;
  uint32_t start_time = millis();
  while ((millis() - start_time) < Calibration_Duration_MS && sample_count < MAX_SAMPLES) {
    if (qmc5883lDataReady()) {
      mag.read();
      samples[sample_count].x = static_cast<float>(mag.getX());
      samples[sample_count].y = static_cast<float>(mag.getY());
      samples[sample_count].z = static_cast<float>(mag.getZ());
      sample_count++;
    }
    delay(1);
  }

  // Compute calibration parameters
  if (calibration_stage_callback != nullptr) {
    calibration_stage_callback(COMPASS_CAL_STAGE_COMPUTING);
  }
  int result = calibrate_ellipsoid(samples, sample_count, QMC5883L_Reference_LSB, &cal_coeffs);
  if (result == 0 && cal_coeffs.is_valid) {
    // Ellipsoid
    last_calibration_method = "Ellipsoid";
    saveCalibrationToNVS();
    if (calibration_stage_callback != nullptr) {
      calibration_stage_callback(COMPASS_CAL_STAGE_DONE);
    }
  } else {
    // Fallback to Min-Max
    if (sample_count == 0) {
      last_calibration_method = "Error";
      free(samples);
      if (calibration_stage_callback != nullptr) {
        calibration_stage_callback(COMPASS_CAL_STAGE_FAILED);
      }
      return;
    }
    mag.clearCalibration();
    long minX = samples[0].x, maxX = samples[0].x;
    long minY = samples[0].y, maxY = samples[0].y;
    long minZ = samples[0].z, maxZ = samples[0].z;
    for (int i = 0; i < sample_count; i++) {
      if (samples[i].x < minX) minX = samples[i].x;
      if (samples[i].x > maxX) maxX = samples[i].x;
      if (samples[i].y < minY) minY = samples[i].y;
      if (samples[i].y > maxY) maxY = samples[i].y;
      if (samples[i].z < minZ) minZ = samples[i].z;
      if (samples[i].z > maxZ) maxZ = samples[i].z;
    }
    const float xOffset = (minX + maxX) * 0.5f;
    const float yOffset = (minY + maxY) * 0.5f;
    const float zOffset = (minZ + maxZ) * 0.5f;
    const float xHalfSpan = (maxX - minX) * 0.5f;
    const float yHalfSpan = (maxY - minY) * 0.5f;
    const float zHalfSpan = (maxZ - minZ) * 0.5f;
    const float averageHalfSpan = (xHalfSpan + yHalfSpan + zHalfSpan) / 3.0f;
    mag.setCalibrationOffsets(xOffset, yOffset, zOffset);
    mag.setCalibrationScales(
        (xHalfSpan > 0.0f) ? (averageHalfSpan / xHalfSpan) : 1.0f,
        (yHalfSpan > 0.0f) ? (averageHalfSpan / yHalfSpan) : 1.0f,
        (zHalfSpan > 0.0f) ? (averageHalfSpan / zHalfSpan) : 1.0f);
    cal_coeffs.is_valid = false;
    last_calibration_method = "Min/Max";
    saveCalibrationToNVS();
    if (calibration_stage_callback != nullptr) {
      calibration_stage_callback(COMPASS_CAL_STAGE_DONE);
    }
  }
  free(samples);
}