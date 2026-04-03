#include "compass.h"
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <SensorQMI8658.hpp>
#include <QMC5883LCompass.h>

static SensorQMI8658 qmi;
static QMC5883LCompass mag;
static Preferences prefs;

constexpr const char *Compass_NVS_Namespace = "compass";
constexpr const char *Calibration_Valid_Key = "cal_valid";
constexpr const char *Offset_X_Key = "off_x";
constexpr const char *Offset_Y_Key = "off_y";
constexpr const char *Offset_Z_Key = "off_z";
constexpr const char *Scale_X_Key = "scale_x";
constexpr const char *Scale_Y_Key = "scale_y";
constexpr const char *Scale_Z_Key = "scale_z";


void loadCalibrationFromNVS() {
  if (!prefs.begin(Compass_NVS_Namespace, true)) {
    return;
  }
  const bool valid = prefs.getBool(Calibration_Valid_Key, false);
  if (valid) {
    mag.setCalibrationOffsets(
        prefs.getFloat(Offset_X_Key, 0.0f),
        prefs.getFloat(Offset_Y_Key, 0.0f),
        prefs.getFloat(Offset_Z_Key, 0.0f));
    mag.setCalibrationScales(
        prefs.getFloat(Scale_X_Key, 1.0f),
        prefs.getFloat(Scale_Y_Key, 1.0f),
        prefs.getFloat(Scale_Z_Key, 1.0f));
  }
  prefs.end();
}


void saveCalibrationToNVS() {
  if (!prefs.begin(Compass_NVS_Namespace, false)) {
    return;
  }
  prefs.putFloat(Offset_X_Key, mag.getCalibrationOffset(0));
  prefs.putFloat(Offset_Y_Key, mag.getCalibrationOffset(1));
  prefs.putFloat(Offset_Z_Key, mag.getCalibrationOffset(2));
  prefs.putFloat(Scale_X_Key, mag.getCalibrationScale(0));
  prefs.putFloat(Scale_Y_Key, mag.getCalibrationScale(1));
  prefs.putFloat(Scale_Z_Key, mag.getCalibrationScale(2));
  prefs.putBool(Calibration_Valid_Key, true);
  prefs.end();
}


bool compassInit(int sda, int scl) {
  // QMI8658
  qmi.begin(Wire, 0x6B, sda, scl);  // Default address 0x6B
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_2);
  qmi.enableAccelerometer();
  qmi.disableGyroscope();

  // QMC5883L
  mag.init();  // Default address 0x0D
  mag.setMode(0x01, 0x04, 0x10, 0x00);  // mode=continuous, ODR=50Hz, range=8G, OSR=512
  loadCalibrationFromNVS();

  return true;
}


float readHeading() {
  // Accelerometer
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);

  // Magnetometer
  mag.read();
  const float mx = static_cast<float>(mag.getX());
  const float my = static_cast<float>(mag.getY());
  const float mz = static_cast<float>(mag.getZ());

  // Tilt compensation
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float roll  = atan2f(ay, az);

  // Rotate magnetometer vector into the horizontal plane.
  const float mx2 =  mx * cosf(pitch)
                   + mz * sinf(pitch);
  const float my2 =  mx * sinf(roll) * sinf(pitch)
                   + my * cosf(roll)
                   - mz * sinf(roll) * cosf(pitch);

  // Heading in degrees, normalised to [0, 360).
  float heading = atan2f(-my2, mx2) * (180.0f / static_cast<float>(M_PI));
  if (heading < 0.0f) {
    heading += 360.0f;
  }
  return heading;
}


void compassCalibrate() {
  mag.calibrate();
  saveCalibrationToNVS();
}