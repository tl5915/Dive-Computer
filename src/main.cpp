#include <Arduino.h>
// Power Management
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_sleep.h>
// Peripherals
#include <SPI.h>
#include <Wire.h>
#include <BH1750.h>
#include <MS5837.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <QMC5883LCompass.h>
#include <SensorQMI8658.hpp>
// Data Processing
#include <vector>
#include <ZHL16C.h>
#include <Preferences.h>
#include <magnetometer_calibration.h>


// Pins
constexpr uint8_t Battery_Pin = 1;
constexpr uint8_t DC_Pin = 4;
constexpr uint8_t CS_Pin = 5;
constexpr uint8_t SCK_Pin = 6;
constexpr uint8_t MOSI_Pin = 7;
constexpr uint8_t RST_Pin = 8;
constexpr uint8_t Backlight_Pin = 15;
constexpr uint8_t SCL_Pin = 10;
constexpr uint8_t SDA_Pin = 11;
constexpr uint8_t Calibration_Button_Pin = 40;

// Define Objects
SPIClass spi(FSPI);
Preferences prefs;
MS5837 sensor;
BH1750 lightMeter;
SensorQMI8658 qmi;
QMC5883LCompass qmc;
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

// Battery Percentage Lookup Table
constexpr float Voltage_Table[] = {
    3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
    3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
constexpr uint8_t Percentage_Table[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
    55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
constexpr size_t Battery_Table_Size = sizeof(Voltage_Table) / sizeof(Voltage_Table[0]);

// LCD Constants
constexpr uint16_t LCD_Width = 240;
constexpr uint16_t LCD_Height = 280;
constexpr float Low_Battery_Threshold = 3.7f;          // Dim screen below 3.7 V
constexpr float Critical_Battery_Threshold = 3.2f;     // Deep sleep below 3.2 V
constexpr float Battery_Divider_Ratio = 3.0f;          // 2:1 voltage divider
constexpr float Ambient_Lux_Max = 800.0f;              // Lux level at high backlight
constexpr uint8_t Backlight_Low = 8;                   // Low backlight in dark surroundings
constexpr uint8_t Backlight_High = 255;                // High backlight in bright surroundings
constexpr uint32_t Message_US = 2000000;               // Display messages for 2 seconds
constexpr uint32_t Bottom_Timer_Loop_US = 100000;      // Bottom timer mode: 10 Hz display refresh rate
constexpr uint32_t Dive_Computer_Loop_US = 1000000;    // Dive computer mode: 1 Hz display refresh rate
constexpr uint32_t Deco_Update_Period_US = 10000000;   // Update tissue model every 10 seconds

// QMC5883L Register Addresses
constexpr uint8_t QMC5883L_I2C_Address = 0x0D;
constexpr uint8_t QMC5883L_Status_Register = 0x06;
constexpr uint8_t QMC5883L_Status_DRDY_Mask = 0x01;

// Compass Constants
constexpr float Reference_Field_Gauss = 0.49f;                // UK average magnetic field strength
constexpr float Magnetometer_Lsb_Per_Gauss = 3000.0f;         // QMC5883L at +/- 8 Gauss range: 3000 LSB/Gauss
constexpr uint32_t Compass_Calibration_Duration_MS = 60000;   // 60 seconds for compass calibration
constexpr const char *Compass_NVS_Namespace = "compass";
constexpr const char *Calibration_Valid_Key = "cal_valid";
constexpr const char *Hard_Iron_X_Key = "hi_x";
constexpr const char *Hard_Iron_Y_Key = "hi_y";
constexpr const char *Hard_Iron_Z_Key = "hi_z";
constexpr const char *Soft_Iron_Key = "soft_iron";
constexpr const char *Reference_Mag_Gauss_Key = "ref_gauss";
constexpr const char *Lsb_Per_Gauss_Key = "lsb_gauss";
constexpr const char *Fitted_Field_Lsb_Key = "fit_lsb";

// Tap Detection Constants
constexpr float Tap_Threshold_G = 2.0f;          // Acceleration spike threshold
constexpr uint32_t Tap_Window_US = 2000000;      // 3 taps window 2 seconds
constexpr uint32_t Tap_Min_Spacing_US = 100000;  // Minimum gap between taps 100 ms

// Button Constants
constexpr uint32_t Button_Debounce_US = 50000;      // Button debounce time 50 ms
constexpr uint32_t Calibration_Delay_US = 5000000;  // Delay 5 seconds before calibration

// Depth Sensor Constants
constexpr uint16_t Density = 1020;        // EN13319 density
constexpr float MBAR_PER_ATM = 1013.25f;  // mBar per atm
constexpr float Depth_Offset = 0.2f;      // Sea level offset 0.2 m
constexpr float Dive_Start_Depth = 1.0f;  // Start timer at 1 m

// Decompression Constants
constexpr u_int8_t GF_Low = 60;
constexpr u_int8_t GF_High = 85;
constexpr float Setpoint = 1.2f;

// Display
enum DisplayMode : uint8_t { MODE_BOTTOM_TIMER, MODE_DIVE_COMPUTER };
DisplayMode currentMode = MODE_BOTTOM_TIMER;
uint64_t Dive_Display_US = 0;
uint16_t Loop_Usage_Percent = 0;
// Modes
bool ripNtear_Mode = false;
bool ripNtear_Pending = false;
// Dive metrics
float Depth = 0.0f;
float Heading = 0.0f;
int Minutes = 0;
int Seconds = 0;
bool Dive_Timer_Started = false;
uint64_t Timer_Start_US = 0;
// Battery
float Battery_Voltage = 0.0f;
uint8_t Battery_Percentage = 0;
float Ambient_Lux = 0.0f;
uint8_t Backlight_Level = Backlight_High;
// Calibration button
bool Calibration_Armed = false;
uint64_t Calibration_Start_US = 0;
uint8_t Button_Last_Reading = HIGH;
uint8_t Button_Stable_State = HIGH;
uint64_t Button_Last_Change_US = 0;
// Tap detection
uint8_t tripleTapCount = 0;
uint64_t tripleTapFirstUs = 0;
uint64_t tripleTapLastUs = 0;
bool tripleTapAboveThresh = false;
uint8_t doubleTapCount = 0;
uint64_t doubleTapFirstUs = 0;
uint64_t doubleTapLastUs = 0;
bool doubleTapAboveThresh = false;
// Decompression
DecoResult lastDecoResult = {false, 0, 0, 0, 0};
uint64_t Deco_Last_Update_US = 0;
// Compass calibration
CompassCalibrationMatrices Compass_Matrices = {};
// Calibration method
enum class CompassCalibrationMethod : uint8_t {
  None = 0,
  Ellipsoid = 1,
  MinMax = 2,
  Error = 3,
};
CompassCalibrationMethod Last_Calibration_Method = CompassCalibrationMethod::None;
static const char* compassCalibrationMethodText(CompassCalibrationMethod method) {
  switch (method) {
    case CompassCalibrationMethod::Ellipsoid:
      return "Ellipsoid";
    case CompassCalibrationMethod::MinMax:
      return "Min/Max";
    case CompassCalibrationMethod::Error:
      return "Error";
    case CompassCalibrationMethod::None:
    default:
      return "None";
  }
}
CompassCalibrationQuality Last_Calibration_Quality = {};
// Compass labels
struct CompassLabel {
  int degrees;
  const char *label;
};
const CompassLabel Compass_Labels[] = {
    {0, "N"}, {30, "30"}, {60, "60"},
    {90, "E"}, {120, "120"}, {150, "150"},
    {180, "S"}, {210, "210"}, {240, "240"},
    {270, "W"}, {300, "300"}, {330, "330"}
  };


// QMC5883L Data Ready Check
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

// Load Calibration
static bool loadCompassCalibrationFromNVS() {
  if (!prefs.begin(Compass_NVS_Namespace, true)) {
    return false;
  }
  const bool valid = prefs.getBool(Calibration_Valid_Key, false);
  if (!valid || !prefs.isKey(Hard_Iron_X_Key)) {
    prefs.end();
    return false;
  }
  CompassCalibrationMatrices loaded = {};
  loaded.hard_iron[0] = prefs.getFloat(Hard_Iron_X_Key, 0.0f);
  loaded.hard_iron[1] = prefs.getFloat(Hard_Iron_Y_Key, 0.0f);
  loaded.hard_iron[2] = prefs.getFloat(Hard_Iron_Z_Key, 0.0f);
  loaded.reference_field_gauss = prefs.getFloat(Reference_Mag_Gauss_Key, Reference_Field_Gauss);
  loaded.lsb_per_gauss = prefs.getFloat(Lsb_Per_Gauss_Key, Magnetometer_Lsb_Per_Gauss);
  loaded.fitted_field_lsb = prefs.getFloat(
      Fitted_Field_Lsb_Key,
      loaded.reference_field_gauss * loaded.lsb_per_gauss);
  const size_t soft_iron_size = prefs.getBytesLength(Soft_Iron_Key);
  if (soft_iron_size != (9U * sizeof(float))) {
    prefs.end();
    return false;
  }
  prefs.getBytes(Soft_Iron_Key, loaded.soft_iron, soft_iron_size);
  loaded.is_valid = true;
  prefs.end();
  compassConfigureReferenceField(loaded.reference_field_gauss, loaded.lsb_per_gauss);
  Compass_Matrices = loaded;
  return compassSetCalibrationMatrices(&Compass_Matrices);
}

// Save Calibration
static void saveCompassCalibrationToNVS(const CompassCalibrationMatrices& matrices) {
  if (!prefs.begin(Compass_NVS_Namespace, false)) {
    return;
  }
  prefs.putFloat(Hard_Iron_X_Key, matrices.hard_iron[0]);
  prefs.putFloat(Hard_Iron_Y_Key, matrices.hard_iron[1]);
  prefs.putFloat(Hard_Iron_Z_Key, matrices.hard_iron[2]);
  prefs.putBytes(Soft_Iron_Key, matrices.soft_iron, 9U * sizeof(float));
  prefs.putFloat(Reference_Mag_Gauss_Key, matrices.reference_field_gauss);
  prefs.putFloat(Lsb_Per_Gauss_Key, matrices.lsb_per_gauss);
  prefs.putFloat(Fitted_Field_Lsb_Key, matrices.fitted_field_lsb);
  prefs.putBool(Calibration_Valid_Key, matrices.is_valid);
  prefs.end();
}

// Compass Initialisation
static bool initCompassSensors(int sda, int scl) {
  // QMI8658
  qmi.begin(Wire, 0x6B, sda, scl);
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G,
                          SensorQMI8658::ACC_ODR_125Hz,
                          SensorQMI8658::LPF_MODE_2);
  qmi.enableAccelerometer();
  qmi.disableGyroscope();
  // QMC5883L
  qmc.init();
  qmc.setMode(0x01, 0x08, 0x10, 0x00);  // Continuous mode; ODR 100 Hz; Range 8 Gauss, OSR 512
  compassConfigureReferenceField(Reference_Field_Gauss, Magnetometer_Lsb_Per_Gauss);
  if (!loadCompassCalibrationFromNVS()) {
    Compass_Matrices = {};
    compassSetCalibrationMatrices(nullptr);
  }
  return true;
}

// Read Accelerometer
float readAccelMagnitude() {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);
  return sqrtf((ax * ax) + (ay * ay) + (az * az));
}

// Read Compass Heading
static float readCompassHeading() {
  // Tilt reading
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);
  // Magnetometer reading
  qmc.read();
  const float raw_mag[3] = {
      static_cast<float>(qmc.getX()),
      static_cast<float>(qmc.getY()),
      static_cast<float>(qmc.getZ())};
  const float accel[3] = {ax, ay, az};
  // Apply calibration and tilt compensation
  float compensated[3] = {0.0f, 0.0f, 0.0f};
  compassApplyCalibrationAndTiltCompensation(raw_mag, accel, compensated);
  return compassHeadingFromMagneticVector(compensated);
}

// Compass Calibration - Data Collection
static bool collectCompassSamples(std::vector<float>& mag_samples_xyz) {
  mag_samples_xyz.reserve(3U * 10000U);
  const uint32_t start_ms = millis();
  while ((millis() - start_ms) < Compass_Calibration_Duration_MS) {
    if (qmc5883lDataReady()) {
      qmc.read();
      mag_samples_xyz.push_back(static_cast<float>(qmc.getX()));
      mag_samples_xyz.push_back(static_cast<float>(qmc.getY()));
      mag_samples_xyz.push_back(static_cast<float>(qmc.getZ()));
    }
  }
  return true;
}

// Compass Calibration - Computation
static bool computeCompassCalibration(const std::vector<float>& mag_samples_xyz) {
  const size_t sample_count = mag_samples_xyz.size() / 3U;
  if (sample_count < 50U) {
    Last_Calibration_Method = CompassCalibrationMethod::Error;
    return false;
  }
  CompassCalibrationMatrices fitted = {};
  CompassCalibrationFitMethod fit_method = CompassCalibrationFitMethod::Error;
  if (!compassCalibrateFromSamplesWithFallback(
          mag_samples_xyz.data(),
          sample_count,
          &fitted,
          &fit_method)) {
    Last_Calibration_Method = CompassCalibrationMethod::Error;
    return false;
  }
  Compass_Matrices = fitted;
  if (!compassSetCalibrationMatrices(&Compass_Matrices)) {
    Last_Calibration_Method = CompassCalibrationMethod::Error;
    return false;
  }
  saveCompassCalibrationToNVS(Compass_Matrices);
  switch (fit_method) {
    case CompassCalibrationFitMethod::Ellipsoid:
      Last_Calibration_Method = CompassCalibrationMethod::Ellipsoid;
      break;
    case CompassCalibrationFitMethod::MinMax:
      Last_Calibration_Method = CompassCalibrationMethod::MinMax;
      break;
    default:
      Last_Calibration_Method = CompassCalibrationMethod::Error;
      break;
  }
  Last_Calibration_Quality = {};
  compassScoreCalibrationQuality(
      mag_samples_xyz.data(), sample_count, &Compass_Matrices, &Last_Calibration_Quality);
  return true;
}

// Display Centre Text 
void drawCentreText(const char *text, int16_t y, uint8_t textSize, uint16_t colour) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  display.setTextSize(textSize);
  display.setTextWrap(false);
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = ((display.width() - static_cast<int16_t>(w)) / 2) - x1;
  display.setCursor(x, y);
  display.setTextColor(colour);
  display.print(text);
}

// Display Column-Centred Text
void drawColCentreText(const char *text, int16_t colX, int16_t colW,
                               int16_t y, uint8_t textSize, uint16_t colour) {
  int16_t x1 = 0, y1 = 0;
  uint16_t w = 0, h = 0;
  display.setTextSize(textSize);
  display.setTextWrap(false);
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = colX + ((colW - static_cast<int16_t>(w)) / 2) - x1;
  display.setCursor(x, y);
  display.setTextColor(colour);
  display.print(text);
}

// Timer
void updateTimer(float Depth, uint64_t nowUs) {
  if (Depth >= Dive_Start_Depth && !Dive_Timer_Started) {
    Timer_Start_US = nowUs;
    Dive_Timer_Started = true;
  }
  if (Dive_Timer_Started) {
    const uint64_t elapsedUs = nowUs - Timer_Start_US;
    Minutes = static_cast<int>(elapsedUs / 60000000ULL);
    Seconds = static_cast<int>((elapsedUs % 60000000ULL) / 1000000ULL);
  }
}

// Battery Voltage
float readBattery() {
  constexpr uint8_t sampleCount = 16;
  uint32_t millivoltSum = 0;
  for (uint8_t sample = 0; sample < sampleCount; ++sample) {
    millivoltSum += analogReadMilliVolts(Battery_Pin);
  }
  float voltage = (millivoltSum / sampleCount) / 1000.0f * Battery_Divider_Ratio;
  return voltage;
}

// Battery Percentage
uint8_t batteryPercentage(float Battery_Voltage) {
  if (Battery_Voltage <= Voltage_Table[0]) {
    return Percentage_Table[0];
  }
  if (Battery_Voltage >= Voltage_Table[Battery_Table_Size - 1]) {
    return Percentage_Table[Battery_Table_Size - 1];
  }
  for (size_t index = 0; index < (Battery_Table_Size - 1); ++index) {
    float lower = Voltage_Table[index];
    float upper = Voltage_Table[index + 1];
    if (Battery_Voltage < upper) {
      return ((Battery_Voltage - lower) < (upper - Battery_Voltage))
                 ? Percentage_Table[index]
                 : Percentage_Table[index + 1];
    }
  }
  return 0;
}

// Battery Indicator
void drawBatteryIndicator(int16_t rightEdgeX, int16_t topY, uint8_t percentage) {
  uint16_t fillColor = ST77XX_GREEN;
  if (percentage < 10) {
    fillColor = ST77XX_RED;
  } else if (percentage < 25) {
    fillColor = ST77XX_YELLOW;
  }
  constexpr int16_t outlineW = 50;
  constexpr int16_t outlineH = 18;
  constexpr int16_t tipW = 4;
  constexpr int16_t padding = 3;
  const int16_t outlineX = rightEdgeX - outlineW - tipW;
  const int16_t outlineY = topY;
  const int16_t fillWidth = map(percentage, 0, 100, 0, outlineW - (padding * 2));
  display.drawRect(outlineX, outlineY, outlineW, outlineH, ST77XX_WHITE);
  display.fillRect(outlineX + outlineW, outlineY + 5, tipW, outlineH - 10, ST77XX_WHITE);
  display.fillRect(outlineX + padding, outlineY + padding, fillWidth, outlineH - (padding * 2), fillColor);
}

// Compass Display
void drawCompassBanner(int16_t x, int16_t y, int16_t w, int16_t h, float direction) {
  const int16_t centerX = x + (w / 2);
  const float pixelsPerDegree = static_cast<float>(w) / 120.0f;
  const int16_t topLineY = y + 24;
  const int16_t bottomLineY = y + h - 1;
  const int16_t majorTickBottomY = topLineY + 5;
  const int16_t minorTickBottomY = topLineY + 3;
  const int16_t labelY = topLineY + 8;

  display.drawFastHLine(x, topLineY, w, ST77XX_WHITE);
  display.drawFastHLine(x, bottomLineY, w, ST77XX_WHITE);

  for (int tickDegrees = 0; tickDegrees < 360; tickDegrees += 15) {
    if (tickDegrees % 30 == 0) {
      continue;
    }
    float delta = static_cast<float>(tickDegrees) - direction;
    if (delta > 180.0f) {
      delta -= 360.0f;
    }
    if (delta < -180.0f) {
      delta += 360.0f;
    }
    if (delta < -60.0f || delta > 60.0f) {
      continue;
    }
    const int16_t tickX = centerX + static_cast<int16_t>(delta * pixelsPerDegree);
    if (tickX < x || tickX >= (x + w)) {
      continue;
    }
    display.drawLine(tickX, topLineY + 1, tickX, minorTickBottomY, ST77XX_WHITE);
  }

  for (const CompassLabel &marker : Compass_Labels) {
    float delta = static_cast<float>(marker.degrees) - direction;
    if (delta > 180.0f) {
      delta -= 360.0f;
    }
    if (delta < -180.0f) {
      delta += 360.0f;
    }
    if (delta < -60.0f || delta > 60.0f) {
      continue;
    }
    const int16_t tickX = centerX + static_cast<int16_t>(delta * pixelsPerDegree);
    if (tickX < x || tickX >= (x + w)) {
      continue;
    }
    int16_t x1 = 0;
    int16_t y1 = 0;
    uint16_t labelW = 0;
    uint16_t labelH = 0;
    display.setTextSize(1);
    display.getTextBounds(marker.label, 0, 0, &x1, &y1, &labelW, &labelH);

    display.drawLine(tickX, topLineY + 1, tickX, majorTickBottomY, ST77XX_WHITE);
    display.setTextColor(ST77XX_WHITE);
    display.setCursor(tickX - static_cast<int16_t>(labelW / 2), labelY);
    display.print(marker.label);
  }
  display.fillTriangle(centerX - 4, topLineY + 2, centerX + 4, topLineY + 2, centerX, topLineY + 8, ST77XX_CYAN);
  display.drawLine(centerX, topLineY + 8, centerX, bottomLineY - 1, ST77XX_CYAN);
}

// Heading Display
void drawHeadingValue(int16_t x, int16_t y, int16_t w, float direction) {
  int heading = static_cast<int>(direction + 0.5f);
  if (heading == 360) {
    heading = 0;
  }
  char headingString[8];
  snprintf(headingString, sizeof(headingString), "-%03d-", heading);
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t textW = 0;
  uint16_t textH = 0;
  display.setTextSize(2);
  display.getTextBounds(headingString, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor(x + ((w - static_cast<int16_t>(textW)) / 2), y + 2);
  display.setTextColor(ST77XX_CYAN);
  display.print(headingString);
}

// Calibration Score Display
static void drawCalibrationScore() {
  display.fillScreen(ST77XX_BLACK);
  drawCentreText("Calibration Result", 15, 2, ST77XX_WHITE);
  if (!Last_Calibration_Quality.is_valid) {
    drawCentreText("N/A", 85, 4, ST77XX_RED);
    return;
  }
  // Score percentage colour
  const int score = static_cast<int>(Last_Calibration_Quality.score_percent + 0.5f);
  uint16_t score_colour;
  if (score >= 80) {
    score_colour = ST77XX_GREEN;
  } else if (score >= 60) {
    score_colour = ST77XX_YELLOW;
  } else if (score >= 40) {
    score_colour = ST77XX_ORANGE;
  } else {
    score_colour = ST77XX_RED;
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "%d%%", score);
  drawCentreText(buf, 50, 3, score_colour);
  snprintf(buf, sizeof(buf), "%u Sample Count: ",
           static_cast<unsigned int>(Last_Calibration_Quality.used_sample_count));
  drawCentreText(buf, 100, 2, ST77XX_CYAN);
  snprintf(buf, sizeof(buf), "Span:  %d%%",
           static_cast<int>(Last_Calibration_Quality.raw_span_score + 0.5f));
  drawCentreText(buf, 138, 1, ST77XX_WHITE);
  snprintf(buf, sizeof(buf), "Oct:   %d%%",
           static_cast<int>(Last_Calibration_Quality.octant_coverage_score + 0.5f));
  drawCentreText(buf, 150, 1, ST77XX_WHITE);
  snprintf(buf, sizeof(buf), "PCA: %d%%",
           static_cast<int>(Last_Calibration_Quality.unit_vector_pca_ratio_score + 0.5f));
  drawCentreText(buf, 162, 1, ST77XX_WHITE);
  snprintf(buf, sizeof(buf), "Residual: %d%%",
           static_cast<int>(Last_Calibration_Quality.ellipsoid_residual_score + 0.5f));
  drawCentreText(buf, 174, 1, ST77XX_WHITE);
  snprintf(buf, sizeof(buf), "Rad Std:   %d%%",
           static_cast<int>(Last_Calibration_Quality.calibrated_radius_std_score + 0.5f));
  drawCentreText(buf, 186, 1, ST77XX_WHITE);
}

// Compass Calibration Button
bool calibrationButtonPressed(uint64_t nowUs) {
  const uint8_t reading = static_cast<uint8_t>(digitalRead(Calibration_Button_Pin));
  if (reading != Button_Last_Reading) {
    Button_Last_Change_US = nowUs;
    Button_Last_Reading = reading;
  }
  if ((nowUs - Button_Last_Change_US) >= Button_Debounce_US) {
    if (reading != Button_Stable_State) {
      Button_Stable_State = reading;
      if (Button_Stable_State == LOW) {
        return true;
      }
    }
  }
  return false;
}

// Backlight Level
uint8_t ambientBacklightLevel(float lux) {
  if (lux <= 0.0f) {
    return Backlight_Low;
  }
  float normalised = log10f(lux + 1.0f) / log10f(Ambient_Lux_Max + 1.0f);
  if (normalised < 0.0f) normalised = 0.0f;
  if (normalised > 1.0f) normalised = 1.0f;
  return static_cast<uint8_t>(Backlight_Low + normalised * static_cast<float>(Backlight_High - Backlight_Low));
}

// Bottom Timer Display Update
void updateDisplay(float Depth, int Minutes, int Seconds, float Battery_Voltage) {
  display.fillScreen(ST77XX_BLACK);
  const int16_t screenW = display.width();
  const int16_t screenH = display.height();
  const int16_t topAreaH = (screenH * 2) / 3;
  constexpr int16_t rightPanelW = 92;

  // Depth strings
  const int depthInteger = static_cast<int>(Depth);
  int depthDecimal = static_cast<int>(Depth * 10.0f + 0.5f) % 10;
  if (depthDecimal < 0) {
    depthDecimal += 10;
  }
  char integerPart[4];
  char decimalPart[4];
  const char *unitPart = "m";
  snprintf(integerPart, sizeof(integerPart), "%d", depthInteger);
  snprintf(decimalPart, sizeof(decimalPart), "%d", depthDecimal);
  constexpr uint8_t integerSize = 15;
  constexpr uint8_t decimalSize = 7;
  constexpr uint8_t dotSize = 4;
  const char *dotPart = ".";
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t integerW = 0;
  uint16_t integerH = 0;
  uint16_t decimalW = 0;
  uint16_t decimalH = 0;
  uint16_t unitW = 0;
  uint16_t unitH = 0;
  uint16_t dotW = 0;
  uint16_t dotH = 0;
  display.setTextSize(integerSize);
  display.getTextBounds(integerPart, 0, 0, &x1, &y1, &integerW, &integerH);
  display.setTextSize(decimalSize);
  display.getTextBounds(decimalPart, 0, 0, &x1, &y1, &decimalW, &decimalH);
  display.setTextSize(decimalSize);
  display.getTextBounds(unitPart, 0, 0, &x1, &y1, &unitW, &unitH);
  display.setTextSize(dotSize);
  display.getTextBounds(dotPart, 0, 0, &x1, &y1, &dotW, &dotH);
  const int16_t depthX = 6;
  const int16_t depthY = 6;
  const int16_t depthBottom = depthY + static_cast<int16_t>(integerH);
  const int16_t dotX = depthX + static_cast<int16_t>(integerW) + 2;
  const int16_t dotY = depthBottom - static_cast<int16_t>(dotH) - 8;
  const int16_t decimalX = dotX + static_cast<int16_t>(dotW) + 2;
  const int16_t decimalY = depthBottom - static_cast<int16_t>(decimalH);
  const int16_t unitX = decimalX + static_cast<int16_t>(decimalW) + 2;
  const int16_t unitY = depthBottom - static_cast<int16_t>(unitH);

  // Draw depth block on top-left
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(integerSize);
  display.setCursor(depthX, depthY);
  display.print(integerPart);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(dotSize);
  display.setCursor(dotX, dotY);
  display.print(dotPart);
  display.setTextSize(decimalSize);
  display.setCursor(decimalX, decimalY);
  display.print(decimalPart);
  display.setTextColor(ST77XX_CYAN);
  display.setTextSize(decimalSize);
  display.setCursor(unitX, unitY);
  display.print(unitPart);

  // Draw battery indicator on top-right
  const int16_t rightEdge = screenW - 8;
  drawBatteryIndicator(rightEdge, 10, Battery_Percentage);

  // Draw loop usage percent under battery (% of 100 ms)
  char loopUsageStr[8];
  snprintf(loopUsageStr, sizeof(loopUsageStr), "%u%%", Loop_Usage_Percent);
  display.setTextSize(2);
  display.setTextColor(ST77XX_CYAN);
  {
    int16_t ux1 = 0, uy1 = 0;
    uint16_t uW = 0, uH = 0;
    display.getTextBounds(loopUsageStr, 0, 0, &ux1, &uy1, &uW, &uH);
    display.setCursor(rightEdge - static_cast<int16_t>(uW), 29);
    display.print(loopUsageStr);
  }

  // Draw timer below battery indicator
  char timerString[8];
  snprintf(timerString, sizeof(timerString), "%3d:%02d", Minutes, Seconds);
  display.setTextSize(4);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(screenW - rightPanelW + 4, 56);
  display.print(timerString);

  // Draw compass on bottom
  const int16_t compassY = topAreaH;
  const int16_t compassH = screenH - topAreaH;
  display.drawFastHLine(0, compassY, screenW, ST77XX_WHITE);
  drawHeadingValue(0, compassY, screenW, Heading);
  drawCompassBanner(0, compassY, screenW, compassH, Heading);
}

// Triple-tap
bool detectTripleTap(uint64_t nowUs) {
  const float mag = readAccelMagnitude();
  if (tripleTapCount > 0 && (nowUs - tripleTapFirstUs) > Tap_Window_US) {
    tripleTapCount= 0;
  }
  bool newTap = false;
  if (mag > Tap_Threshold_G) {
    if (!tripleTapAboveThresh) {
      tripleTapAboveThresh = true;
      if (tripleTapCount == 0 || (nowUs - tripleTapLastUs) >= Tap_Min_Spacing_US) {
        newTap = true;
      }
    }
  } else {
    tripleTapAboveThresh = false;
  }
  if (newTap) {
    if (tripleTapCount == 0) tripleTapFirstUs = nowUs;
    tripleTapCount++;
    tripleTapLastUs = nowUs;
    if (tripleTapCount >= 3) {
      tripleTapCount = 0;
      return true;
    }
  }
  return false;
}

// Double-tap
bool detectDoubleTap(uint64_t nowUs) {
  const float mag = readAccelMagnitude();
  if (doubleTapCount > 0 && (nowUs - doubleTapFirstUs) > Tap_Window_US) {
    doubleTapCount = 0;
  }
  bool newTap = false;
  if (mag > Tap_Threshold_G) {
    if (!doubleTapAboveThresh) {
      doubleTapAboveThresh = true;
      if (doubleTapCount == 0 || (nowUs - doubleTapLastUs) >= Tap_Min_Spacing_US) {
        newTap = true;
      }
    }
  } else {
    doubleTapAboveThresh = false;
  }
  if (newTap) {
    if (doubleTapCount == 0) doubleTapFirstUs = nowUs;
    doubleTapCount++;
    doubleTapLastUs = nowUs;
    if (doubleTapCount >= 2) {
      doubleTapCount = 0;
      return true;
    }
  }
  return false;
}

// Dive Computer Display Update
void updateDiveComputerDisplay(float depth, int minutes, int seconds, float battVoltage, const DecoResult &deco) {
  display.fillScreen(ST77XX_BLACK);

  // Depth at top-left
  const int depthInt = static_cast<int>(depth);
  int depthDec = static_cast<int>(roundf(depth * 10.0f)) % 10;
  if (depthDec < 0) depthDec += 10;
  char depthNum[8];
  snprintf(depthNum, sizeof(depthNum), "%d.%d", depthInt, depthDec);
  display.setTextSize(4);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(5, 8);
  display.print(depthNum);
  display.setTextColor(ST77XX_CYAN);
  display.print("m");

  // Battery indicator at top-right
  drawBatteryIndicator((LCD_Width - 8), 5, Battery_Percentage);

  // Loop usage percent under battery (% of 100 ms)
  char loopUsageStr[8];
  snprintf(loopUsageStr, sizeof(loopUsageStr), "%u%%", Loop_Usage_Percent);
  display.setTextSize(2);
  display.setTextColor(ST77XX_CYAN);
  {
    int16_t ux1 = 0, uy1 = 0;
    uint16_t uW = 0, uH = 0;
    display.getTextBounds(loopUsageStr, 0, 0, &ux1, &uy1, &uW, &uH);
    display.setCursor(((LCD_Width - 8) - static_cast<int16_t>(uW)), 24);
    display.print(loopUsageStr);
  }

  // Timer below battery and loop usage
  char timerStr[8];
  snprintf(timerStr, sizeof(timerStr), "%d:%02d", minutes, seconds);
  {
    int16_t tx1 = 0, ty1 = 0;
    uint16_t tW = 0, tH = 0;
    display.setTextSize(3);
    display.getTextBounds(timerStr, 0, 0, &tx1, &ty1, &tW, &tH);
    display.setCursor(LCD_Width - 8 - static_cast<int16_t>(tW), 44);
    display.setTextColor(ST77XX_WHITE);
    display.print(timerStr);
  }

  // Current GF at mid left
  char gfStr[12];
  if (ripNtear_Mode) {
    snprintf(gfStr, sizeof(gfStr), "GF 99/99");
  } else {
    snprintf(gfStr, sizeof(gfStr), "GF 60/85");
  }
  display.setTextSize(1);
  display.setTextColor(ripNtear_Mode ? ST77XX_RED : ST77XX_GREEN);
  display.setCursor(8, 50);
  display.print(gfStr);

  // Horizontal divider
  constexpr int16_t divY = 58;
  display.drawFastHLine(0, divY, LCD_Width, ST77XX_WHITE);

  // Deco section
  constexpr int16_t col1X = 0, col1W = 60;
  constexpr int16_t col2X = 60, col2W = 60;
  constexpr int16_t col3X = 120, col3W = 60;
  constexpr int16_t col4X = 180, col4W = 60;
  display.drawFastVLine(col2X, divY + 1, LCD_Height - divY - 1, ST77XX_WHITE);
  display.drawFastVLine(col3X, divY + 1, LCD_Height - divY - 1, ST77XX_WHITE);
  display.drawFastVLine(col4X, divY + 1, LCD_Height - divY - 1, ST77XX_WHITE);
  // Surface GF colour
  uint16_t surfGfColor = ST77XX_GREEN;
  if (deco.surfGF > 100) {
    surfGfColor = ST77XX_RED;
  } else if (deco.surfGF > 85) {
    surfGfColor = ST77XX_YELLOW;
  }
  // Deco display
  if (!deco.inDeco) {
    // NO DECO at top centre
    drawCentreText("NO DECO", 78, 4, ST77XX_GREEN);
    // TTS at bottom left
    char ttsStr[16];
    snprintf(ttsStr, sizeof(ttsStr), "TTS:%d", deco.timeToSurface);
    display.setTextSize(3);
    display.setTextColor(ST77XX_CYAN);
    display.setCursor(8, 210);
    display.print(ttsStr);
    // Surface GF at bottom right
    char surfGfStr[16];
    snprintf(surfGfStr, sizeof(surfGfStr), "sGF:%u%%", deco.surfGF);
    int16_t sx1 = 0, sy1 = 0;
    uint16_t sw = 0, sh = 0;
    display.setTextSize(3);
    display.getTextBounds(surfGfStr, 0, 0, &sx1, &sy1, &sw, &sh);
    display.setTextColor(surfGfColor);
    display.setCursor(LCD_Width - 8 - static_cast<int16_t>(sw), 210);
    display.print(surfGfStr);
  } else {
    // Full 4 column deco table
    drawColCentreText("STOP", col1X, col1W, 66, 2, ST77XX_CYAN);
    drawColCentreText("TIME", col2X, col2W, 66, 2, ST77XX_CYAN);
    drawColCentreText("TTS", col3X, col3W, 66, 2, ST77XX_CYAN);
    drawColCentreText("sGF", col4X, col4W, 66, 2, ST77XX_CYAN);
    char stopStr[8], timeStr[8], ttsStr[8], surfGfValStr[8];
    snprintf(stopStr, sizeof(stopStr), "%um", deco.nextStopDepth);
    snprintf(timeStr, sizeof(timeStr), "%dm", deco.stopTime);
    snprintf(ttsStr, sizeof(ttsStr), "%dm", deco.timeToSurface);
    snprintf(surfGfValStr, sizeof(surfGfValStr), "%u%%", deco.surfGF);
    drawColCentreText(stopStr, col1X, col1W, 94, 3, ST77XX_WHITE);
    drawColCentreText(timeStr, col2X, col2W, 94, 3, ST77XX_WHITE);
    drawColCentreText(ttsStr, col3X, col3W, 94, 3, ST77XX_WHITE);
    drawColCentreText(surfGfValStr, col4X, col4W, 94, 3, surfGfColor);
  }
}


void setup() {
  // Power Conservation
  esp_wifi_stop();              // WiFi off
  esp_bt_controller_disable();  // Bluetooth off
  setCpuFrequencyMhz(80);       // Reduce CPU frequency

  // ADC
  pinMode(Battery_Pin, INPUT);
  analogReadResolution(12);        // Internal ADC resolution 12-bit
  analogSetAttenuation(ADC_11db);  // 2.5V range

  // I2C Initialisation
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(400000);  // I2C clock speed 400kHz

  // MS5837 Initialisation
  sensor.init();
  sensor.setModel(MS5837::MS5837_30BA);  // 30 bar model
  sensor.setFluidDensity(Density);       // Set water density

  // BH1750 Initialisation
  lightMeter.begin(BH1750::CONTINUOUS_LOW_RES_MODE);  // 4 lux, 16 ms

  // QMI8658 and QMC5883L Initialisation
  initCompassSensors(SDA_Pin, SCL_Pin);

  // ZHL-16C Initialisation
  decoSetup(GF_Low, GF_High, Setpoint);
  decoInit();
  Deco_Last_Update_US = static_cast<uint64_t>(esp_timer_get_time());

  // Display Initialisation
  spi.begin(SCK_Pin, -1, MOSI_Pin, CS_Pin);
  pinMode(Calibration_Button_Pin, INPUT_PULLUP);
  pinMode(Backlight_Pin, OUTPUT);
  analogWrite(Backlight_Pin, Backlight_High);
  display.init(LCD_Width, LCD_Height, SPI_MODE3);
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);
  drawCentreText("Bottom", 70, 4, ST77XX_WHITE);
  drawCentreText("Timer", 126, 4, ST77XX_WHITE);
  esp_sleep_enable_timer_wakeup(Message_US);
  esp_light_sleep_start();
}


void loop() {
  const uint64_t loopStartUs = static_cast<uint64_t>(esp_timer_get_time());
  bool modeChanged = false;
  bool tripleTapTriggered = false;

  // Bottom Timer or Diver Computer
  if (detectTripleTap(loopStartUs)) {
    currentMode = (currentMode == MODE_BOTTOM_TIMER) ? MODE_DIVE_COMPUTER : MODE_BOTTOM_TIMER;
    modeChanged = true;
    tripleTapTriggered = true;
  }

  //  Rip & Tear Mode (no more GF, raw Bühlmann ZHL-16C algorithm)
  if (currentMode == MODE_DIVE_COMPUTER) {
    if (tripleTapTriggered) {
      ripNtear_Pending = false;
    } else if (detectDoubleTap(loopStartUs)) {
      ripNtear_Pending = true;
    }
    if (ripNtear_Pending && tripleTapCount == 0) {
      ripNtear_Mode = !ripNtear_Mode;
      ripNtear(ripNtear_Mode);
      ripNtear_Pending = false;
      Dive_Display_US = loopStartUs - Dive_Computer_Loop_US;
      Deco_Last_Update_US = loopStartUs - Deco_Update_Period_US;
    }
  } else {
    ripNtear_Pending = false;
  }

  // Recalculate decompression when switched to dive computer mode
  if (modeChanged && currentMode == MODE_DIVE_COMPUTER) {
    Dive_Display_US = loopStartUs - Dive_Computer_Loop_US;
    Deco_Last_Update_US = loopStartUs - Deco_Update_Period_US;
  }

  // Compass Calibration
  if (calibrationButtonPressed(loopStartUs) && !Calibration_Armed) {
    Calibration_Armed = true;
    Calibration_Start_US = loopStartUs;
  }
  if (Calibration_Armed) {
    const uint64_t elapsedUs = loopStartUs - Calibration_Start_US;
    // Start Calibration
    if (elapsedUs < Calibration_Delay_US) {
      display.fillScreen(ST77XX_BLACK);
      drawCentreText("Compass", 70, 4, ST77XX_WHITE);
      drawCentreText("Calibration", 126, 4, ST77XX_WHITE);
      delay(100);
      return;
    }
    // Data collection
    display.fillScreen(ST77XX_BLACK);
    drawCentreText("Calibrating", 98, 4, ST77XX_CYAN);
    std::vector<float> mag_samples_xyz;
    collectCompassSamples(mag_samples_xyz);
    // Computing calibration
    display.fillScreen(ST77XX_BLACK);
    drawCentreText("Computing", 98, 4, ST77XX_YELLOW);
    const bool calibration_ok = computeCompassCalibration(mag_samples_xyz);
    // Calibration complete
    display.fillScreen(ST77XX_BLACK);
    if (calibration_ok) {
      drawCentreText("Done", 98, 4, ST77XX_GREEN);
    } else {
      drawCentreText("Failed", 98, 4, ST77XX_RED);
    }
    drawCentreText(compassCalibrationMethodText(Last_Calibration_Method), 140, 2, ST77XX_WHITE);
    esp_sleep_enable_timer_wakeup(Message_US);
    esp_light_sleep_start();
    drawCalibrationScore();
    esp_sleep_enable_timer_wakeup(Message_US * 3);
    esp_light_sleep_start();
    Calibration_Armed = false;
    return;
  }

  // Depth
  sensor.read();
  const float pressureAtm = sensor.pressure() / MBAR_PER_ATM;
  Depth = sensor.depth() - Depth_Offset;  // Sea level offset
  if (Depth < 0.0f) Depth = 0.0f;         // Minimum depth 0 meter
  if (Depth > 99.9f) Depth = 99.9f;       // Maximum depth 99.9 meters

  // Timer
  updateTimer(Depth, loopStartUs);

  // Compass
  Heading = readCompassHeading();

  // Backlight
  Ambient_Lux = lightMeter.readLightLevel();

  // Battery
  Battery_Voltage = readBattery();
  Battery_Percentage = batteryPercentage(Battery_Voltage);

  // ZHL-16C
  if ((loopStartUs - Deco_Last_Update_US) >= Deco_Update_Period_US) {
    const float dtMin = static_cast<float>(loopStartUs - Deco_Last_Update_US) / 60000000.0f;
    Deco_Last_Update_US = loopStartUs;
    decoUpdate(pressureAtm, dtMin);
    if (currentMode == MODE_DIVE_COMPUTER) {
      lastDecoResult = decoCompute(pressureAtm);
    }
  }

  // Low Battery
  if (Battery_Voltage < Low_Battery_Threshold) {
    Backlight_Level = Backlight_Low;
    analogWrite(Backlight_Pin, Backlight_Level);
  } else {
    Backlight_Level = ambientBacklightLevel(Ambient_Lux);
    analogWrite(Backlight_Pin, Backlight_Level);
  }
  if (Battery_Voltage < Critical_Battery_Threshold) {
    display.fillScreen(ST77XX_BLACK);
    drawCentreText("Battery", 86, 4, ST77XX_RED);
    drawCentreText("Low", 144, 4, ST77XX_RED);
    esp_sleep_enable_timer_wakeup(Message_US);
    esp_light_sleep_start();
    digitalWrite(Backlight_Pin, LOW);
    esp_deep_sleep_start();
  }

  // Display Update
  if (currentMode == MODE_DIVE_COMPUTER) {
    if ((loopStartUs - Dive_Display_US) >= Dive_Computer_Loop_US) {
      updateDiveComputerDisplay(Depth, Minutes, Seconds, Battery_Voltage, lastDecoResult);
      Dive_Display_US = loopStartUs;
    }
  } else {
    updateDisplay(Depth, Minutes, Seconds, Battery_Voltage);
  }

  // Loop Usage
  const uint64_t elapsedUs = static_cast<uint64_t>(esp_timer_get_time()) - loopStartUs;
  const uint64_t roundedUsage =
      (elapsedUs * 100ULL + (static_cast<uint64_t>(Bottom_Timer_Loop_US) / 2ULL)) /
      static_cast<uint64_t>(Bottom_Timer_Loop_US);
  Loop_Usage_Percent = static_cast<uint16_t>((roundedUsage > 999ULL) ? 999ULL : roundedUsage);

  // Loop Interval
  if (elapsedUs < static_cast<int64_t>(Bottom_Timer_Loop_US)) {
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(Bottom_Timer_Loop_US) - static_cast<uint64_t>(elapsedUs));
    esp_light_sleep_start();
  }
}