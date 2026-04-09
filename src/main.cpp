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
#include <SensorQMI8658.hpp>
#include <SensorQMC5883P.hpp>
// Data Processing
#include <vector>
#include <ZHL16C.h>
#include <Preferences.h>
#include <magnetometer_calibration.h>

// Pins
constexpr uint8_t Battery_Pin = 1;
constexpr uint8_t DRDY_Pin = 2;
constexpr uint8_t DC_Pin = 4;
constexpr uint8_t CS_Pin = 5;
constexpr uint8_t SCK_Pin = 6;
constexpr uint8_t MOSI_Pin = 7;
constexpr uint8_t RST_Pin = 8;
constexpr uint8_t Backlight_Pin = 15;
constexpr uint8_t SCL_Pin = 10;
constexpr uint8_t SDA_Pin = 11;
constexpr uint8_t Sensor_VCC_Pin = 3;
constexpr uint8_t Sensor_GND_Pin = 16;
constexpr uint8_t Sensor_SCL_Pin = 17;
constexpr uint8_t Sensor_SDA_Pin = 18;
constexpr uint8_t Calibration_Button_Pin = 40;

// Define Objects
SPIClass spi(FSPI);
TwoWire sensorWire(1);
Preferences prefs;
MS5837 sensor;
BH1750 lightMeter;
SensorQMI8658 qmi;
SensorQMC5883P qmc;
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

// I2C Addresses
constexpr uint8_t QMI8658_I2C_Address = 0x6B;
constexpr uint8_t QMC5883P_I2C_Address = 0x2C;
constexpr uint8_t BH1750_I2C_Address = 0x23;

// LCD Constants
constexpr uint16_t LCD_Width = 260;
constexpr uint16_t LCD_Height = 280;
constexpr uint8_t Low_Battery_Threshold = 20;       // Dim screen below 20% battery
constexpr uint8_t Critical_Battery_Threshold = 0;   // Deep sleep below 0% battery
constexpr float Battery_Divider_Ratio = 3.0f;       // 2:1 voltage divider
constexpr float Ambient_Lux_Max = 800.0f;           // Lux level at high backlight
constexpr uint8_t Backlight_Low = 8;                // Low backlight in dark surroundings
constexpr uint8_t Backlight_High = 255;             // High backlight in bright surroundings
constexpr uint32_t Message_MS = 2000;               // Display messages for 2 seconds
constexpr uint32_t Loop_MS = 100;                   // Bottom timer mode: 10 Hz refresh rate
constexpr uint32_t Deco_Update_MS = 10000;          // Update tissue model every 10 seconds

// Compass Constants
constexpr float Reference_Field_Gauss = 0.49f;                // UK average magnetic field strength
constexpr float Magnetometer_Lsb_Per_Gauss = 15000.0f;        // QMC5883P at FS_2G: 15000 LSB/Gauss
constexpr uint32_t Compass_Calibration_Duration_MS = 120000;  // 120 seconds for compass calibration
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
constexpr float Tap_Threshold_G = 0.3f;                 // Envelope trigger threshold
constexpr float Tap_Release_Threshold_G = 0.2f;         // Hysteresis release threshold
constexpr float Tap_Gravity_LPF_Alpha = 0.94f;          // Gravity adaptation
constexpr float Tap_Envelope_Decay_G_Per_MS = 0.0012f;  // Envelope fall speed
constexpr uint8_t Tap_Burst_Samples = 6;                // Burst-read to catch short impulses
constexpr uint16_t Tap_Burst_Spacing_MS = 2;            // 2 ms between burst samples
constexpr uint32_t Tap_Window_MS = 3000;                // 3 taps window
constexpr uint32_t Tap_Min_Spacing_MS = 200;            // Minimum gap between taps

// Button Constants
constexpr uint32_t Button_Debounce_MS = 50;         // Button debounce time 50 ms
constexpr uint32_t Calibration_Delay_MS = 5000;     // Delay 5 seconds before calibration

// Depth Sensor Constants
constexpr uint16_t Density = 1020;        // EN13319 density
constexpr float MBAR_PER_ATM = 1013.25f;  // mBar per atm
constexpr float Depth_Offset = 0.2f;      // Sea level offset 0.2 m
constexpr float Dive_Start_Depth = 1.0f;  // Start timer at 1 m


// --- TESTING --- //
constexpr uint8_t MS5837_I2C_Address = 0x76;
constexpr float Fake_Depth_Meters = 98.0f;
bool Pressure_Sensor_Available = false;
// --- TESTING --- //


// ZHL-16C Constants
constexpr u_int8_t GF_Low = 60;
constexpr u_int8_t GF_High = 85;
constexpr float Setpoint = 1.2f;

// Loop usage
uint16_t Loop_Usage_Percent = 0;
// Modes
bool ripNtear_Mode = false;
// Dive metrics
float Depth = 0.0f;
float Heading = 0.0f;
int Minutes = 0;
int Seconds = 0;
bool Dive_Timer_Started = false;
uint64_t Timer_Start_MS = 0;
// Battery
float Battery_Voltage = 0.0f;
uint8_t Battery_Percentage = 0;
float Ambient_Lux = 0.0f;
uint8_t Backlight_Level = Backlight_High;
// Calibration button
bool Calibration_Armed = false;
uint64_t Calibration_Start_MS = 0;
uint8_t Button_Last_Reading = HIGH;
uint8_t Button_Stable_State = HIGH;
uint64_t Button_Last_Change_MS = 0;
// Tap detection
uint8_t tripleTapCount = 0;
uint64_t tripleTapFirstMS = 0;
uint64_t tripleTapLastMS = 0;
bool tripleTapAboveThresh = false;
float Tap_Instant_Signal_G = 0.0f;
// Decompression
DecoResult lastDecoResult = {false, 0, 0, 0, 0};
uint64_t Deco_Last_Update_MS = 0;
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
    {0, "N"},
    {90, "E"},
    {180, "S"},
    {270, "W"}
  };
// Battery Percentage Lookup Table
constexpr float Voltage_Table[] = {
    3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
    3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
constexpr uint8_t Percentage_Table[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
    55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
constexpr size_t Battery_Table_Size = sizeof(Voltage_Table) / sizeof(Voltage_Table[0]);


// --- TESTING --- //
static bool isMs5837Present() {
  sensorWire.beginTransmission(MS5837_I2C_Address);
  return sensorWire.endTransmission() == 0;
}
static float depthToPressureMbar(float depth_m) {
  const float hydrostatic_pa = static_cast<float>(Density) * 9.80665f * depth_m;
  return MBAR_PER_ATM + (hydrostatic_pa / 100.0f);
}
static void readDepthAndPressure(float& depth_m, float& pressure_mbar) {
  if (Pressure_Sensor_Available) {
    sensor.read();
    pressure_mbar = sensor.pressure();
    depth_m = sensor.depth() - Depth_Offset;
  } else {
    depth_m = Fake_Depth_Meters;
    pressure_mbar = depthToPressureMbar(Fake_Depth_Meters + Depth_Offset);
  }
}
// --- TESTING --- //


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

// Compass Calibration Button
bool calibrationButtonPressed(uint64_t nowMS) {
  const uint8_t reading = static_cast<uint8_t>(digitalRead(Calibration_Button_Pin));
  if (reading != Button_Last_Reading) {
    Button_Last_Change_MS = nowMS;
    Button_Last_Reading = reading;
  }
  if ((nowMS - Button_Last_Change_MS) >= Button_Debounce_MS) {
    if (reading != Button_Stable_State) {
      Button_Stable_State = reading;
      if (Button_Stable_State == LOW) {
        return true;
      }
    }
  }
  return false;
}

// Read X-axis tap envelope
float readTapSignalX(uint64_t nowMS) {
  static bool gravityInitialised = false;
  static float gravityX = 0.0f;
  static float envelope = 0.0f;
  static uint64_t lastNowMS = 0;
  static bool sampledForCurrentNow = false;
  static uint64_t sampledNowMS = 0;
  if (sampledForCurrentNow && sampledNowMS == nowMS) {
    return envelope;
  }
  float peakSignal = 0.0f;
  float lastSignal = 0.0f;
  for (uint8_t sample = 0; sample < Tap_Burst_Samples; ++sample) {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    qmi.getAccelerometer(ax, ay, az);
    if (!gravityInitialised) {
      gravityX = ax;
      gravityInitialised = true;
      lastNowMS = nowMS;
    }
    gravityX = (Tap_Gravity_LPF_Alpha * gravityX) + ((1.0f - Tap_Gravity_LPF_Alpha) * ax);
    const float dynamicX = ax - gravityX;
    const float signal = fabsf(dynamicX);
    if (signal > peakSignal) {
      peakSignal = signal;
    }
    lastSignal = signal;
    if ((sample + 1U) < Tap_Burst_Samples) {
      delay(Tap_Burst_Spacing_MS);
    }
  }
  Tap_Instant_Signal_G = lastSignal;
  uint64_t dtMS = nowMS - lastNowMS;
  if (dtMS > 1000ULL) {
    dtMS = 1000ULL;
  }
  lastNowMS = nowMS;
  const float decay = Tap_Envelope_Decay_G_Per_MS * static_cast<float>(dtMS);
  if (envelope > decay) {
    envelope -= decay;
  } else {
    envelope = 0.0f;
  }
  if (peakSignal > envelope) {
    envelope = peakSignal;
  }
  sampledForCurrentNow = true;
  sampledNowMS = nowMS;
  return envelope;
}

// Triple-tap
bool detectTripleTap(uint64_t nowMS) {
  const float tapSignal = readTapSignalX(nowMS);
  if (tripleTapCount > 0 && (nowMS - tripleTapFirstMS) > Tap_Window_MS) {
    tripleTapCount= 0;
    tripleTapAboveThresh = false;
  }
  bool newTap = false;
  if (tapSignal > Tap_Threshold_G) {
    if (!tripleTapAboveThresh) {
      tripleTapAboveThresh = true;
      if (tripleTapCount == 0 || (nowMS - tripleTapLastMS) >= Tap_Min_Spacing_MS) {
        newTap = true;
      }
    }
  } else if (Tap_Instant_Signal_G < Tap_Release_Threshold_G) {
    tripleTapAboveThresh = false;
  }
  if (newTap) {
    if (tripleTapCount == 0) tripleTapFirstMS = nowMS;
    tripleTapCount++;
    tripleTapLastMS = nowMS;
    if (tripleTapCount >= 3) {
      tripleTapCount = 0;
      return true;
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

// Read Magnetometer
static void readQmcAxesTransformed(float out_mag[3]) {
  MagnetometerData mag_data = {};
  qmc.readData(mag_data);
  const float qmc_x = static_cast<float>(mag_data.raw.x);
  const float qmc_y = static_cast<float>(mag_data.raw.y);
  const float qmc_z = static_cast<float>(mag_data.raw.z);
  out_mag[0] = -qmc_y;  // X_qmi = -Y_qmc
  out_mag[1] = qmc_x;   // Y_qmi = X_qmc
  out_mag[2] = qmc_z;   // Z_qmi = Z_qmc
}

// Read Compass Heading
static float readCompassHeading() {
  // Tilt reading
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);
  // Magnetometer reading
  float raw_mag[3] = {0.0f, 0.0f, 0.0f};
  readQmcAxesTransformed(raw_mag);
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
    if (digitalRead(DRDY_Pin) == HIGH) {
      float raw_mag[3] = {0.0f, 0.0f, 0.0f};
      readQmcAxesTransformed(raw_mag);
      mag_samples_xyz.push_back(raw_mag[0]);
      mag_samples_xyz.push_back(raw_mag[1]);
      mag_samples_xyz.push_back(raw_mag[2]);
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
void drawColCentreText(const char *text, int16_t colX, int16_t colW, int16_t y, uint8_t textSize, uint16_t colour) {
  int16_t x1 = 0, y1 = 0;
  uint16_t w = 0, h = 0;
  display.setTextSize(textSize);
  display.setTextWrap(false);
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = colX + ((colW - static_cast<int16_t>(w)) / 2) - x1;
  display.setCursor(x, y);
  display.setTextColor(colour, ST77XX_BLACK);
  display.print(text);
}

// Timer
void updateTimer(float Depth, uint64_t nowMS) {
  if (Depth >= Dive_Start_Depth && !Dive_Timer_Started) {
    Timer_Start_MS = nowMS;
    Dive_Timer_Started = true;
  }
  if (Dive_Timer_Started) {
    const uint64_t elapsedMS = nowMS - Timer_Start_MS;
    Minutes = static_cast<int>(elapsedMS / 60000ULL);
    if (Minutes > 999) Minutes = 999;
    Seconds = static_cast<int>((elapsedMS % 60000ULL) / 1000ULL);
    if (Seconds > 59) Seconds = 59;
  }
}

// Battery Voltage
uint8_t readBatteryPercentage() {
  constexpr uint8_t sampleCount = 16;
  uint32_t millivoltSum = 0;
  for (uint8_t i = 0; i < sampleCount; ++i) {
    millivoltSum += analogReadMilliVolts(Battery_Pin);
  }
  float voltage = (millivoltSum / sampleCount) / 1000.0f * Battery_Divider_Ratio;
  if (voltage <= Voltage_Table[0]) {
    return Percentage_Table[0]; 
  }
  if (voltage >= Voltage_Table[Battery_Table_Size - 1]) {
    return Percentage_Table[Battery_Table_Size - 1];
  }
  for (size_t i = 0; i < Battery_Table_Size - 1; ++i) {
    float lower = Voltage_Table[i];
    float upper = Voltage_Table[i + 1];
    if (voltage < upper) {
      return ((voltage - lower) < (upper - voltage))
                 ? Percentage_Table[i]
                 : Percentage_Table[i + 1];
    }
  }
  return 0;
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
void drawBatteryIndicator(uint8_t percentage) {
  uint16_t fillColor = ST77XX_GREEN;
  if (percentage < 10) {
    fillColor = ST77XX_RED;
  } else if (percentage < 25) {
    fillColor = ST77XX_YELLOW;
  }
  constexpr int16_t outlineW = 40;
  constexpr int16_t outlineH = 12;
  constexpr int16_t tipW = 3;
  constexpr int16_t padding = 3;
  const int16_t outlineX = 216;
  const int16_t outlineY = 16;
  const int16_t fillWidth = map(percentage, 0, 100, 0, outlineW - (padding * 2));
  display.fillRect(outlineX + padding, outlineY + padding, outlineW - (padding * 2), outlineH - (padding * 2), ST77XX_BLACK);
  display.drawRect(outlineX, outlineY, outlineW, outlineH, ST77XX_WHITE);
  display.fillRect(outlineX + outlineW, outlineY + 5, tipW, outlineH - 10, ST77XX_WHITE);
  display.fillRect(outlineX + padding, outlineY + padding, fillWidth, outlineH - (padding * 2), fillColor);
}

// Heading Display
void drawHeadingValue(int16_t y, float direction) {
  int heading = static_cast<int>(direction + 0.5f);
  if (heading == 360) {
    heading = 0;
  }
  char headingString[6];
  snprintf(headingString, sizeof(headingString), "-%03u-", heading);
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t textW = 0;
  uint16_t textH = 0;
  display.setTextSize(3);
  display.getTextBounds(headingString, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor(280 - static_cast<int16_t>(textW), y);
  display.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  display.print(headingString);
}

// Compass Display
void drawCompassBanner(float direction) {
  const int16_t centerX = 140;
  const int16_t compassY = 200;
  const float pixelsPerDegree = 280.0f / 180.0f;
  const int16_t majorTickBottomY = compassY + 10;
  const int16_t minorTickBottomY = compassY + 5;
  const int16_t labelY = compassY + 15;
  for (int tickDegrees = 45; tickDegrees < 360; tickDegrees += 90) {
    float delta = static_cast<float>(tickDegrees) - direction;
    if (delta > 180.0f) {
      delta -= 360.0f;
    }
    if (delta < -180.0f) {
      delta += 360.0f;
    }
    if (delta < -90.0f || delta > 90.0f) {
      continue;
    }
    const int16_t tickX = centerX + static_cast<int16_t>(delta * pixelsPerDegree);
    if (tickX < 0 || tickX >= 280) {
      continue;
    }
    display.drawLine(tickX, compassY + 1, tickX, minorTickBottomY, ST77XX_WHITE);
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
    if (tickX < 0 || tickX >= 280) {
      continue;
    }
    int16_t x1 = 0;
    int16_t y1 = 0;
    uint16_t labelW = 0;
    uint16_t labelH = 0;
    display.setTextSize(2);
    display.getTextBounds(marker.label, 0, 0, &x1, &y1, &labelW, &labelH);
    display.drawLine(tickX, compassY + 1, tickX, majorTickBottomY, ST77XX_WHITE);
    display.setTextColor(ST77XX_WHITE);
    display.setCursor(tickX - static_cast<int16_t>(labelW / 2), labelY);
    display.print(marker.label);
  }
}

// Calibration Score Display
static void drawCalibrationScore() {
  display.fillScreen(ST77XX_BLACK);
  drawCentreText("Result", 10, 3, ST77XX_CYAN);
  if (!Last_Calibration_Quality.is_valid) {
    drawCentreText("N/A", 80, 4, ST77XX_RED);
    return;
  }
  // Score percentage colour
  const uint8_t score = static_cast<uint8_t>(Last_Calibration_Quality.score_percent + 0.5f);
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
  display.setTextSize(3);
  display.setTextColor(score_colour);
  display.setCursor(10, 50);
  display.print("Score: ");
  display.print(score);
  display.print("%");
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(10, 90);
  display.print("Count: ");
  display.print(Last_Calibration_Quality.used_sample_count);
  display.setTextSize(2);
  display.setCursor(10, 130);
  display.print("Span: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.used_sample_count));
  display.setCursor(10, 162);
  display.print("Oct: ");
  display.print(static_cast<int>(Last_Calibration_Quality.octant_coverage_score + 0.5f));
  display.print("%");
  display.setCursor(10, 194);
  display.print("PCA: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.unit_vector_pca_ratio_score + 0.5f));
  display.print("%");
  display.setCursor(10, 226);
  display.print("Residual: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.ellipsoid_residual_score + 0.5f));
  display.print("%");
  display.setCursor(10, 258);
  display.print("Rad Std: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.calibrated_radius_std_score + 0.5f));
  display.print("%");
}

// Bottom Timer Display Update
void updateDisplay(float Depth, int Minutes, int Seconds, const DecoResult &deco) {
  // Draw depth block on top-left
  uint8_t depthInteger = static_cast<uint8_t>(Depth);
  uint8_t depthDecimal = static_cast<uint8_t>(Depth * 10.0f + 0.5f) % 10;
  char integerPart[3];
  char decimalPart[2];
  const char *unitPart = "m";
  snprintf(integerPart, sizeof(integerPart), "%2u", depthInteger);
  snprintf(decimalPart, sizeof(decimalPart), "%u", depthDecimal);
  constexpr uint8_t integerSize = 12;
  constexpr uint8_t decimalSize = 6;
  constexpr uint8_t dotSize = 2;
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
  const int16_t depthX = 26;
  const int16_t depthY = 12;
  const int16_t depthBottom = depthY + static_cast<int16_t>(integerH);
  const int16_t dotX = depthX + static_cast<int16_t>(integerW) + 2;
  const int16_t dotY = depthBottom - static_cast<int16_t>(dotH) - 8;
  const int16_t decimalX = dotX + static_cast<int16_t>(dotW) + 2;
  const int16_t decimalY = depthBottom - static_cast<int16_t>(decimalH);
  const int16_t unitX = decimalX + static_cast<int16_t>(decimalW) + 2;
  const int16_t unitY = depthBottom - static_cast<int16_t>(unitH);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.setTextSize(integerSize);
  display.setCursor(depthX, depthY);
  display.print(integerPart);
  display.setTextSize(dotSize);
  display.setCursor(dotX, dotY);
  display.print(dotPart);
  display.setTextSize(decimalSize);
  display.setCursor(decimalX, decimalY);
  display.print(decimalPart);
  display.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  display.setTextSize(decimalSize);
  display.setCursor(unitX, unitY);
  display.print(unitPart);
  // Draw battery indicator on top right
  drawBatteryIndicator(Battery_Percentage);
  // Draw loop usage left of battery
  char loopUsageStr[9];
  snprintf(loopUsageStr, sizeof(loopUsageStr), "CPU:%3u%%", Loop_Usage_Percent);
  display.setTextSize(1);
  display.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  {
    int16_t ux1 = 0, uy1 = 0;
    uint16_t uW = 0, uH = 0;
    display.getTextBounds(loopUsageStr, 0, 0, &ux1, &uy1, &uW, &uH);
    display.setCursor(210 - static_cast<int16_t>(uW), 18);
    display.print(loopUsageStr);
  }
  // Current GF below battery
  char gfStr[6];
  if (ripNtear_Mode) {
    snprintf(gfStr, sizeof(gfStr), "99/99");
  } else {
    snprintf(gfStr, sizeof(gfStr), "60/85");
  }
  display.setTextSize(1);
  display.setTextColor(ripNtear_Mode ? ST77XX_RED : ST77XX_GREEN, ST77XX_BLACK);
  display.setCursor(221, 32);
  display.print(gfStr);
  // Draw timer below depth
  char timerString[8];
  snprintf(timerString, sizeof(timerString), "%3u:%02u", Minutes, Seconds);
  display.setTextSize(3);
  display.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  display.setCursor(0, 120);
  display.print(timerString);
  // Draw heading right of timer
  drawHeadingValue(120, Heading);
  // Draw Deco table below timer and heading
  display.drawFastHLine(0, 155, 280, ST77XX_WHITE);
  display.drawFastVLine(70, 156, 44, ST77XX_WHITE);
  display.drawFastVLine(140, 156, 44, ST77XX_WHITE);
  display.drawFastVLine(210, 156, 44, ST77XX_WHITE);
  constexpr int16_t leftColX = 1;
  constexpr int16_t rightColX = 141;
  constexpr int16_t cellW = 68;
  constexpr int16_t topTitleY = 128;
  display.setTextSize(2);
  display.setTextColor(ST77XX_CYAN);
  display.setCursor(0, 170);
  display.print("D");
  display.setCursor(74, 170);
  display.print("S");
  display.setTextSize(1);
  display.setCursor(144, 168);
  display.print("T");
  display.setCursor(144, 178);
  display.print("T");
  display.setCursor(144, 188);
  display.print("S");
  display.setCursor(214, 168);
  display.print("s");
  display.setCursor(214, 178);
  display.print("G");
  display.setCursor(214, 188);
  display.print("F");
  uint16_t surfGfColor = ST77XX_GREEN;
  if (deco.surfGF > 100) {
    surfGfColor = ST77XX_RED;
  } else if (deco.surfGF > 85) {
    surfGfColor = ST77XX_YELLOW;
  }
  char stopStr[4], timeStr[4], ttsStr[4], sGfStr[4];
  snprintf(stopStr, sizeof(stopStr), "%3u", deco.nextStopDepth);
  snprintf(timeStr, sizeof(timeStr), "%3u", deco.stopTime);
  snprintf(ttsStr, sizeof(ttsStr), "%3u", deco.timeToSurface);
  snprintf(sGfStr, sizeof(sGfStr), "%3u", deco.surfGF);
  drawColCentreText(stopStr, 20, 36, 170, 2, ST77XX_WHITE);
  drawColCentreText(timeStr, 94, 36, 170, 2, ST77XX_WHITE);
  drawColCentreText(ttsStr, 158, 36, 170, 2, ST77XX_WHITE);
  drawColCentreText(sGfStr, 228, 36, 170, 2, surfGfColor);
  // Draw compass banner at bottom
  display.fillRect(0, 201, 280, 39, ST77XX_BLACK);
  display.drawFastHLine(0, 200, 280, ST77XX_WHITE);
  drawCompassBanner(Heading);
  display.fillTriangle(136, 202, 144, 202, 140, 210, ST77XX_CYAN);
  display.drawLine(140, 210, 140, 240, ST77XX_CYAN);
}


void setup() {
  // Power conservation
  esp_wifi_stop();              // WiFi off
  esp_bt_controller_disable();  // Bluetooth off
  setCpuFrequencyMhz(80);       // Reduce CPU frequency

  // Display initialisation
  spi.begin(SCK_Pin, -1, MOSI_Pin, CS_Pin);
  pinMode(Backlight_Pin, OUTPUT);
  analogWrite(Backlight_Pin, Backlight_High);
  display.init(LCD_Width, LCD_Height, SPI_MODE3);
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);
  drawCentreText("Dive", 60, 6, ST77XX_WHITE);
  drawCentreText("Computer", 152, 6, ST77XX_WHITE);
  delay(Message_MS);
  display.fillScreen(ST77XX_BLACK);

  // ADC
  pinMode(Battery_Pin, INPUT);
  analogReadResolution(12);        // Internal ADC resolution 12-bit
  analogSetAttenuation(ADC_11db);  // 2.5V range

  // Power MS5837
  pinMode(Sensor_GND_Pin, OUTPUT);
  pinMode(Sensor_VCC_Pin, OUTPUT);
  digitalWrite(Sensor_GND_Pin, LOW);
  digitalWrite(Sensor_VCC_Pin, HIGH);
  delay(10);

  // MS5837 dedicated I2C initialisation
  sensorWire.begin(Sensor_SDA_Pin, Sensor_SCL_Pin);
  sensorWire.setClock(400000);
  delay(10);

  // I2C initialisation
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(400000);
  delay(10);

  // MS5837 initialisation
  // sensor.init(sensorWire);
  // sensor.setModel(MS5837::MS5837_30BA);  // 30 bar model
  // sensor.setFluidDensity(Density);       // Set water density

// --- TESTING --- //
  Pressure_Sensor_Available = isMs5837Present();
  if (Pressure_Sensor_Available) {
    Pressure_Sensor_Available = sensor.init(sensorWire);
    if (Pressure_Sensor_Available) {
      sensor.setModel(MS5837::MS5837_30BA);
      sensor.setFluidDensity(Density);
    }
  }
// --- TESTING --- //

  // BH1750 initialisation
  lightMeter.begin(BH1750::CONTINUOUS_LOW_RES_MODE, BH1750_I2C_Address, &Wire);  // 4 lux, 16 ms

  // QMI8658 initialisation
  qmi.begin(Wire, QMI8658_I2C_Address, SDA_Pin, SCL_Pin);
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G,
                          SensorQMI8658::ACC_ODR_125Hz,
                          SensorQMI8658::LPF_MODE_2);
  qmi.enableAccelerometer();
  qmi.disableGyroscope();

  // QMC5883P initialisation
  pinMode(Calibration_Button_Pin, INPUT_PULLUP);
  pinMode(DRDY_Pin, INPUT);
  qmc.begin(Wire, QMC5883P_I2C_Address, SDA_Pin, SCL_Pin);
  qmc.configMagnetometer(
      OperationMode::CONTINUOUS_MEASUREMENT,
      MagFullScaleRange::FS_2G,
      50.0f,
      MagOverSampleRatio::OSR_8,
      MagDownSampleRatio::DSR_8);
  compassConfigureReferenceField(Reference_Field_Gauss, Magnetometer_Lsb_Per_Gauss);
  if (!loadCompassCalibrationFromNVS()) {
    Compass_Matrices = {};
    compassSetCalibrationMatrices(nullptr);
  }

  // ZHL-16C initialisation
  decoSetup(GF_Low, GF_High, Setpoint);
  decoInit();
  Deco_Last_Update_MS = millis();
}


void loop() {
  const uint64_t loopStartMS = millis();

  // Rip & Tear Mode (raw Bühlmann ZHL-16C algorithm)
  if (detectTripleTap(loopStartMS)) {
    ripNtear_Mode = true;
    ripNtear(true);
    Deco_Last_Update_MS = loopStartMS - Deco_Update_MS;
  }

  // Compass calibration
  if (calibrationButtonPressed(loopStartMS) && !Calibration_Armed) {
    Calibration_Armed = true;
    Calibration_Start_MS = loopStartMS;
  }
  if (Calibration_Armed) {
    const uint64_t elapsedMs = loopStartMS - Calibration_Start_MS;
    // Start calibration
    if (elapsedMs < Calibration_Delay_MS) {
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
    delay(Message_MS);
    drawCalibrationScore();
    delay(Message_MS * 5);
    Calibration_Armed = false;
    Deco_Last_Update_MS = loopStartMS - Loop_MS;
    return;
  }

  // Depth
  // sensor.read();
  // const float pressureAtm = sensor.pressure() / MBAR_PER_ATM;
  // Depth = sensor.depth() - Depth_Offset;  // Sea level offset
  // if (Depth < 0.0f) Depth = 0.0f;         // Minimum depth 0 meter
  // if (Depth > 99.9f) Depth = 99.9f;       // Maximum depth 99.9 meters


// --- TESTING --- //
  float pressureMbar = MBAR_PER_ATM;
  readDepthAndPressure(Depth, pressureMbar);
  const float pressureAtm = pressureMbar / MBAR_PER_ATM;
// --- TESTING --- //


  // Timer
  updateTimer(Depth, loopStartMS);

  // Compass
  Heading = readCompassHeading();

  // Backlight
  Ambient_Lux = lightMeter.readLightLevel();

  // Battery
  Battery_Percentage = readBatteryPercentage();

  // ZHL-16C
  if ((loopStartMS - Deco_Last_Update_MS) >= Deco_Update_MS) {
    const float dtMin = static_cast<float>(loopStartMS - Deco_Last_Update_MS) / 60000.0f;
    Deco_Last_Update_MS = loopStartMS;
    decoUpdate(pressureAtm, dtMin);
    lastDecoResult = decoCompute(pressureAtm);
  }

  // Low Battery
  if (Battery_Percentage < Low_Battery_Threshold) {
    Backlight_Level = Backlight_Low;
    analogWrite(Backlight_Pin, Backlight_Level);
  } else {
    Backlight_Level = ambientBacklightLevel(Ambient_Lux);
    analogWrite(Backlight_Pin, Backlight_Level);
  }
  if (Battery_Percentage < Critical_Battery_Threshold) {
    display.fillScreen(ST77XX_BLACK);
    drawCentreText("Battery", 86, 4, ST77XX_RED);
    drawCentreText("Low", 144, 4, ST77XX_RED);
    delay(Message_MS);
    digitalWrite(Backlight_Pin, LOW);
    esp_deep_sleep_start();
  }

  // Display Update
  updateDisplay(Depth, Minutes, Seconds, lastDecoResult);

  // Loop Usage
  uint64_t elapsedMS = millis() - loopStartMS;
  if (Loop_MS > 0) {
    Loop_Usage_Percent = (elapsedMS * 100ULL + (Loop_MS / 2ULL)) / Loop_MS;
  }

  // Loop Interval
  if (elapsedMS < Loop_MS) {
    delay(Loop_MS - elapsedMS);
  }
}