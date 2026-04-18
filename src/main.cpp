#include <Arduino.h>
// Power Management
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_pm.h>
// Peripherals
#include <WebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <BH1750.h>
#include <MS5837.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SensorQMI8658.hpp>
#include <SensorQMC5883P.hpp>
// Data Processing
#include <Preferences.h>
#include <vector>
#include <ZHL16C.h>
#include <TJpg_Decoder.h>
#include <Adafruit_AHRS.h>
#include <magnetometer_calibration.h>
// Header Files
#include <pgmspace.h>
#include "demo.h"
#include "image.h"
#include "compass_label.h"

// Pins
constexpr gpio_num_t RST_Pin = GPIO_NUM_0;  // Boot button
constexpr gpio_num_t Backlight_Pin = GPIO_NUM_1;
constexpr gpio_num_t IMU_INT_Pin = GPIO_NUM_3;
constexpr gpio_num_t Battery_Pin = GPIO_NUM_5;
constexpr gpio_num_t Button_Pin = GPIO_NUM_9;
constexpr gpio_num_t Sensor_SDA_Pin = GPIO_NUM_10;
constexpr gpio_num_t Sensor_VCC_Pin = GPIO_NUM_19;
constexpr gpio_num_t Sensor_SCL_Pin = GPIO_NUM_20;
constexpr gpio_num_t MOSI_Pin = GPIO_NUM_38;
constexpr gpio_num_t SCK_Pin = GPIO_NUM_39;
constexpr gpio_num_t MISO_Pin = GPIO_NUM_40;
constexpr gpio_num_t SD_CS_Pin = GPIO_NUM_41;
constexpr gpio_num_t DC_Pin = GPIO_NUM_42;
constexpr gpio_num_t U0TXD_Pin = GPIO_NUM_43;
constexpr gpio_num_t U0RXD_Pin = GPIO_NUM_44;
constexpr gpio_num_t LCD_CS_Pin = GPIO_NUM_45;
constexpr gpio_num_t SCL_Pin = GPIO_NUM_47;
constexpr gpio_num_t SDA_Pin = GPIO_NUM_48;

// Objects
SPIClass spi(FSPI);
Adafruit_ST7789 display(&spi, LCD_CS_Pin, DC_Pin, RST_Pin);
Preferences prefs;
TwoWire sensorWire(1);
MS5837 sensor;
BH1750 lightMeter;
SensorQMI8658 qmi;
SensorQMC5883P qmc;
Adafruit_Madgwick ahrs;

// I2C Addresses
constexpr uint8_t QMI8658_I2C_Address = 0x6B;
constexpr uint8_t QMC5883P_I2C_Address = 0x2C;
constexpr uint8_t BH1750_I2C_Address = 0x23;
constexpr uint8_t MS5837_I2C_Address = 0x76;

// LCD Constants
constexpr uint16_t LCD_Width = 320;
constexpr uint16_t LCD_Height = 240;
constexpr uint8_t Low_Battery_Threshold = 10;      // Dim screen at 10%
constexpr uint8_t Critical_Battery_Threshold = 0;  // Auto shut down at 0%
constexpr float Battery_Divider_Ratio = 3.0f;      // 2:1 voltage divider
constexpr float Ambient_Lux_Max = 40000.0f;        // Lux at high backlight
constexpr uint8_t Backlight_Low = 8;               // Low backlight
constexpr uint8_t Backlight_High = 255;            // High backlight
constexpr uint32_t Message_MS = 3000;              // Display messages for 3 seconds
constexpr uint32_t Display_Update_MS = 50;         // Display refresh rate: 20 Hz
constexpr uint32_t Button_Update_MS = 10;          // Button checks and compass: 100 Hz
constexpr uint32_t Depth_Update_MS = 500;          // Depth sensor and ADC updates: 2 Hz
constexpr uint32_t Deco_Update_MS = 5000;          // Decompression model updates: 0.2 Hz

// Button Constants
constexpr uint32_t Button_Debounce_MS = 50;       // Button debounce time
constexpr uint32_t Button_Short_MS = 799;         // Short press threshold
constexpr uint32_t Button_Long_MS = 800;          // Long press threshold
constexpr uint32_t Button_Hold_MS = 5000;         // Press and hold threshold
constexpr uint32_t Calibration_Delay_MS = 10000;  // Delay before calibration

// Depth Sensor Constants
constexpr uint16_t Density = 1020;        // EN13319 density
constexpr float MBAR_PER_ATM = 1013.25f;  // mBar per atm
constexpr float Depth_Offset = 0.2f;      // Sea level offset 0.2 m
constexpr float Dive_Start_Depth = 1.0f;  // Start timer at 1 m

// ZHL-16C Constants
constexpr uint8_t GF_Low = 60;    // Gradient factor low
constexpr uint8_t GF_High = 85;   // Gradient factor high
constexpr float Setpoint = 1.2f;  // CCR setpoint

// IMU Offset Constants
constexpr float Accel_Offset_X = 0.0;
constexpr float Accel_Offset_Y = 0.0f;
constexpr float Accel_Offset_Z = 0.0f;
constexpr float Gyro_Offset_X = 0.4791f;
constexpr float Gyro_Offset_Y = 5.7993f;
constexpr float Gyro_Offset_Z = -0.7015f;

// Compass Constants
constexpr float Compass_Offset = 0.0;                        // Compass offset degrees
constexpr float Reference_Field_Gauss = 0.49f;               // UK average magnetic field strength
constexpr float Magnetometer_Lsb_Per_Gauss = 3750.0f;        // QMC5883P at FS_8G: 3750 LSB/Gauss
constexpr uint32_t Compass_Calibration_Duration_MS = 30000;  // 30 seconds for compass calibration
constexpr const char *Compass_NVS_Namespace = "compass";
constexpr const char *Calibration_Valid_Key = "cal_valid";
constexpr const char *Hard_Iron_X_Key = "hi_x";
constexpr const char *Hard_Iron_Y_Key = "hi_y";
constexpr const char *Hard_Iron_Z_Key = "hi_z";
constexpr const char *Soft_Iron_Key = "soft_iron";
constexpr const char *Reference_Mag_Gauss_Key = "ref_gauss";
constexpr const char *Lsb_Per_Gauss_Key = "lsb_gauss";
constexpr const char *Fitted_Field_Lsb_Key = "fit_lsb";
constexpr const char *Settings_NVS_Namespace = "settings";
constexpr const char *Demo_Mode_Key = "demo_mode";

// Default Compass Calibration Matrix
constexpr CompassCalibrationMatrices Default_Compass_Matrices = {
  .hard_iron = {431.913489, 296.874488, 374.476474},
  .soft_iron = {
    0.000330f, 0.000001f, -0.000020f,
    0.000001f, 0.000308f, 0.000008f,
    -0.000020f, 0.000008f, 0.000300f
  },
  .reference_field_gauss = Reference_Field_Gauss,
  .lsb_per_gauss = Magnetometer_Lsb_Per_Gauss,
  .fitted_field_lsb = Reference_Field_Gauss * Magnetometer_Lsb_Per_Gauss,
  .is_valid = true
};

// Battery Percentage Lookup Table
constexpr float Voltage_Table[] = {
    3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f, 3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
constexpr uint8_t Percentage_Table[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
constexpr size_t Battery_Table_Size = sizeof(Voltage_Table) / sizeof(Voltage_Table[0]);

// SPIFFS JPEG path
constexpr const char *Spiffs_Jpeg_Path = "/image.jpg";

// WiFi AP credentials
constexpr const char *SSID = "DiveComputer";
constexpr const char *Password = "12345678";

// RTOS Variables
TaskHandle_t Display_Task_Handle = nullptr;
TaskHandle_t Sensor_Task_Handle = nullptr;
GFXcanvas16 *Frame_Canvas = nullptr;
uint16_t *Frame_Back_Previous = nullptr;
uint16_t *Frame_Back_Current = nullptr;
size_t Frame_Buffer_Bytes = 0;
uint16_t Render_Width = 0;
uint16_t Render_Height = 0;
bool Frame_Have_Previous = false;
bool Spiffs_Jpeg_Ready = false;

// Modes
bool Demo_Mode = false;
bool ripNtear_Mode = false;

// Dive Variables
float Heading = 0.0f;
float Depth = 0.0f;
int Minutes = 0;
int Seconds = 0;
int Stopwatch_Minutes = 0;
int Stopwatch_Seconds = 0;
uint64_t Timer_Start_MS = 0;
uint64_t Stopwatch_Start_MS = 0;
uint64_t Simulated_Profile_Start_MS = 0;
bool Dive_Timer_Started = false;
bool Stopwatch_Active = false;
bool sensorAvailable = false;

// Battery Variables
float Ambient_Lux = 0.0f;
float Battery_Voltage = 0.0f;
uint8_t Battery_Percentage = 0;
uint8_t Backlight_Level = Backlight_High;

// Press Button Variables
uint8_t Button_Last_Reading = HIGH;
uint8_t Button_Stable_State = HIGH;
uint64_t Button_Last_Change_MS = 0;
uint64_t Button_Press_Start_MS = 0;
bool Button_Long_Event_Fired = false;
bool Button_Hold_Event_Fired = false;

// Boot Button Variables
uint8_t Boot_Last_Reading = HIGH;
uint8_t Boot_Stable_State = HIGH;
uint64_t Boot_Last_Change_MS = 0;
uint64_t Boot_Press_Start_MS = 0;
bool Boot_Long_Event_Fired = false;
bool Boot_Hold_Event_Fired = false;

// Decompression Variables
DecoResult lastDecoResult = {false, 0, 0, 0, 0};
uint64_t Deco_Last_Update_MS = 0;

// Compass Labels
float projectCompassX3D(float x, float centerX, float width) {
  constexpr float Projection_Exponent = 2.1f;
  if (x <= centerX) {
    const float t = constrain(x / centerX, 0.0f, 1.0f);
    return powf(t, Projection_Exponent) * centerX;
  }
  const float rightSpan = width - centerX;
  const float t = constrain((width - x) / rightSpan, 0.0f, 1.0f);
  return (centerX - (powf(t, Projection_Exponent) * centerX)) + centerX;
}
float smoothstep01(float t) {
  t = constrain(t, 0.0f, 1.0f);
  return t * t * (3.0f - (2.0f * t));
}
float compassCenterWeight(float projectedX, float centerX) {
  const float maxDist = centerX;
  const float dist = fabsf(projectedX - centerX);
  const float linearWeight = 1.0f - constrain(dist / maxDist, 0.0f, 1.0f);
  return smoothstep01(linearWeight);
}
void drawScaledXBitmap(Adafruit_GFX &target, int16_t x, int16_t y, uint8_t srcW, uint8_t srcH, const uint8_t *bitmap, float scale, uint16_t color) {
  if (bitmap == nullptr || scale <= 0.0f) {
    return;
  }
  const int16_t dstW = static_cast<int16_t>(srcW * scale + 0.5f);
  const int16_t dstH = static_cast<int16_t>(srcH * scale + 0.5f);
  if (dstW <= 0 || dstH <= 0) {
    return;
  }
  const uint8_t bytesPerRow = static_cast<uint8_t>((srcW + 7) / 8);
  for (int16_t dy = 0; dy < dstH; ++dy) {
    const uint8_t sy = static_cast<uint8_t>((static_cast<int32_t>(dy) * srcH) / dstH);
    for (int16_t dx = 0; dx < dstW; ++dx) {
      const uint8_t sx = static_cast<uint8_t>((static_cast<int32_t>(dx) * srcW) / dstW);
      const uint16_t byteIndex = static_cast<uint16_t>(sy * bytesPerRow + (sx / 8));
      const uint8_t rowByte = pgm_read_byte(bitmap + byteIndex);
      const bool isSet = (rowByte & (1U << (sx & 7))) != 0;
      if (isSet) {
        target.drawPixel(x + dx, y + dy, color);
      }
    }
  }
}

// Compass Calibration Method and Quality
CompassCalibrationMatrices Compass_Matrices = {};
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
enum class CalibrationFailReason : uint8_t {
  None = 0,
  TooFewSamples,
  FitFailed,
  ApplyFailed,
  InvalidQuality,
};
CalibrationFailReason Last_Calibration_Fail_Reason = CalibrationFailReason::None;
size_t Last_Calibration_Sample_Count = 0;
uint32_t Last_Calibration_Elapsed_MS = 0;
static const char* calibrationFailReasonText(CalibrationFailReason reason) {
  switch (reason) {
    case CalibrationFailReason::TooFewSamples:
      return "Too few samples";
    case CalibrationFailReason::FitFailed:
      return "Fit failed";
    case CalibrationFailReason::ApplyFailed:
      return "Apply failed";
    case CalibrationFailReason::InvalidQuality:
      return "Invalid quality";
    case CalibrationFailReason::None:
    default:
      return "None";
  }
}

// Press Button State
enum class PressButtonEvent : uint8_t {
  None = 0,
  ShortPress,
  LongPress,
  HoldPress,
};
PressButtonEvent checkButton(uint64_t nowMS) {
  const uint8_t reading = static_cast<uint8_t>(digitalRead(Button_Pin));
  if (reading != Button_Last_Reading) {
    Button_Last_Change_MS = nowMS;
    Button_Last_Reading = reading;
  }
  PressButtonEvent event = PressButtonEvent::None;
  if ((nowMS - Button_Last_Change_MS) >= Button_Debounce_MS) {
    if (reading != Button_Stable_State) {
      Button_Stable_State = reading;
      if (Button_Stable_State == LOW) {
        Button_Press_Start_MS = nowMS;
        Button_Long_Event_Fired = false;
        Button_Hold_Event_Fired = false;
      } else {
        const uint64_t heldMS = nowMS - Button_Press_Start_MS;
        if (!Button_Long_Event_Fired && heldMS <= Button_Short_MS) {
          event = PressButtonEvent::ShortPress;
        } else if (!Button_Hold_Event_Fired && heldMS >= Button_Long_MS) {
          Button_Long_Event_Fired = true;
          event = PressButtonEvent::LongPress;
        }
      }
    }
    if (Button_Stable_State == LOW) {
      const uint64_t heldMS = nowMS - Button_Press_Start_MS;
      if (!Button_Hold_Event_Fired && heldMS >= Button_Hold_MS) {
        Button_Hold_Event_Fired = true;
        event = PressButtonEvent::HoldPress;
      }
    }
  }
  return event;
}

// Boot Button State
enum class BootButtonEvent : uint8_t {
  None = 0,
  ShortPress,
  LongPress,
  HoldPress,
};
BootButtonEvent checkBootButton(uint64_t nowMS) {
  const uint8_t reading = static_cast<uint8_t>(digitalRead(RST_Pin));
  if (reading != Boot_Last_Reading) {
    Boot_Last_Change_MS = nowMS;
    Boot_Last_Reading = reading;
  }
  BootButtonEvent event = BootButtonEvent::None;
  if ((nowMS - Boot_Last_Change_MS) >= Button_Debounce_MS) {
    if (reading != Boot_Stable_State) {
      Boot_Stable_State = reading;
      if (Boot_Stable_State == LOW) {
        Boot_Press_Start_MS = nowMS;
        Boot_Long_Event_Fired = false;
        Boot_Hold_Event_Fired = false;
      } else {
        const uint64_t heldMS = nowMS - Boot_Press_Start_MS;
        if (!Boot_Long_Event_Fired && heldMS <= Button_Short_MS) {
          event = BootButtonEvent::ShortPress;
        } else if (!Boot_Long_Event_Fired && heldMS >= Button_Long_MS) {
          Boot_Long_Event_Fired = true;
          event = BootButtonEvent::LongPress;
        }
      }
    }
    if (Boot_Stable_State == LOW) {
      const uint64_t heldMS = nowMS - Boot_Press_Start_MS;
      if (!Boot_Hold_Event_Fired && heldMS >= Button_Hold_MS) {
        Boot_Hold_Event_Fired = true;
        event = BootButtonEvent::HoldPress;
      }
    }
  }
  return event;
}

// Frame Buffer Management
bool initFrameBuffers() {
  Render_Width = static_cast<uint16_t>(display.width());
  Render_Height = static_cast<uint16_t>(display.height());
  Frame_Buffer_Bytes = static_cast<size_t>(Render_Width) * static_cast<size_t>(Render_Height) * sizeof(uint16_t);
  Frame_Canvas = new GFXcanvas16(Render_Width, Render_Height);
  if (Frame_Canvas == nullptr || Frame_Canvas->getBuffer() == nullptr) {
    return false;
  }
  Frame_Back_Previous = static_cast<uint16_t *>(heap_caps_malloc(Frame_Buffer_Bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  Frame_Back_Current = static_cast<uint16_t *>(heap_caps_malloc(Frame_Buffer_Bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (Frame_Back_Previous == nullptr || Frame_Back_Current == nullptr) {
    return false;
  }
  memset(Frame_Back_Previous, 0, Frame_Buffer_Bytes);
  memset(Frame_Back_Current, 0, Frame_Buffer_Bytes);
  Frame_Have_Previous = false;
  return true;
}

// Flush from PSRAM
void flushDirtyRectFromPSRAM() {
  if (Frame_Back_Previous == nullptr || Frame_Back_Current == nullptr || Render_Width == 0 || Render_Height == 0) {
    return;
  }
  uint16_t dirtyX0 = Render_Width;
  uint16_t dirtyY0 = Render_Height;
  uint16_t dirtyX1 = 0;
  uint16_t dirtyY1 = 0;
  bool hasDirty = !Frame_Have_Previous;
  if (!hasDirty) {
    for (uint16_t y = 0; y < Render_Height; ++y) {
      const size_t rowOffset = static_cast<size_t>(y) * static_cast<size_t>(Render_Width);
      int16_t left = -1;
      int16_t right = -1;
      for (uint16_t x = 0; x < Render_Width; ++x) {
        const size_t idx = rowOffset + static_cast<size_t>(x);
        if (Frame_Back_Current[idx] != Frame_Back_Previous[idx]) {
          if (left < 0) {
            left = static_cast<int16_t>(x);
          }
          right = static_cast<int16_t>(x);
        }
      }
      if (left >= 0) {
        hasDirty = true;
        if (y < dirtyY0) dirtyY0 = y;
        if (y > dirtyY1) dirtyY1 = y;
        if (static_cast<uint16_t>(left) < dirtyX0) dirtyX0 = static_cast<uint16_t>(left);
        if (static_cast<uint16_t>(right) > dirtyX1) dirtyX1 = static_cast<uint16_t>(right);
      }
    }
  }
  if (hasDirty) {
    if (!Frame_Have_Previous) {
      dirtyX0 = 0;
      dirtyY0 = 0;
      dirtyX1 = static_cast<uint16_t>(Render_Width - 1);
      dirtyY1 = static_cast<uint16_t>(Render_Height - 1);
    }
    const uint16_t dirtyW = static_cast<uint16_t>(dirtyX1 - dirtyX0 + 1);
    const uint16_t dirtyH = static_cast<uint16_t>(dirtyY1 - dirtyY0 + 1);
    display.startWrite();
    display.setAddrWindow(dirtyX0, dirtyY0, dirtyW, dirtyH);
    for (uint16_t row = 0; row < dirtyH; ++row) {
      const uint16_t srcY = static_cast<uint16_t>(dirtyY0 + row);
      uint16_t *rowPtr = &Frame_Back_Current[(static_cast<size_t>(srcY) * Render_Width) + dirtyX0];
      display.writePixels(rowPtr, dirtyW, true, false);
    }
    display.endWrite();
  }
  uint16_t *tmp = Frame_Back_Previous;
  Frame_Back_Previous = Frame_Back_Current;
  Frame_Back_Current = tmp;
  Frame_Have_Previous = true;
}

// Load Compass Calibration Values
bool loadCompassCalibrationFromNVS() {
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

// Save Compass Calibration Values
void saveCompassCalibrationToNVS(const CompassCalibrationMatrices& matrices) {
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

// Load Demo Mode State
void loadDemoMode() {
  if (prefs.begin(Settings_NVS_Namespace, true)) {
    Demo_Mode = prefs.getBool(Demo_Mode_Key, false);
    prefs.end();
  }
}

// Save Demo Mode State
void saveDemoMode() {
  prefs.begin(Settings_NVS_Namespace, false);
  prefs.putBool(Demo_Mode_Key, Demo_Mode);
  prefs.end();
}

// Display Centre Text 
void drawCentreText(const char *text, int16_t y, uint8_t font, uint16_t colour) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  display.setTextSize(font);
  display.setTextWrap(false);
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = ((display.width() - static_cast<int16_t>(w)) / 2) - x1;
  display.setCursor(x, y);
  display.setTextColor(colour);
  display.print(text);
}

// Display Column-Centred Text
void drawColCentreText(Adafruit_GFX &target, const char *text, int16_t colX, int16_t colW, int16_t y, uint8_t font, uint16_t colour) {
  int16_t x1 = 0, y1 = 0;
  uint16_t w = 0, h = 0;
  target.setTextSize(font);
  target.setTextWrap(false);
  target.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = colX + ((colW - static_cast<int16_t>(w)) / 2) - x1;
  target.setCursor(x, y);
  target.setTextColor(colour, ST77XX_BLACK);
  target.print(text);
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

// Read Gyroscope and Map to Body Frame
void readGyroTransformed(float out_gyro[3]) {
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  qmi.getGyroscope(gx, gy, gz);
  gx -= Gyro_Offset_X;
  gy -= Gyro_Offset_Y;
  gz -= Gyro_Offset_Z;
  out_gyro[0] = gx;   // Right = QMI_X
  out_gyro[1] = -gy;  // Down = -QMI_Y
  out_gyro[2] = -gz;  // Forward = -QMI_Z
}

// Read Accelerometer and Map to Body Frame
void readAccelTransformed(float out_accel[3]) {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  qmi.getAccelerometer(ax, ay, az);
  ax -= Accel_Offset_X;
  ay -= Accel_Offset_Y;
  az -= Accel_Offset_Z;
  out_accel[0] = ax;   // Right = QMI_X
  out_accel[1] = -ay;  // Down = -QMI_Y
  out_accel[2] = -az;  // Forward = QMI_Z
}

// Read Magnetometer and Map to Body Frame
void readMagTransformed(float out_mag[3]) {
  MagnetometerData mag_data = {};
  qmc.readData(mag_data);
  float mx = static_cast<float>(mag_data.raw.x);
  float my = static_cast<float>(mag_data.raw.y);
  float mz = static_cast<float>(mag_data.raw.z);
  out_mag[0] = -mx;  // Right = -QMC_X
  out_mag[1] = -my;  // Down = -QMC_Y
  out_mag[2] = -mz;  // Forward = -QMC_Z
}

// Read Compass Heading
float readCompassHeading() {
  float accel_body[3] = {0.0f, 0.0f, 0.0f};
  readAccelTransformed(accel_body);  // Calibrated accelerometer data
  float gyro_body[3]  = {0.0f, 0.0f, 0.0f};
  readGyroTransformed(gyro_body);  // Calibrated gyroscope data
  float raw_mag_body[3] = {0.0f, 0.0f, 0.0f};
  float cal_mag_body[3] = {0.0f, 0.0f, 0.0f};
  readMagTransformed(raw_mag_body);
  compassApplyCalibration(raw_mag_body, cal_mag_body);  // Calibrated magnetometer data
  const float ax = accel_body[2];
  const float ay = accel_body[0];
  const float az = accel_body[1];
  const float gx = gyro_body[2];
  const float gy = gyro_body[0];
  const float gz = gyro_body[1];
  const float mx = cal_mag_body[2];
  const float my = cal_mag_body[0];
  const float mz = cal_mag_body[1];
  ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz);  // Map body frame to AHRS frame
  float heading = ahrs.getYaw();  // Yaw is heading down
  if (heading < 0.0f) heading += 360.0f;
  heading = 180.0f - heading - Compass_Offset;  // E/W flip and compass offset
  if (heading < 0.0f) heading += 360.0f;
  return heading;
}

// Compass Calibration - Data Collection
bool collectCompassSamples(std::vector<float>& mag_samples_xyz) {
  constexpr uint32_t Calibration_Sample_Period_MS = 10;  // ODR 100 Hz
  constexpr size_t Calibration_Max_Samples = 9999U;      // Maximum 9999 samples
  Last_Calibration_Sample_Count = 0;
  Last_Calibration_Elapsed_MS = 0;
  mag_samples_xyz.reserve(3U * Calibration_Max_Samples);
  const uint32_t start_ms = millis();
  uint32_t last_sample_ms = start_ms;
  while ((millis() - start_ms) < Compass_Calibration_Duration_MS) {
    const uint32_t now_ms = millis();
    if ((now_ms - last_sample_ms) >= Calibration_Sample_Period_MS) {
      if ((mag_samples_xyz.size() / 3U) >= Calibration_Max_Samples) {
        break;
      }
      float raw_mag[3] = {0.0f, 0.0f, 0.0f};
      readMagTransformed(raw_mag);
      mag_samples_xyz.push_back(raw_mag[0]);
      mag_samples_xyz.push_back(raw_mag[1]);
      mag_samples_xyz.push_back(raw_mag[2]);
      last_sample_ms = now_ms;
    }
  }
  Last_Calibration_Elapsed_MS = millis() - start_ms;
  Last_Calibration_Sample_Count = mag_samples_xyz.size() / 3U;
  return true;
}

// Compass Calibration - Computation
bool computeCompassCalibration(const std::vector<float>& mag_samples_xyz) {
  const size_t sample_count = mag_samples_xyz.size() / 3U;
  Last_Calibration_Fail_Reason = CalibrationFailReason::None;
  if (sample_count < 50U) {
    Last_Calibration_Method = CompassCalibrationMethod::Error;
    Last_Calibration_Fail_Reason = CalibrationFailReason::TooFewSamples;
    return false;
  }
  CompassCalibrationMatrices fitted = {};
  CompassCalibrationFitMethod fit_method = CompassCalibrationFitMethod::Error;
  if (!compassCalibrateFromSamplesWithFallback(mag_samples_xyz.data(), sample_count, &fitted, &fit_method)) {
    Last_Calibration_Method = CompassCalibrationMethod::Error;
    Last_Calibration_Fail_Reason = CalibrationFailReason::FitFailed;
    return false;
  }
  if (fit_method == CompassCalibrationFitMethod::Ellipsoid) {
    Compass_Matrices = fitted;
  } else if (fit_method == CompassCalibrationFitMethod::MinMax) {
    Compass_Matrices.hard_iron[0] = fitted.hard_iron[0];
    Compass_Matrices.hard_iron[1] = fitted.hard_iron[1];
    Compass_Matrices.hard_iron[2] = fitted.hard_iron[2];
    for (int i = 0; i < 9; i++) {
      Compass_Matrices.soft_iron[i] = Default_Compass_Matrices.soft_iron[i];
    }
  Compass_Matrices.reference_field_gauss = fitted.reference_field_gauss;
  Compass_Matrices.lsb_per_gauss = fitted.lsb_per_gauss;
  Compass_Matrices.fitted_field_lsb = fitted.fitted_field_lsb;
  Compass_Matrices.is_valid = fitted.is_valid;
  }
  if (!compassSetCalibrationMatrices(&Compass_Matrices)) {
    Last_Calibration_Method = CompassCalibrationMethod::Error;
    Last_Calibration_Fail_Reason = CalibrationFailReason::ApplyFailed;
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
  if (!Last_Calibration_Quality.is_valid) {
    Last_Calibration_Fail_Reason = CalibrationFailReason::InvalidQuality;
  }
  return true;
}

// Compass Calibration - Diagnostics Display
void drawCalibrationDiagnostics(bool calibration_ok) {
  display.fillScreen(ST77XX_BLACK);
  float sample_rate_hz = 0.0f;
  if (Last_Calibration_Elapsed_MS > 0) {
    sample_rate_hz = (1000.0f * static_cast<float>(Last_Calibration_Sample_Count)) / static_cast<float>(Last_Calibration_Elapsed_MS);
  }
  display.setTextColor(ST77XX_WHITE);
  uint16_t state_colour;
  if (calibration_ok) {
    state_colour = ST77XX_GREEN;
  } else {
    state_colour = ST77XX_RED;
  }
  display.setTextSize(3);
  display.setTextColor(state_colour);
  display.setCursor(20, 20);
  display.print(calibration_ok ? "Success" : "Failed");
  display.setTextSize(2);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(20, 60);
  display.print("Method: ");
  display.print(compassCalibrationMethodText(Last_Calibration_Method));
  display.setCursor(20, 90);
  display.print("Reason: ");
  display.print(calibrationFailReasonText(Last_Calibration_Fail_Reason));
  display.setCursor(20, 120);
  display.print("Samples: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Sample_Count));
  display.setCursor(20, 150);
  display.print("Rate Hz: ");
  display.print(sample_rate_hz, 1);
  display.setCursor(20, 180);
  display.print("Time ms: ");
  display.print(static_cast<unsigned long>(Last_Calibration_Elapsed_MS));
  display.setCursor(20, 210);
  display.print("Quality: ");
  display.print(Last_Calibration_Quality.is_valid ? "Valid" : "Invalid");
}

// Calibration Score Display
void drawCalibrationScore() {
  display.fillScreen(ST77XX_BLACK);
  if (!Last_Calibration_Quality.is_valid) {
    drawCentreText("N/A", 20, 3, ST77XX_RED);
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
  display.setCursor(20, 20);
  display.print("Score: ");
  display.print(score);
  display.print("%");
  display.setTextSize(2);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(20, 70);
  display.print("Count: ");
  display.print(Last_Calibration_Quality.used_sample_count);
  display.setCursor(20, 100);
  display.print("Span: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.used_sample_count));
  display.setCursor(20, 130);
  display.print("Oct: ");
  display.print(static_cast<int>(Last_Calibration_Quality.octant_coverage_score + 0.5f));
  display.print("%");
  display.setCursor(20, 160);
  display.print("PCA: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.unit_vector_pca_ratio_score + 0.5f));
  display.print("%");
  display.setCursor(20, 190);
  display.print("Rad Std: ");
  display.print(static_cast<unsigned int>(Last_Calibration_Quality.calibrated_radius_std_score + 0.5f));
  display.print("%");
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

// Stopwatch
void updateStopwatch(uint64_t nowMS) {
  if (!Stopwatch_Active) {
    return;
  }
  const uint64_t elapsedMS = nowMS - Stopwatch_Start_MS;
  Stopwatch_Minutes = static_cast<int>(elapsedMS / 60000ULL);
  if (Stopwatch_Minutes > 999) Stopwatch_Minutes = 999;
  Stopwatch_Seconds = static_cast<int>((elapsedMS % 60000ULL) / 1000ULL);
  if (Stopwatch_Seconds > 59) Stopwatch_Seconds = 59;
}
void startStopwatch(uint64_t nowMS) {
  Stopwatch_Active = true;
  Stopwatch_Start_MS = nowMS;
  Stopwatch_Minutes = 0;
  Stopwatch_Seconds = 0;
}
void stopAndResetStopwatch() {
  Stopwatch_Active = false;
  Stopwatch_Start_MS = 0;
  Stopwatch_Minutes = 0;
  Stopwatch_Seconds = 0;
}

// Battery Percentage
uint8_t readBatteryPercentage() {
  constexpr uint8_t sampleCount = 16;  // Oversampling
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
      return ((voltage - lower) < (upper - voltage)) ? Percentage_Table[i] : Percentage_Table[i + 1];
    }
  }
  return 0;
}

// Battery Indicator
void drawBatteryIndicator(Adafruit_GFX &target, uint8_t percentage) {
  uint16_t fillColor = ST77XX_GREEN;
  if (percentage < 10) {
    fillColor = ST77XX_RED;
  } else if (percentage < 25) {
    fillColor = ST77XX_YELLOW;
  }
  constexpr int16_t outlineW = 40;
  constexpr int16_t outlineH = 14;
  constexpr int16_t padding = 3;
  const int16_t outlineX = 276;  // Top-right corner
  const int16_t outlineY = 0;
  const int16_t fillWidth = map(percentage, 0, 100, 0, outlineW - (padding * 2));
  target.fillRect(outlineX + padding, outlineY + padding, outlineW - (padding * 2), outlineH - (padding * 2), ST77XX_BLACK);
  target.drawRect(outlineX, outlineY, outlineW, outlineH, ST77XX_WHITE);
  target.fillRect(outlineX + outlineW, outlineY + 5, 4, outlineH - 10, ST77XX_WHITE);
  target.fillRect(outlineX + padding, outlineY + padding, fillWidth, outlineH - (padding * 2), fillColor);
}

// Heading Block
void drawHeadingValue(Adafruit_GFX &target, int16_t x, int16_t y, int8_t font, float direction) {
  int heading = static_cast<int>(direction + 0.5f);
  if (heading == 360) {
    heading = 0;
  }
  char headingString[4];
  snprintf(headingString, sizeof(headingString), "%03u", heading);
  target.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  target.setTextSize(font);
  target.setCursor(x, y);
  target.print(headingString);
  target.setTextSize(2);
  target.cp437(true);
  target.write(248);  // Degree symbol
}

// Compass Block
void drawCompassBanner(Adafruit_GFX &target, float direction) {
  const int16_t bannerWidth = static_cast<int16_t>(target.width());
  const int16_t centerX = bannerWidth / 2;
  const int16_t compassY = 202;
  const int16_t labelBaseY = 216;
  const float majorLabelScale = 2.0f;
  const float minorLabelScale = majorLabelScale;
  target.setTextWrap(false);
  target.fillRect(0, compassY + 12, bannerWidth, 37, ST77XX_BLACK);  // Tick and label area
  target.drawFastHLine(0, 200, bannerWidth, ST77XX_WHITE);  // Thick upper line
  target.drawFastHLine(0, 201, bannerWidth, ST77XX_WHITE);  // Thick upper line
  target.fillTriangle(centerX - 4, 203, centerX + 4, 203, centerX, 212, ST77XX_CYAN);  // Heading indicator arrow
  target.drawLine(centerX, 212, centerX, 239, ST77XX_CYAN);  // Heading indicator line
  const float pixelsPerDegree = static_cast<float>(bannerWidth) / 180.0f;
  const int16_t majorTickBottomY = compassY + 10;  // Major tick 10 pixels high
  const int16_t minorTickBottomY = compassY + 6;   // Minor tick 6 pixels high
  const int16_t microTickBottomY = compassY + 4;   // Micro tick 4 pixels high
  for (uint8_t step = 1; step < 16; step += 2) {
    float delta = (22.5f * static_cast<float>(step)) - direction;
    if (delta > 180.0f) {
      delta -= 360.0f;
    }
    if (delta < -180.0f) {
      delta += 360.0f;
    }
    if (delta < -90.0f || delta > 90.0f) {
      continue;
    }
    const float linearX = static_cast<float>(centerX) + (delta * pixelsPerDegree);
    const float projectedX = projectCompassX3D(linearX, static_cast<float>(centerX), static_cast<float>(bannerWidth));
    const int16_t tickX = static_cast<int16_t>(projectedX + 0.5f);
    if (tickX < 0 || tickX >= bannerWidth) {
      continue;
    }
    target.drawLine(tickX, compassY + 1, tickX, microTickBottomY, ST77XX_WHITE);  // Projected micro ticks
  }
  for (const CompassLabelBitmapSet &marker : Compass_Label_Bitmaps) {
    float delta = static_cast<float>(marker.degrees) - direction;
    if (delta > 180.0f) {
      delta -= 360.0f;
    }
    if (delta < -180.0f) {
      delta += 360.0f;
    }
    if (delta < -90.0f || delta > 90.0f) {
      continue;
    }
    const float linearX = static_cast<float>(centerX) + (delta * pixelsPerDegree);
    const float projectedX = projectCompassX3D(linearX, static_cast<float>(centerX), static_cast<float>(bannerWidth));
    const int16_t tickX = static_cast<int16_t>(projectedX + 0.5f);
    if (tickX < 0 || tickX >= bannerWidth) {
      continue;
    }
    const bool majorTick = (marker.degrees % 90) == 0;
    target.drawLine(tickX, compassY + 1, tickX, majorTick ? majorTickBottomY : minorTickBottomY, ST77XX_WHITE);  // Projected micro ticks
    if (majorTick) {
      target.drawLine(tickX - 1, compassY + 1, tickX - 1, majorTickBottomY, ST77XX_WHITE);  // Projected major ticks
    }
    const float centerWeight = compassCenterWeight(projectedX, static_cast<float>(centerX));
    uint8_t labelVariant = static_cast<uint8_t>(centerWeight * static_cast<float>(Compass_Label_Bitmap_Variant_Count - 1) + 0.5f);
    if (labelVariant >= Compass_Label_Bitmap_Variant_Count) {
      labelVariant = Compass_Label_Bitmap_Variant_Count - 1;
    }
    const float scale = majorTick ? majorLabelScale : minorLabelScale;
    const int16_t labelW = static_cast<int16_t>(Compass_Label_Bitmap_Width * scale + 0.5f);
    const int16_t labelH = static_cast<int16_t>(Compass_Label_Bitmap_Height * scale + 0.5f);
    const int16_t labelX = tickX - (labelW / 2);
    const int16_t labelY = labelBaseY;
    if (labelX < -labelW || labelX >= bannerWidth || labelY + labelH < compassY + 12 || labelY > compassY + 48) {
      continue;
    }
    const uint8_t *labelBitmap = marker.variants[labelVariant];
    drawScaledXBitmap(target, labelX, labelY, Compass_Label_Bitmap_Width, Compass_Label_Bitmap_Height, labelBitmap, scale, ST77XX_WHITE);  // Projected labels
  }
}

// Main Display
void updateDisplay(Adafruit_GFX &target, float Depth, int Minutes, int Seconds, const DecoResult &deco) {
  // Depth block at top-left (156+24+96=288 pixels wide, 104 pixels high)
  const int32_t depthTenths = static_cast<int32_t>(Depth * 10.0f + ((Depth >= 0.0f) ? 0.5f : -0.5f));
  const int32_t depthTenthsAbs = (depthTenths >= 0) ? depthTenths : -depthTenths;
  const uint16_t depthInteger = static_cast<uint16_t>(depthTenthsAbs / 10);
  const uint8_t depthDecimal = static_cast<uint8_t>(depthTenthsAbs % 10);
  char integerPart[3];
  char decimalPart[2];
  const char *unitPart = "m";
  snprintf(integerPart, sizeof(integerPart), "%2u", depthInteger);
  snprintf(decimalPart, sizeof(decimalPart), "%u", depthDecimal);
  constexpr uint8_t integerSize = 13;
  constexpr uint8_t decimalSize = 8;
  constexpr uint8_t dotSize = 5;
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
  target.setTextSize(integerSize);
  target.getTextBounds(integerPart, 0, 0, &x1, &y1, &integerW, &integerH);
  target.setTextSize(decimalSize);
  target.getTextBounds(decimalPart, 0, 0, &x1, &y1, &decimalW, &decimalH);
  target.setTextSize(decimalSize);
  target.getTextBounds(unitPart, 0, 0, &x1, &y1, &unitW, &unitH);
  target.setTextSize(dotSize);
  target.getTextBounds(dotPart, 0, 0, &x1, &y1, &dotW, &dotH);
  const int16_t depthX = 0;  // Top-left
  const int16_t depthY = 0;
  const int16_t depthBottom = depthY + static_cast<int16_t>(integerH);
  const int16_t dotX = depthX + static_cast<int16_t>(integerW) + 2;
  const int16_t dotY = depthBottom - static_cast<int16_t>(dotH) - 8;
  const int16_t decimalX = dotX + static_cast<int16_t>(dotW) + 2;
  const int16_t decimalY = depthBottom - static_cast<int16_t>(decimalH);
  const int16_t unitX = decimalX + static_cast<int16_t>(decimalW) + 2;
  const int16_t unitY = depthBottom - static_cast<int16_t>(unitH);
  target.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  target.setTextSize(integerSize);
  target.setCursor(depthX, depthY);
  target.print(integerPart);
  target.setTextSize(dotSize);
  target.setCursor(dotX, dotY);
  target.print(dotPart);
  target.setTextSize(decimalSize);
  target.setCursor(decimalX, decimalY);
  target.print(decimalPart);
  target.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  target.setTextSize(decimalSize);
  target.setCursor(unitX, unitY);
  target.print(unitPart);
  target.drawFastHLine(0, depthBottom + 2, LCD_Width, ST77XX_BLUE);  // Depth - Time/Heading seperator
  //Battery indicator at top-right
  drawBatteryIndicator(target, Battery_Percentage);
  // Current GF below battery
  char gfStr[6];
  if (ripNtear_Mode) {
    snprintf(gfStr, sizeof(gfStr), "99/99");
  } else {
    snprintf(gfStr, sizeof(gfStr), "60/85");
  }
  target.setTextSize(2);
  target.setTextColor(ripNtear_Mode ? ST77XX_RED : ST77XX_GREEN, ST77XX_BLACK);
  target.setCursor(260, 20);
  target.print(gfStr);
  // Timer at middle-left
  char timerString[8];
  snprintf(timerString, sizeof(timerString), "%3u:%02u", Minutes, Seconds);
  target.setTextSize(4);
  if (Stopwatch_Active) {
    target.setTextColor(ST77XX_GREEN, ST77XX_RED);  // Red background for stopwatch
    target.fillRect(0, 111, 180, 4, ST77XX_RED);  // Top padding
    target.fillRect(144, 111, 16, 35, ST77XX_RED); // Right padding
  } else {
    target.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  }
  target.setCursor(0, 114);
  target.print(timerString);
  target.drawFastVLine(190, depthBottom + 3, 44, ST77XX_BLUE);  // Time - Heading seperator
  // Heading at middle-right
  drawHeadingValue(target, 236, 114, 4, Heading);
  // Deco table below between Timer/Heading and Compass
  target.drawFastHLine(0, 151, LCD_Width, ST77XX_BLUE);  // Top line
  target.drawFastHLine(0, 196, LCD_Width, ST77XX_BLUE);  // Bottom line
  target.drawFastVLine(0, 152, 40, ST77XX_BLUE);  // Left edge
  target.drawFastVLine(79, 152, 40, ST77XX_BLUE);  // Column 1/2
  target.drawFastVLine(159, 152, 40, ST77XX_BLUE);  // Column 2/3
  target.drawFastVLine(239, 152, 40, ST77XX_BLUE);  // Column 3/4
  target.drawFastVLine(319, 152, 40, ST77XX_BLUE);  // Right edge
  target.setTextSize(3);
  target.setTextColor(ST77XX_CYAN);
  target.setCursor(8, 165);
  target.print("D");  // Depth
  target.setCursor(88, 165);
  target.print("S");  // Stop
  target.setTextSize(2);
  target.setCursor(168, 154);
  target.print("T");  // TTS
  target.setCursor(168, 169);
  target.print("T");
  target.setCursor(168, 184);
  target.print("S");
  target.setCursor(248, 154);
  target.print("s");  // sGF
  target.setCursor(248, 169);
  target.print("G");
  target.setCursor(248, 184);
  target.print("F");
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
  drawColCentreText(target, stopStr, 16, 54, 165, 3, ST77XX_WHITE);  // First stop depth (m)
  drawColCentreText(target, timeStr, 96, 54, 165, 3, ST77XX_WHITE);  // First stop time (min)
  drawColCentreText(target, ttsStr, 176, 54, 165, 3, ST77XX_WHITE);  // Time to surface (min)
  drawColCentreText(target, sGfStr, 256, 54, 165, 3, surfGfColor);  // Surface GF (%)
  // Compass banner at the bottom
  drawCompassBanner(target, Heading);
}

  // Show Startup Image
  void showImage() {
    const uint16_t w = 320;
    const uint16_t h = 240;
    const size_t pixels = static_cast<size_t>(w) * static_cast<size_t>(h);
    uint16_t *buf = static_cast<uint16_t *>(heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buf == nullptr) {
      buf = static_cast<uint16_t *>(malloc(pixels * sizeof(uint16_t)));
    }
    if (buf == nullptr) return;
    for (size_t i = 0; i < pixels; ++i) {
      buf[i] = pgm_read_word(&image[i]);  // RGB565 bitmap
    }
    display.startWrite();
    display.setAddrWindow(0, 0, w, h);
    display.writePixels(buf, static_cast<uint32_t>(pixels), true, false);
    display.endWrite();
    free(buf);
  }

  // WiFi AP JPEG Upload Server
  void startJpegUploadServer() {
    if (Display_Task_Handle != nullptr) {
      vTaskSuspend(Display_Task_Handle);
    }
    esp_wifi_start();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, Password);
    const IPAddress apIP = WiFi.softAPIP();
    display.fillScreen(ST77XX_BLACK);
    display.setTextColor(ST77XX_WHITE);
    display.setTextSize(3);
    display.setCursor(10, 10);
    display.print("Upload JPEG");
    display.setTextSize(2);
    display.setTextColor(ST77XX_CYAN);
    display.setCursor(10, 60);
    display.print("SSID: ");
    display.print(SSID);
    display.setCursor(10, 85);
    display.print("Password: ");
    display.print(Password);
    display.setCursor(10, 110);
    WebServer server(80);
    static const char *uploadHtml =
      "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Dive Computer</title><style>body{font-family:sans-serif;text-align:center;padding:20px}"
      "h2{color:#0af}input[type=file],input[type=submit]{margin:10px;padding:8px 16px;font-size:16px}"
      "input[type=submit]{background:#0af;color:#fff;border:none;border-radius:4px;cursor:pointer}</style></head>"
      "<body><h2>Upload JPEG</h2>"
      "<p>Image must be 320x240 pixels.</p>"
      "<form method='POST' action='/upload' enctype='multipart/form-data'>"
      "<input type='file' name='file' accept='.jpg,.jpeg'><br>"
      "<input type='submit' value='Upload'></form></body></html>";
    server.on("/", HTTP_GET, [&]() {
      server.send(200, "text/html", uploadHtml);
    });
    server.on("/upload", HTTP_POST,
      [&]() {
        server.send(200, "text/plain", "OK");
      },
      [&]() {
        HTTPUpload &upload = server.upload();
        static File uploadFile;
        if (upload.status == UPLOAD_FILE_START) {
          if (SPIFFS.exists(Spiffs_Jpeg_Path)) {
            SPIFFS.remove(Spiffs_Jpeg_Path);
          }
          uploadFile = SPIFFS.open(Spiffs_Jpeg_Path, FILE_WRITE);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (uploadFile) {
            uploadFile.write(upload.buf, upload.currentSize);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          if (uploadFile) {
            uploadFile.close();
          }
          display.fillScreen(ST77XX_BLACK);
          display.setTextSize(3);
          display.setTextColor(ST77XX_GREEN);
          display.setCursor(10, 90);
          display.print("Upload Done!");
          display.setTextSize(2);
          display.setTextColor(ST77XX_WHITE);
          display.setCursor(10, 140);
          display.print("Restarting...");
          delay(Message_MS);
          esp_restart();
        }
      }
    );
    server.begin();
    for (;;) {
      server.handleClient();
      delay(1);
    }
  }

  // SPIFFS JPEG Decode Callback
  bool jpegOutputCallback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
    display.startWrite();
    display.setAddrWindow(x, y, w, h);
    display.writePixels(bitmap, static_cast<uint32_t>(w) * static_cast<uint32_t>(h), true, false);
    display.endWrite();
    return true;
  }
  bool initSpiffsJpegSource() {
    if (!SPIFFS.begin(true)) {
      return false;
    }
    if (!SPIFFS.exists(Spiffs_Jpeg_Path)) {
      return false;
    }
    uint16_t jpgW = 0;
    uint16_t jpgH = 0;
    if (TJpgDec.getFsJpgSize(&jpgW, &jpgH, Spiffs_Jpeg_Path, SPIFFS) != JDR_OK) {
      return false;
    }
    if (jpgW != LCD_Width || jpgH != LCD_Height) {
      return false;
    }
    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(true);
    TJpgDec.setCallback(jpegOutputCallback);
    return true;
  }

  // Display JPEG from SPIFFS
  void showSpiffsJpeg() {
    if (!Spiffs_Jpeg_Ready) {
      if (Display_Task_Handle != nullptr) {
        vTaskSuspend(Display_Task_Handle);
      }
      display.fillScreen(ST77XX_BLACK);
      drawCentreText("No Image", 104, 3, ST77XX_RED);
      delay(Message_MS);
      if (Display_Task_Handle != nullptr) {
        vTaskResume(Display_Task_Handle);
      }
      Frame_Have_Previous = false;
      return;
    }
    if (Display_Task_Handle != nullptr) {
      vTaskSuspend(Display_Task_Handle);
    }
    display.fillScreen(ST77XX_BLACK);
    const JRESULT decodeResult = TJpgDec.drawFsJpg(0, 0, Spiffs_Jpeg_Path, SPIFFS);
    if (decodeResult == JDR_OK) {
      delay(Message_MS * 3);
    } else {
      display.fillScreen(ST77XX_BLACK);
      drawCentreText("Image Error", 104, 3, ST77XX_RED);
      delay(Message_MS);
    }
    display.fillScreen(ST77XX_BLACK);
    if (Display_Task_Handle != nullptr) {
      vTaskResume(Display_Task_Handle);
    }
    Frame_Have_Previous = false;
  }


void setup() {
  // Power on
  pinMode(Button_Pin, INPUT_PULLUP);
  pinMode(RST_Pin, INPUT);
  pinMode(Backlight_Pin, OUTPUT);
  digitalWrite(Backlight_Pin, LOW);  // Backlight initially off
  delay(10);
  bool buttonHeld = true;
  const uint32_t Boot_Time_MS = millis();
  while ((millis() - Boot_Time_MS) < Button_Long_MS) {
    if (digitalRead(Button_Pin) == HIGH) {
      buttonHeld = false;
      break;
    }
    delay(10);
  }
  if (!buttonHeld) {
    esp_sleep_enable_ext0_wakeup(Button_Pin, 0);
    esp_deep_sleep_start();  // Deep sleep if button not held at startup
  }

  // Power conservation
  esp_wifi_stop();
  esp_bt_controller_disable();
  esp_pm_config_esp32s3_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 40,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);

  // ADC initialisation
  pinMode(Battery_Pin, INPUT);
  analogReadResolution(12);  // Internal ADC resolution 12-bit
  analogSetAttenuation(ADC_11db);  // 2.5V range

  // Display initialisation
  spi.begin(SCK_Pin, -1, MOSI_Pin, LCD_CS_Pin);
  display.init(LCD_Width, LCD_Height, SPI_MODE3);
  display.setSPISpeed(40000000);  // 40 MHz SPI clockspeed
  display.setRotation(1);  // Rotate to landscape
  display.fillScreen(ST77XX_BLACK);
  analogWrite(Backlight_Pin, Backlight_High);
  showImage();  // Show startup image
  delay(Message_MS);
  display.fillScreen(ST77XX_BLACK);

  // SPIFFS initialisation
  Spiffs_Jpeg_Ready = initSpiffsJpegSource();

  // MS5837 initialisation if not in demo mode
  loadDemoMode();
  Simulated_Profile_Start_MS = millis();
  if (!Demo_Mode) {
    // Power MS5837
    pinMode(Sensor_VCC_Pin, OUTPUT);
    digitalWrite(Sensor_VCC_Pin, HIGH);
    delay(10);
    // MS5837 dedicated I2C initialisation
    sensorWire.begin(Sensor_SDA_Pin, Sensor_SCL_Pin);
    sensorWire.setClock(400000);  // 400 kHz I2C clcokspeed
    delay(10);
    // MS5837 initialisation
    sensorAvailable = sensor.init(sensorWire);  // Detect MS5837 presence
    if (sensorAvailable) {
      sensor.setModel(MS5837::MS5837_30BA);  // 30 bar model
      sensor.setFluidDensity(Density);  // Set fluid density
    }
  }

  // Peripheral I2C initialisation
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(400000);  // 400 kHz I2C clcokspeed
  delay(10);

  // BH1750 initialisation
  lightMeter.begin(BH1750::CONTINUOUS_LOW_RES_MODE, BH1750_I2C_Address, &Wire);  // 4 lux resolution, 16 ms conversion time

  // QMI8658 initialisation
  qmi.begin(Wire, QMI8658_I2C_Address, SDA_Pin, SCL_Pin);
  qmi.enableSyncSampleMode();
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_112_1Hz, SensorQMI8658::LPF_MODE_2);
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_2);
  qmi.enableGyroscope();
  qmi.enableAccelerometer();  

  // QMC5883P initialisation
  qmc.begin(Wire, QMC5883P_I2C_Address, SDA_Pin, SCL_Pin);
  qmc.configMagnetometer(OperationMode::CONTINUOUS_MEASUREMENT, MagFullScaleRange::FS_8G, 100.0f, MagOverSampleRatio::OSR_8, MagDownSampleRatio::DSR_8);  // ODR 100 Hz
  compassConfigureReferenceField(Reference_Field_Gauss, Magnetometer_Lsb_Per_Gauss);
  if (!loadCompassCalibrationFromNVS()) {
    Compass_Matrices = {};
    compassSetCalibrationMatrices(nullptr);
  }
  ahrs.begin(100.0f);  // AHRS update rate 100 Hz

  // ZHL-16C initialisation
  decoSetup(GF_Low, GF_High, Setpoint);
  decoInit();
  Deco_Last_Update_MS = millis();

  // PSRAM frame buffer initialisation
  if (!initFrameBuffers()) {
    display.fillScreen(ST77XX_BLACK);
    drawCentreText("PSRAM", 96, 3, ST77XX_RED);
    drawCentreText("Error", 144, 3, ST77XX_RED);
    delay(Message_MS);
    display.fillScreen(ST77XX_BLACK);
  }

  // Display update on core 1 at 20 Hz
  xTaskCreatePinnedToCore(
      [](void *) {
        TickType_t lastWakeTick = xTaskGetTickCount();
        const TickType_t displayPeriodTicks = pdMS_TO_TICKS(Display_Update_MS);
        for (;;) {
          const int displayMinutes = Stopwatch_Active ? Stopwatch_Minutes : Minutes;
          const int displaySeconds = Stopwatch_Active ? Stopwatch_Seconds : Seconds;
          if (Frame_Canvas != nullptr && Frame_Back_Current != nullptr) {
            Frame_Canvas->fillScreen(ST77XX_BLACK);
            updateDisplay(*Frame_Canvas, Depth, displayMinutes, displaySeconds, lastDecoResult);
            memcpy(Frame_Back_Current, Frame_Canvas->getBuffer(), Frame_Buffer_Bytes);
            flushDirtyRectFromPSRAM();
          } else {
            updateDisplay(display, Depth, displayMinutes, displaySeconds, lastDecoResult);  // No PSRAM fallback
          }
          if (displayPeriodTicks > 0) {
            vTaskDelayUntil(&lastWakeTick, displayPeriodTicks);
          } else {
            taskYIELD();
          }
        }
      },
      "DisplayTask",
      12288,  // Stack size 12 KB
      nullptr,
      2,  // Priority 2
      &Display_Task_Handle,
      1);  // Core 1

  // Sensor reading and caluclation on core 0
  xTaskCreatePinnedToCore(
      [](void *) {
        TickType_t lastWakeTick = xTaskGetTickCount();
        const TickType_t fastPeriodTicks = pdMS_TO_TICKS(Button_Update_MS);
        uint64_t lastButtonUpdateMS = 0;
        uint64_t lastDepthUpdateMS = 0;
        for (;;) {
          const uint64_t nowMS = millis();
          // 100 Hz tasks
          if ((nowMS - lastButtonUpdateMS) >= Button_Update_MS) {
            lastButtonUpdateMS = nowMS;
            updateStopwatch(nowMS);
            // Press button detection
            const PressButtonEvent buttonEvent = checkButton(nowMS);
            if (buttonEvent == PressButtonEvent::LongPress) {
              // Long press to display image from SPIFFS
              showSpiffsJpeg();
            } else if (buttonEvent == PressButtonEvent::HoldPress) {
              if (Depth >= Dive_Start_Depth) {
                // Hold button to change GF underwater
                ripNtear_Mode = !ripNtear_Mode;
                ripNtear(ripNtear_Mode);
                Deco_Last_Update_MS = nowMS - Deco_Update_MS;
              } else {
                // Hold button to deep sleep on surface
                if (Display_Task_Handle != nullptr) {
                  vTaskSuspend(Display_Task_Handle);
                }
                display.fillScreen(ST77XX_BLACK);
                drawCentreText("Deep", 75, 4, ST77XX_CYAN);
                drawCentreText("Off", 135, 4, ST77XX_CYAN);
                delay(Message_MS);
                display.fillScreen(ST77XX_BLACK);
                esp_sleep_enable_ext0_wakeup(Button_Pin, 0);
                esp_deep_sleep_start();
              }
            } else if (buttonEvent == PressButtonEvent::ShortPress) {
              // Short press toggles stopwatch
              if (Stopwatch_Active) {
                stopAndResetStopwatch();
              } else {
                startStopwatch(nowMS);
              }
            }
            // Boot button detection
            const BootButtonEvent bootEvent = checkBootButton(nowMS);
            if (bootEvent == BootButtonEvent::LongPress) {
              // Long press boot button to toggle demo mode
              Demo_Mode = !Demo_Mode;
              saveDemoMode();
              if (Display_Task_Handle != nullptr) {
                vTaskSuspend(Display_Task_Handle);
              }
              display.fillScreen(ST77XX_BLACK);
              drawCentreText("Demo", 75, 4, ST77XX_CYAN);
              drawCentreText(Demo_Mode ? "On" : "Off", 135, 4, ST77XX_CYAN);
              delay(Message_MS);
              esp_restart();  // Restart after mode change
            } else if (bootEvent == BootButtonEvent::HoldPress) {
              // Hold boot button to upload JPEG image via WiFi AP
              startJpegUploadServer();
            } else if (bootEvent == BootButtonEvent::ShortPress) {
              if (Depth >= Dive_Start_Depth) {
                // Short press underwater to deep sleep (shut down in case of MS5837 error)
                if (Display_Task_Handle != nullptr) {
                  vTaskSuspend(Display_Task_Handle);
                }
                display.fillScreen(ST77XX_BLACK);
                drawCentreText("Power", 75, 4, ST77XX_CYAN);
                drawCentreText("Off", 135, 4, ST77XX_CYAN);
                delay(Message_MS);
                display.fillScreen(ST77XX_BLACK);
                esp_sleep_enable_ext0_wakeup(Button_Pin, 0);
                esp_deep_sleep_start();
              } else {
                if (Display_Task_Handle != nullptr) {
                  vTaskSuspend(Display_Task_Handle);
                }
                // Short press on surface to start compass calibration
                display.fillScreen(ST77XX_BLACK);
                drawCentreText("Compass", 75, 4, ST77XX_WHITE);
                drawCentreText("Calibration", 135, 4, ST77XX_WHITE);
                delay(Calibration_Delay_MS);
                display.fillScreen(ST77XX_BLACK);
                drawCentreText("Calibrating", 104, 4, ST77XX_CYAN);
                std::vector<float> mag_samples_xyz;
                collectCompassSamples(mag_samples_xyz);
                display.fillScreen(ST77XX_BLACK);
                drawCentreText("Computing", 104, 4, ST77XX_YELLOW);
                const bool calibration_ok = computeCompassCalibration(mag_samples_xyz);
                drawCalibrationDiagnostics(calibration_ok);
                delay(Message_MS * 3);
                drawCalibrationScore();
                delay(Message_MS * 3);
                display.fillScreen(ST77XX_BLACK);
                esp_restart();  // Restart after calibration
              }
            }
            // Compass reading
            Heading = readCompassHeading();
          }
          // 2 Hz tasks
          if ((nowMS - lastDepthUpdateMS) >= Depth_Update_MS) {
            lastDepthUpdateMS = nowMS;
            // Depth
            float pressureAtm = 1.0f;
            if (Demo_Mode) {
              // Demo dive profile
              const uint64_t elapsedProfileMS = nowMS - Simulated_Profile_Start_MS;
              Depth = simulatedDepthMeters(elapsedProfileMS);
              pressureAtm = 1.0f + (Depth / 10.0f);
              if (pressureAtm < 1.0f) {
                pressureAtm = 1.0f;
              }
            } else if (sensorAvailable) {
              // MS5837 reading
              sensor.read();
              Depth = sensor.depth() - Depth_Offset;
              pressureAtm = sensor.pressure() / MBAR_PER_ATM;
            } else {
              // MS5837 error fallback
              Depth = 99.9f;
              pressureAtm = 11.0f;
            }
            // Timer
            updateTimer(Depth, nowMS);
            // Backlight and battery
            Ambient_Lux = lightMeter.readLightLevel();
            Battery_Percentage = readBatteryPercentage();
            if (Battery_Percentage <= Low_Battery_Threshold) {
              Backlight_Level = Backlight_Low;  // Low battery dims backlight
            } else {
              Backlight_Level = ambientBacklightLevel(Ambient_Lux);
            }
            analogWrite(Backlight_Pin, Backlight_Level);
            if (Battery_Percentage <= Critical_Battery_Threshold) {
              if (Display_Task_Handle != nullptr) {
                vTaskSuspend(Display_Task_Handle);
              }
              display.fillScreen(ST77XX_BLACK);
              drawCentreText("Battery", 86, 4, ST77XX_RED);
              drawCentreText("Low", 144, 4, ST77XX_RED);
              delay(Message_MS);
              display.fillScreen(ST77XX_BLACK);
              esp_sleep_enable_ext0_wakeup(Button_Pin, 0);
              esp_deep_sleep_start();  // Critical battery shut down
            }
            // 0.2 Hz tasks
            if ((nowMS - Deco_Last_Update_MS) >= Deco_Update_MS) {
              const float dtMin = static_cast<float>(nowMS - Deco_Last_Update_MS) / 60000.0f;  // Minutes since last update
              Deco_Last_Update_MS = nowMS;
              decoUpdate(pressureAtm, dtMin);  // ZHL-16C tissue update
              lastDecoResult = decoCompute(pressureAtm);  // Decompression calculation
            }
          }
          if (fastPeriodTicks > 0) {
            vTaskDelayUntil(&lastWakeTick, fastPeriodTicks);
          } else {
            taskYIELD();
          }
        }
      },
      "SensorTask",
      32768,  // Stack size 32 KB
      nullptr,
      2,  // Priority 2
      &Sensor_Task_Handle,
      0);  // Core 0
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}