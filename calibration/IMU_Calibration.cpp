#include <Arduino.h>
// Power Management
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_pm.h>
// Peripherals
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SensorQMC5883P.hpp>
#include <SensorQMI8658.hpp>
// Data Processing
#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include <magnetometer_calibration.h>

// IMU data collection
struct ImuDataPoint {
  float x, y, z;
};
std::vector<ImuDataPoint> mag_data_collection;
std::vector<ImuDataPoint> accel_raw_collection;
std::vector<ImuDataPoint> gyro_raw_collection;
String mag_data_text;
String mag_calibrated_text;
String accel_data_text;
String gyro_data_text;
String calibration_html_text = "<html><body><h2>No run yet</h2><p>Press button to collect raw IMU data.</p></body></html>";
bool collection_in_progress = false;
bool collection_complete = false;
bool collection_start_pending = false;
uint32_t collection_start_ms = 0;
uint32_t collection_pending_since_ms = 0;
constexpr uint32_t COLLECTION_DURATION_MS = 60000;    // 60 seconds
constexpr uint32_t COLLECTION_START_DELAY_MS = 5000;  // 5 seconds
constexpr uint32_t COLLECTION_SAMPLE_PERIOD_MS = 10;  // 100 Hz
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
WebServer server(80);

// Pins
constexpr uint8_t Boot_Pin = 0;
constexpr uint8_t Battery_Pin = 1;
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
constexpr uint8_t Button_Pin = 40;
constexpr uint8_t Power_Pin = 41;
constexpr uint8_t Buzz_Pin = 42;


// Define Objects
SPIClass spi(FSPI);
SensorQMC5883P qmc;
SensorQMI8658 qmi;
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

// I2C Addresses
constexpr uint8_t QMC5883P_I2C_Address = 0x2C;
constexpr uint8_t QMI8658_I2C_Address = 0x6B;

// LCD Constants
constexpr uint16_t LCD_Width = 260;
constexpr uint16_t LCD_Height = 280;
constexpr uint8_t Backlight_High = 255;
constexpr uint32_t Display_Update_MS = 50;
constexpr uint32_t Fast_Update_MS = 100;

// IMU Constants
constexpr float Accel_Offset_X = 0.093489f;
constexpr float Accel_Offset_Y = -0.051529f;
constexpr float Accel_Offset_Z = 0.034036f;
constexpr float Gyro_Offset_X = 0.4791f;
constexpr float Gyro_Offset_Y = 5.7993f;
constexpr float Gyro_Offset_Z = -0.7015f;

// Compass Constants
constexpr float Compass_Offset = 0.0;                  // Compass offset degrees
constexpr float Reference_Field_Gauss = 0.49f;         // UK average magnetic field strength
constexpr float Magnetometer_Lsb_Per_Gauss = 3750.0f;  // QMC5883P at FS_8G: 3750 LSB/Gauss

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
CompassCalibrationMatrices Compass_Matrices = Default_Compass_Matrices;

// RTOS
TaskHandle_t Display_Task_Handle = nullptr;
TaskHandle_t Sensor_Task_Handle = nullptr;
GFXcanvas16 *Frame_Canvas = nullptr;
uint16_t *Frame_Back_Previous = nullptr;
uint16_t *Frame_Back_Current = nullptr;
size_t Frame_Buffer_Bytes = 0;
uint16_t Render_Width = 0;
uint16_t Render_Height = 0;
bool Frame_Have_Previous = false;

// IMU
uint32_t samples_collected = 0;
uint32_t boot_time_ms = 0;
float live_accel_x = 0.0f;
float live_accel_y = 0.0f;
float live_accel_z = 0.0f;
float live_gyro_x = 0.0f;
float live_gyro_y = 0.0f;
float live_gyro_z = 0.0f;
float live_mag_cal_x = 0.0f;
float live_mag_cal_y = 0.0f;
float live_mag_cal_z = 0.0f;
uint8_t button_last_reading = HIGH;
uint8_t button_stable_state = HIGH;
uint32_t button_last_change_ms = 0;


// Read gyroscope in dps (software-calibrated)
static bool readQmiGyroDps(float out_gyro_dps[3]) {
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  if (!qmi.getGyroscope(gx, gy, gz)) {
    return false;
  }
  out_gyro_dps[0] = gx - Gyro_Offset_X;
  out_gyro_dps[1] = gy - Gyro_Offset_Y;
  out_gyro_dps[2] = gz - Gyro_Offset_Z;
  return true;
}

// Read accelerometer in g (software-calibrated)
static bool readQmiAccelG(float out_accel_g[3]) {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  if (!qmi.getAccelerometer(ax, ay, az)) {
    return false;
  }
  out_accel_g[0] = ax - Accel_Offset_X;
  out_accel_g[1] = ay - Accel_Offset_Y;
  out_accel_g[2] = az - Accel_Offset_Z;
  return true;
}

// Read Magnetometer
static void readQmcAxesTransformed(float out_mag[3]) {
  MagnetometerData mag_data = {};
  qmc.readData(mag_data);
  const float mx = static_cast<float>(mag_data.raw.x);
  const float my = static_cast<float>(mag_data.raw.y);
  const float mz = static_cast<float>(mag_data.raw.z);
  out_mag[0] = -mx;  // Right = -QMC_X
  out_mag[1] = -my;  // Down = -QMC_Y
  out_mag[2] = -mz;  // Forward = -QMC_Z
}

// Frame Buffer Management
static bool initFrameBuffers() {
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
static void flushDirtyRectFromPSRAM() {
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

// Display
void updateDisplay(Adafruit_GFX &target) {
  target.fillScreen(ST77XX_BLACK);
  target.setTextColor(ST77XX_WHITE, ST77XX_BLACK);  
  uint32_t now_ms = millis();
  if (collection_in_progress) {
    // Collecting
    target.setTextSize(4);
    target.setCursor(10, 50);
    target.print("Collecting");
    uint32_t elapsed = now_ms - collection_start_ms;
    uint32_t remaining_sec = (COLLECTION_DURATION_MS - elapsed) / 1000;
    target.setTextSize(3);
    target.setCursor(10, 150);
    target.print(remaining_sec);
    target.setCursor(10, 200);
    target.print("Count:");
    target.print(samples_collected);
  } else if (collection_start_pending) {
    // Delay before collection starts
    target.setTextSize(3);
    target.setCursor(10, 40);
    target.print("Starting in");
    uint32_t pending_elapsed = now_ms - collection_pending_since_ms;
    uint32_t remaining_ms = 0;
    if (pending_elapsed < COLLECTION_START_DELAY_MS) {
      remaining_ms = COLLECTION_START_DELAY_MS - pending_elapsed;
    }
    uint32_t remaining_sec = (remaining_ms + 999) / 1000;
    target.setTextSize(6);
    target.setCursor(10, 110);
    target.print(remaining_sec);
  } else if (collection_complete) {
    // Completed
    target.setTextSize(3);
    target.setCursor(10, 20);
    target.print("Data Ready");
    target.setTextSize(2);
    target.setCursor(10, 80);
    target.print("/g gyro dps");
    target.setCursor(10, 100);
    target.print("/a accel g");
    target.setCursor(10, 120);
    target.print("Samples:");
    target.print(samples_collected);
    target.setCursor(10, 180);
    target.print("WiFi: ESP32");
    target.setCursor(10, 200);
    target.print("pwd: 12345678");
    target.setCursor(10, 220);
    target.print("192.168.4.1");
  } else {
    // Idle: show live gyro/accel/calibrated-mag values
    target.setTextSize(2);
    target.setCursor(10, 10);
    target.print("GYR dps");
    target.setCursor(10, 30);
    target.print("X:");
    target.print(live_gyro_x, 2);
    target.setCursor(10, 46);
    target.print("Y:");
    target.print(live_gyro_y, 2);
    target.setCursor(10, 62);
    target.print("Z:");
    target.print(live_gyro_z, 2);

    target.setCursor(10, 92);
    target.print("ACC g");
    target.setCursor(10, 112);
    target.print("X:");
    target.print(live_accel_x, 3);
    target.setCursor(10, 128);
    target.print("Y:");
    target.print(live_accel_y, 3);
    target.setCursor(10, 144);
    target.print("Z:");
    target.print(live_accel_z, 3);

    target.setCursor(10, 174);
    target.print("MAG cal");
    target.setCursor(10, 194);
    target.print("X:");
    target.print(live_mag_cal_x, 3);
    target.setCursor(10, 210);
    target.print("Y:");
    target.print(live_mag_cal_y, 3);
    target.setCursor(10, 226);
    target.print("Z:");
    target.print(live_mag_cal_z, 3);

    target.setTextSize(2);
    target.setCursor(190, 246);
    target.print("dps/g/cal");
  }
}

// Web Server
void handleRootView() {
  server.send(200, "text/html", calibration_html_text);
}
void handleMagView() {
  server.send(200, "text/plain", mag_data_text);
}
void handleMagCalView() {
  server.send(200, "text/plain", mag_calibrated_text);
}
void handleAccelView() {
  server.send(200, "text/plain", accel_data_text);
}
void handleGyroView() {
  server.send(200, "text/plain", gyro_data_text);
}

void setup() {
  // Power on
  pinMode(Power_Pin, OUTPUT);
  digitalWrite(Power_Pin, HIGH);
  delay(10);
  pinMode(Button_Pin, INPUT);
  // Boot time
  boot_time_ms = millis();
  
  // Power conservation
  pinMode(Buzz_Pin, OUTPUT);
  digitalWrite(Buzz_Pin, LOW);
  esp_bt_controller_disable();
  esp_pm_config_esp32s3_t pm_config = {
    .max_freq_mhz = 240,
    .min_freq_mhz = 80,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);

  // Display initialisation
  spi.begin(SCK_Pin, -1, MOSI_Pin, CS_Pin);
  pinMode(Backlight_Pin, OUTPUT);
  analogWrite(Backlight_Pin, Backlight_High);
  display.init(LCD_Width, LCD_Height, SPI_MODE3);
  display.setSPISpeed(40000000); 
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);

  // I2C initialisation
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(400000);
  delay(10);

  // QMI8658 initialisation
  qmi.begin(Wire, QMI8658_I2C_Address, SDA_Pin, SCL_Pin);
  qmi.enableSyncSampleMode();
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS,
                      SensorQMI8658::GYR_ODR_112_1Hz,
                      SensorQMI8658::LPF_MODE_2);
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                          SensorQMI8658::ACC_ODR_125Hz,
                          SensorQMI8658::LPF_MODE_2);
  qmi.enableGyroscope();
  qmi.enableAccelerometer();

  // QMC5883P initialisation
  pinMode(Boot_Pin, INPUT);
  qmc.begin(Wire, QMC5883P_I2C_Address, SDA_Pin, SCL_Pin);
  qmc.configMagnetometer(
      OperationMode::CONTINUOUS_MEASUREMENT,
      MagFullScaleRange::FS_8G,
      100.0f,
      MagOverSampleRatio::OSR_8,
      MagDownSampleRatio::DSR_8
    );
  compassConfigureReferenceField(Reference_Field_Gauss, Magnetometer_Lsb_Per_Gauss);
  compassSetCalibrationMatrices(&Compass_Matrices);

  // PSRAM frame buffer initialisation
  initFrameBuffers();
  
  // Initialise WiFi AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32", "12345678");
  
  // Setup web server
  server.on("/", handleRootView);
  server.on("/m", handleMagView);
  server.on("/m/c", handleMagCalView);
  server.on("/a", handleAccelView);
  server.on("/g", handleGyroView);
  server.begin();

  // Display on core 1
  xTaskCreatePinnedToCore(
      [](void *) {
        TickType_t lastWakeTick = xTaskGetTickCount();
        const TickType_t displayPeriodTicks = pdMS_TO_TICKS(Display_Update_MS);
        for (;;) {
          if (Frame_Canvas != nullptr && Frame_Back_Current != nullptr) {
            Frame_Canvas->fillScreen(ST77XX_BLACK);
            updateDisplay(*Frame_Canvas);
            memcpy(Frame_Back_Current, Frame_Canvas->getBuffer(), Frame_Buffer_Bytes);
            flushDirtyRectFromPSRAM();
          } else {
            updateDisplay(display);
          }
          if (displayPeriodTicks > 0) {
            vTaskDelayUntil(&lastWakeTick, displayPeriodTicks);
          } else {
            taskYIELD();
          }
        }
      },
      "DisplayTask",
      8192,
      nullptr,
      2,
      &Display_Task_Handle,
      1);

  // Collect data on core 0
  xTaskCreatePinnedToCore(
      [](void *) {
        TickType_t lastWakeTick = xTaskGetTickCount();
        const TickType_t fastPeriodTicks = pdMS_TO_TICKS(Fast_Update_MS);
        const TickType_t collectionPeriodTicks = pdMS_TO_TICKS(COLLECTION_SAMPLE_PERIOD_MS);
        uint64_t lastCollectionSampleMS = 0;
        for (;;) {
          const uint64_t nowMS = millis();
          // Update live gyro/accel/calibrated-mag view
          if (!collection_in_progress && !collection_complete) {
            float gyro_dps[3] = {0.0f, 0.0f, 0.0f};
            float accel_g[3] = {0.0f, 0.0f, 0.0f};
            float mag_raw[3] = {0.0f, 0.0f, 0.0f};
            float mag_cal[3] = {0.0f, 0.0f, 0.0f};
            if (readQmiGyroDps(gyro_dps)) {
              live_gyro_x = gyro_dps[0];
              live_gyro_y = gyro_dps[1];
              live_gyro_z = gyro_dps[2];
            }
            if (readQmiAccelG(accel_g)) {
              live_accel_x = accel_g[0];
              live_accel_y = accel_g[1];
              live_accel_z = accel_g[2];
            }
            readQmcAxesTransformed(mag_raw);
            compassApplyCalibration(mag_raw, mag_cal);
            live_mag_cal_x = mag_cal[0];
            live_mag_cal_y = mag_cal[1];
            live_mag_cal_z = mag_cal[2];
          }
          // Start collection
          const uint8_t button_reading = static_cast<uint8_t>(digitalRead(Button_Pin));
          if (button_reading != button_last_reading) {
            button_last_change_ms = static_cast<uint32_t>(nowMS);
            button_last_reading = button_reading;
          }
          if ((nowMS - button_last_change_ms) >= BUTTON_DEBOUNCE_MS) {
            if (button_reading != button_stable_state) {
              button_stable_state = button_reading;
              if (button_stable_state == LOW && !collection_in_progress && !collection_complete && !collection_start_pending) {
                collection_start_pending = true;
                collection_pending_since_ms = static_cast<uint32_t>(nowMS);
              }
            }
          }
          if (collection_start_pending && !collection_in_progress && !collection_complete) {
            if ((nowMS - collection_pending_since_ms) >= COLLECTION_START_DELAY_MS) {
              collection_start_pending = false;
              collection_in_progress = true;
              collection_start_ms = static_cast<uint32_t>(nowMS);
              mag_data_collection.clear();
              accel_raw_collection.clear();
              gyro_raw_collection.clear();
              samples_collected = 0;
              lastCollectionSampleMS = nowMS;
            }
          }
          if (!collection_in_progress || collection_complete) {
            if (collection_complete) {
              server.handleClient();
            }
            if (fastPeriodTicks > 0) {
              vTaskDelayUntil(&lastWakeTick, fastPeriodTicks);
            } else {
              taskYIELD();
            }
            continue;
          }
          // Handle collection
          if (collection_in_progress) {
            collection_in_progress = true;
            uint32_t elapsed = static_cast<uint32_t>(nowMS) - collection_start_ms;
            // Collect samples
            if ((nowMS - lastCollectionSampleMS) >= COLLECTION_SAMPLE_PERIOD_MS) {
              float mag[3] = {0.0f, 0.0f, 0.0f};
              float accel_g[3] = {0.0f, 0.0f, 0.0f};
              float gyro_dps[3] = {0.0f, 0.0f, 0.0f};
              readQmcAxesTransformed(mag);
              const bool accel_ok = readQmiAccelG(accel_g);
              const bool gyro_ok = readQmiGyroDps(gyro_dps);
              mag_data_collection.push_back({mag[0], mag[1], mag[2]});
              if (accel_ok) {
                accel_raw_collection.push_back({accel_g[0], accel_g[1], accel_g[2]});
              }
              if (gyro_ok) {
                gyro_raw_collection.push_back({gyro_dps[0], gyro_dps[1], gyro_dps[2]});
              }
              if (accel_ok && gyro_ok) {
                samples_collected++;
              }
              lastCollectionSampleMS = nowMS;
            }
            // Check if collection is done
            if (elapsed >= COLLECTION_DURATION_MS) {
              collection_in_progress = false;
              collection_complete = true;
              // Format magnetometer data
              mag_data_text = "";
              for (const auto& point : mag_data_collection) {
                mag_data_text += String(point.x, 1);
                mag_data_text += "\t";
                mag_data_text += String(point.y, 1);
                mag_data_text += "\t";
                mag_data_text += String(point.z, 1);
                mag_data_text += "\n";
              }
              // Format calibrated magnetometer data using compass calibration library
              mag_calibrated_text = "";
              for (const auto& point : mag_data_collection) {
                float raw[3] = { point.x, point.y, point.z };
                float calibrated[3] = {0.0f, 0.0f, 0.0f};
                compassApplyCalibration(raw, calibrated);
                mag_calibrated_text += String(calibrated[0], 3);
                mag_calibrated_text += "\t";
                mag_calibrated_text += String(calibrated[1], 3);
                mag_calibrated_text += "\t";
                mag_calibrated_text += String(calibrated[2], 3);
                mag_calibrated_text += "\n";
              }
              // Format accelerometer data (software-calibrated g)
              accel_data_text = "";
              for (const auto& point : accel_raw_collection) {
                accel_data_text += String(point.x, 5);
                accel_data_text += "\t";
                accel_data_text += String(point.y, 5);
                accel_data_text += "\t";
                accel_data_text += String(point.z, 5);
                accel_data_text += "\n";
              }
              // Format gyroscope data (software-calibrated dps)
              gyro_data_text = "";
              for (const auto& point : gyro_raw_collection) {
                gyro_data_text += String(point.x, 5);
                gyro_data_text += "\t";
                gyro_data_text += String(point.y, 5);
                gyro_data_text += "\t";
                gyro_data_text += String(point.z, 5);
                gyro_data_text += "\n";
              }
              calibration_html_text = "<!doctype html><html><head><meta charset='utf-8'>";
              calibration_html_text += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
              calibration_html_text += "<title>IMU Raw Export</title>";
              calibration_html_text += "<style>body{font-family:Arial,sans-serif;background:#f5f7fb;color:#1d2636;padding:20px;}";
              calibration_html_text += "main{max-width:760px;margin:0 auto;background:#fff;padding:20px;border-radius:12px;";
              calibration_html_text += "box-shadow:0 10px 26px rgba(0,0,0,.08);}pre{background:#eef3ff;padding:12px;border-radius:8px;}";
              calibration_html_text += "</style></head><body><main><h1>IMU Export</h1>";
              calibration_html_text += "<ul><li><a href='/m'>/m</a> magnetometer (transformed, pre-cal)</li>";
              calibration_html_text += "<li><a href='/m/c'>/m/c</a> magnetometer (transformed, post-cal)</li>";
              calibration_html_text += "<li><a href='/a'>/a</a> accelerometer (g)</li>";
              calibration_html_text += "<li><a href='/g'>/g</a> gyroscope (dps)</li></ul>";
              calibration_html_text += "<p>Sample count: " + String(samples_collected) + "</p>";
              calibration_html_text += "</main></body></html>";
            }
          }
          // Handle web server
          if (collection_complete) {
            server.handleClient();
          }
          if (collectionPeriodTicks > 0) {
            vTaskDelayUntil(&lastWakeTick, collectionPeriodTicks);
          } else {
            taskYIELD();
          }
        }
      },
      "SensorTask",
      12288,
      nullptr,
      2,
      &Sensor_Task_Handle,
      0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}