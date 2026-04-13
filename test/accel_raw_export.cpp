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
// Data Processing
#include <vector>
#include <WiFi.h>
#include <WebServer.h>

// Magnetometer data collection
struct MagDataPoint {
  float x, y, z;
};

std::vector<MagDataPoint> mag_data_collection;
String collected_data_text;
bool collection_in_progress = false;
bool collection_complete = false;
uint32_t collection_start_ms = 0;
constexpr uint32_t COLLECTION_DURATION_MS = 120000;  // 120 seconds
constexpr uint32_t COLLECTION_SAMPLE_PERIOD_MS = 10;  // 100 Hz
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
constexpr uint8_t Sys_Pin = 40;
constexpr uint8_t Power_Pin = 41;
constexpr uint8_t Buzz_Pin = 42;


// Define Objects
SPIClass spi(FSPI);
SensorQMC5883P qmc;
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

// I2C Addresses
constexpr uint8_t QMC5883P_I2C_Address = 0x2C;

// LCD Constants
constexpr uint16_t LCD_Width = 260;
constexpr uint16_t LCD_Height = 280;
constexpr uint8_t Backlight_High = 255;
constexpr uint32_t Display_Update_MS = 50;
constexpr uint32_t Fast_Update_MS = 100;

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
float mx = 0.0f;
float my = 0.0f;
float mz = 0.0f;
uint32_t samples_collected = 0;
uint32_t boot_time_ms = 0;

// Read Magnetometer
static void readQmcAxesTransformed(float out_mag[3]) {
  MagnetometerData mag_data = {};
  qmc.readData(mag_data);
  const float qmc_x = static_cast<float>(mag_data.raw.x);
  const float qmc_y = static_cast<float>(mag_data.raw.y);
  const float qmc_z = static_cast<float>(mag_data.raw.z);
  out_mag[0] = -qmc_x;  // Right = -QMC_X
  out_mag[1] = -qmc_y;  // Down = -QMC_Y
  out_mag[2] = -qmc_z;  // Forward = -QMC_Z
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
  uint32_t time_since_boot = now_ms - boot_time_ms;
  if (time_since_boot < 10000) {
    // Countdown
    uint32_t count_down = (10000 - time_since_boot) / 1000;
    target.setTextSize(10);
    target.setCursor(110, 90);
    target.print(count_down);
  } else if (collection_in_progress) {
    // Collecting
    target.setTextSize(4);
    target.setCursor(50, 50);
    target.print("Collecting");
    uint32_t elapsed = now_ms - collection_start_ms;
    uint32_t remaining_sec = (COLLECTION_DURATION_MS - elapsed) / 1000;
    target.setTextSize(3);
    target.setCursor(50, 150);
    target.print(remaining_sec);
    target.setCursor(50, 200);
    target.print("Count: ");
    target.print(samples_collected);
  } else if (collection_complete) {
    // Completed
    target.setTextSize(4);
    target.setCursor(50, 50);
    target.print("Done!");
    target.setTextSize(3);
    target.setCursor(20, 150);
    target.print("WiFi: ESP32");
    target.setCursor(20, 200);
    target.print("pwd: 12345678");
  }
}

// Web Server
void handleStatus() {
  String status = "Magnetometer Raw Data Export\n\n";
  if (collection_complete) {
    status += "Status: COMPLETE\n";
    status += "Samples: ";
    status += String(samples_collected);
    status += "\n\n";
    status += "Download: GET /download\n";
    status += "View: GET /view\n";
  } else if (collection_in_progress) {
    status += "Status: Collecting in progress...\n";
  } else {
    status += "Status: Waiting to start...\n";
  }
  server.send(200, "text/plain", status);
}

void handleDataDownload() {
  server.sendHeader("Content-Disposition", "attachment; filename=\"magnetometer_raw.txt\"");
  server.send(200, "text/plain", collected_data_text);
}

void handleDataView() {
  server.send(200, "text/plain", collected_data_text);
}

void setup() {
  // Power on
  pinMode(Power_Pin, OUTPUT);
  digitalWrite(Power_Pin, HIGH);
  delay(10);
  pinMode(Sys_Pin, INPUT);
  
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

  // QMC5883P initialisation
  pinMode(Boot_Pin, INPUT);
  qmc.begin(Wire, QMC5883P_I2C_Address, SDA_Pin, SCL_Pin);
  qmc.configMagnetometer(
      OperationMode::CONTINUOUS_MEASUREMENT,
      MagFullScaleRange::FS_8G,
      200.0f,
      MagOverSampleRatio::OSR_8,
      MagDownSampleRatio::DSR_8
    );

  // PSRAM frame buffer initialisation
  initFrameBuffers();
  
  // Initialize WiFi AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32", "12345678");
  
  // Setup web server
  server.on("/", handleStatus);
  server.on("/download", handleDataDownload);
  server.on("/view", handleDataView);
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
        uint64_t lastCollectionSampleMS = 0;
        for (;;) {
          const uint64_t nowMS = millis();
          uint32_t time_since_boot = nowMS - boot_time_ms;
          // Check time
          if (!collection_in_progress && !collection_complete && time_since_boot >= 10000) {
            collection_in_progress = true;
            collection_start_ms = nowMS;
            mag_data_collection.clear();
            samples_collected = 0;
            lastCollectionSampleMS = nowMS;
          }
          // Handle collection
          if (collection_in_progress) {
            uint32_t elapsed = nowMS - collection_start_ms;
            // Collect samples
            if ((nowMS - lastCollectionSampleMS) >= COLLECTION_SAMPLE_PERIOD_MS) {
              float mag[3] = {0.0f, 0.0f, 0.0f};
              readQmcAxesTransformed(mag);
              mag_data_collection.push_back({mag[0], mag[1], mag[2]});
              samples_collected++;
              lastCollectionSampleMS = nowMS;
            }
            // Check if collection is done
            if (elapsed >= COLLECTION_DURATION_MS) {
              collection_in_progress = false;
              collection_complete = true;
              // Format data
              collected_data_text = "";
              for (const auto& point : mag_data_collection) {
                collected_data_text += String(point.x, 2);
                collected_data_text += "\t";
                collected_data_text += String(point.y, 2);
                collected_data_text += "\t";
                collected_data_text += String(point.z, 2);
                collected_data_text += "\n";
              }
            }
          }
          // Handle web server
          if (collection_complete) {
            server.handleClient();
          }
          
          if (fastPeriodTicks > 0) {
            vTaskDelayUntil(&lastWakeTick, fastPeriodTicks);
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