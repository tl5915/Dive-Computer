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
#include <SensorQMI8658.hpp>
#include <SensorQMC5883P.hpp>
// Data Processing
#include <vector>


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
SensorQMI8658 qmi;
SensorQMC5883P qmc;
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

// I2C Addresses
constexpr uint8_t QMI8658_I2C_Address = 0x6B;
constexpr uint8_t QMC5883P_I2C_Address = 0x2C;

// LCD Constants
constexpr uint16_t LCD_Width = 260;
constexpr uint16_t LCD_Height = 280;
constexpr uint8_t Backlight_High = 255;            // High backlight in bright surroundings
constexpr uint32_t Display_Update_MS = 50;         // Display refresh rate: 20 Hz
constexpr uint32_t Fast_Update_MS = 100;           // Heading/tap/button checks: 10 Hz

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
float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;
float mx = 0.0f;
float my = 0.0f;
float mz = 0.0f;

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

// Main Display
void updateDisplay(Adafruit_GFX &target) {
  target.fillScreen(ST77XX_BLACK);
  target.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  target.setTextSize(2);
  target.setCursor(20, 20);
  target.print("Acc X:");
  target.print(ax, 3);
  target.setCursor(20, 40);
  target.print("Acc Y:");
  target.print(ay, 3);
  target.setCursor(20, 60);
  target.print("Acc Z:");
  target.print(az, 3);
  target.setCursor(20, 120);
  target.print("Mag X:");
  target.print(mx, 3);
  target.setCursor(20, 140);
  target.print("Mag Y:");
  target.print(my, 3);
  target.setCursor(20, 160);
  target.print("Mag Z:");
  target.print(mz, 3);
}


void setup() {
  // Power on
  pinMode(Power_Pin, OUTPUT);
  digitalWrite(Power_Pin, HIGH);
  delay(10);
  pinMode(Sys_Pin, INPUT);
  // Power conservation
  pinMode(Buzz_Pin, OUTPUT);
  digitalWrite(Buzz_Pin, LOW);
  esp_wifi_stop();
  esp_bt_controller_disable();
  esp_pm_config_esp32s3_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 40,
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
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G,
                          SensorQMI8658::ACC_ODR_125Hz,
                          SensorQMI8658::LPF_MODE_2);
  qmi.enableAccelerometer();
  qmi.disableGyroscope();

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


  // PSRAM frame buffer initialisation
  initFrameBuffers();

  // Display update on one core
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

  xTaskCreatePinnedToCore(
      [](void *) {
        TickType_t lastWakeTick = xTaskGetTickCount();
        const TickType_t fastPeriodTicks = pdMS_TO_TICKS(Fast_Update_MS);
        uint64_t lastFastUpdateMS = 0;
        uint64_t lastSlowUpdateMS = 0;
        for (;;) {
          const uint64_t nowMS = millis();
          // 10 Hz
          if ((nowMS - lastSlowUpdateMS) >= Fast_Update_MS) {
            lastSlowUpdateMS = nowMS;
            // Power button
            if (digitalRead(Sys_Pin) == HIGH) {
              digitalWrite(Power_Pin, LOW);
            }
            // Magnetometer
            MagnetometerData mag_data = {};
            qmc.readData(mag_data);
            mx = static_cast<float>(mag_data.raw.x);
            my = static_cast<float>(mag_data.raw.y);
            mz = static_cast<float>(mag_data.raw.z);
            // Accelerometer
            qmi.getAccelerometer(ax, ay, az);
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