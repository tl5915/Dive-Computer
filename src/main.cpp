#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MS5837.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <esp_bt.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include "compass.h"


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
MS5837 sensor;
SPIClass spi(FSPI);
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

// Battery Percentage Lookup Table
constexpr float Voltage_Table[] = {
    3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
    3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
constexpr uint8_t Percentage_Table[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
    55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
constexpr size_t Battery_Table_Size = sizeof(Voltage_Table) / sizeof(Voltage_Table[0]);

// Constants
constexpr uint16_t LCD_Width = 240;
constexpr uint16_t LCD_Height = 280;
constexpr uint32_t Button_Debounce_MS = 50;
constexpr uint32_t Calibration_Delay_MS = 5000;
constexpr uint32_t Message_US = 1000000;
constexpr uint32_t Loop_US = 500000;
constexpr float Depth_Offset = 0.2f;
constexpr float Dive_Start_Depth = 1.0f;
constexpr float Low_Battery_Threshold = 3.7f;
constexpr float Critical_Battery_Threshold = 3.2f;
constexpr float Battery_Divider_Ratio = 3.0f;
constexpr uint8_t Backlight_Normal = 255;
constexpr uint8_t Backlight_Low = 8;

// Variables
float Depth = 0.0f;
float Heading = 0.0f;
float Battery_Voltage = 0.0f;
uint8_t Battery_Percentage = 0;
uint32_t Timer_Start_Millis = 0;
bool Dive_Timer_Started = false;
int Minutes = 0;
int Seconds = 0;
uint8_t Backlight_Level = Backlight_Normal;
bool Calibration_Armed = false;
uint32_t Calibration_Start_MS = 0;
uint8_t Button_Last_Reading = HIGH;
uint8_t Button_Stable_State = HIGH;
uint32_t Button_Last_Change_MS = 0;

// Compass Labels
struct CompassLabel {
  int degrees;
  const char *label;
};
const CompassLabel Compass_Labels[] = {
    {0, "N"},   {30, "30"},  {60, "60"},   {90, "E"},
    {120, "120"}, {150, "150"}, {180, "S"},   {210, "210"},
    {240, "240"}, {270, "W"},   {300, "300"}, {330, "330"}};


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

// Timer
void updateTimer(float Depth) {
  if (Depth >= Dive_Start_Depth && !Dive_Timer_Started) {
    Timer_Start_Millis = millis();
    Dive_Timer_Started = true;
  }
  if (Dive_Timer_Started) {
    const uint32_t elapsedMillis = millis() - Timer_Start_Millis;
    Minutes = static_cast<int>(elapsedMillis / 60000UL);
    Seconds = static_cast<int>((elapsedMillis % 60000UL) / 1000UL);
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

// Compass
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

bool calibrationButtonPressed() {
  const uint8_t reading = static_cast<uint8_t>(digitalRead(Calibration_Button_Pin));
  if (reading != Button_Last_Reading) {
    Button_Last_Change_MS = millis();
    Button_Last_Reading = reading;
  }

  if ((millis() - Button_Last_Change_MS) >= Button_Debounce_MS) {
    if (reading != Button_Stable_State) {
      Button_Stable_State = reading;
      if (Button_Stable_State == LOW) {
        return true;
      }
    }
  }
  return false;
}

// Display Update
void updateDisplay(float Depth, int Minutes, int Seconds, float Battery_Voltage) {
  Battery_Percentage = batteryPercentage(Battery_Voltage);

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

  // Draw depth block on top-left.
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

  // Draw battery indicator on top-right.
  const int16_t rightEdge = screenW - 8;
  drawBatteryIndicator(rightEdge, 10, Battery_Percentage);

  // Draw timer below battery indicator.
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
  sensor.setFluidDensity(1020);          // EN13319 density

  // QMI8658 and QMC5883L Initialisation
  compassInit(SDA_Pin, SCL_Pin);

  // Display Initialisation
  spi.begin(SCK_Pin, -1, MOSI_Pin, CS_Pin);
  pinMode(Calibration_Button_Pin, INPUT_PULLUP);
  pinMode(Backlight_Pin, OUTPUT);
  analogWrite(Backlight_Pin, Backlight_Normal);
  display.init(LCD_Width, LCD_Height, SPI_MODE3);
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);
  drawCentreText("Bottom", 70, 4, ST77XX_WHITE);
  drawCentreText("Timer", 126, 4, ST77XX_WHITE);
  esp_sleep_enable_timer_wakeup(Message_US);
  esp_light_sleep_start();
}


void loop() {
  if (calibrationButtonPressed() && !Calibration_Armed) {
    Calibration_Armed = true;
    Calibration_Start_MS = millis();
  }

  if (Calibration_Armed) {
    const uint32_t elapsed = millis() - Calibration_Start_MS;
    if (elapsed < Calibration_Delay_MS) {
      display.fillScreen(ST77XX_BLACK);
      drawCentreText("Compass", 70, 4, ST77XX_WHITE);
      drawCentreText("Calibration", 126, 4, ST77XX_WHITE);
      esp_sleep_enable_timer_wakeup(Loop_US);
      esp_light_sleep_start();
      return;
    }

    display.fillScreen(ST77XX_BLACK);
    drawCentreText("Calibrating", 98, 4, ST77XX_CYAN);
    compassCalibrate();

    display.fillScreen(ST77XX_BLACK);
    drawCentreText("Done", 98, 4, ST77XX_GREEN);
    esp_sleep_enable_timer_wakeup(Message_US);
    esp_light_sleep_start();
    Calibration_Armed = false;
  }

  sensor.read();
  Depth = sensor.depth() - Depth_Offset;  // Sea level offset
  if (Depth < 0.0f) Depth = 0.0f;         // Minimum depth 0 meter
  if (Depth > 99.9f) Depth = 99.9f;       // Maximum depth 99.9 meters

  updateTimer(Depth);

  Heading = readHeading();

  Battery_Voltage = readBattery();

  updateDisplay(Depth, Minutes, Seconds, Battery_Voltage);

  if (Battery_Voltage < Low_Battery_Threshold) {
    analogWrite(Backlight_Pin, Backlight_Low);
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

  esp_sleep_enable_timer_wakeup(Loop_US);
  esp_light_sleep_start();
}