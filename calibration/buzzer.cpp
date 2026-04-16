#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

constexpr uint8_t DC_Pin = 4;
constexpr uint8_t CS_Pin = 5;
constexpr uint8_t SCK_Pin = 6;
constexpr uint8_t MOSI_Pin = 7;
constexpr uint8_t RST_Pin = 8;
constexpr uint8_t Backlight_Pin = 15;
constexpr uint8_t Power_Pin = 41;
constexpr uint8_t Buzzer_Pin = 42;

constexpr uint16_t LCD_Width = 260;
constexpr uint16_t LCD_Height = 280;

constexpr uint16_t Start_Hz = 2000;
constexpr uint16_t End_Hz = 5000;
constexpr uint16_t Step_Hz = 100;
constexpr uint16_t Hold_MS = 500;

SPIClass spi(FSPI);
Adafruit_ST7789 display(&spi, CS_Pin, DC_Pin, RST_Pin);

void buzzerTest(Adafruit_ST7789 &display, uint16_t frequencyHz) {
    display.fillScreen(ST77XX_BLACK);
	display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
	display.setTextSize(6);
	display.setCursor(20, 100);
	display.print(frequencyHz);
	display.print(" Hz");
}

void setup() {
    pinMode(Power_Pin, OUTPUT);
    digitalWrite(Power_Pin, HIGH);
	pinMode(Buzzer_Pin, OUTPUT);
	digitalWrite(Buzzer_Pin, LOW);
    spi.begin(SCK_Pin, -1, MOSI_Pin, CS_Pin);
	pinMode(Backlight_Pin, OUTPUT);
	analogWrite(Backlight_Pin, 255);
	display.init(LCD_Width, LCD_Height, SPI_MODE3);
    display.setSPISpeed(40000000); 
	display.setRotation(1);
}

void loop() {
    for (uint16_t frequencyHz = Start_Hz;
			 frequencyHz <= End_Hz;
			 frequencyHz = static_cast<uint16_t>(frequencyHz + Step_Hz)) {
		display.fillScreen(ST77XX_BLACK);
	    display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
	    display.setTextSize(6);
	    display.setCursor(20, 100);
	    display.print(frequencyHz);
	    display.print(" Hz");
		tone(Buzzer_Pin, frequencyHz);
		delay(Hold_MS);
	}
    noTone(Buzzer_Pin);
}
