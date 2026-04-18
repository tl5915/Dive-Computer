#include <arduino.h>
#include "app_system.h"

#include "driver/temperature_sensor.h"

#include "esp_flash.h"
#include "esp_psram.h"

#include "../lvgl_ui/lvgl_ui.h"
#include "../../bsp_lv_port.h"

#include "../../bsp_spi.h"
#include "SD.h"
#include "SPI.h"

temperature_sensor_handle_t temp_sensor = NULL;

#define BATTERY_ADC_SIZE 9

// 排序函数（冒泡排序实现，可根据需要替换为其他排序算法）
static void bubble_sort(uint16_t *data, uint16_t size)
{
    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = 0; j < size - i - 1; j++)
        {
            if (data[j] > data[j + 1])
            {
                uint16_t temp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = temp;
            }
        }
    }
}

static uint16_t average_filter(uint16_t *samples)
{
    uint16_t out;
    bubble_sort(samples, BATTERY_ADC_SIZE);
    for (int i = 1; i < BATTERY_ADC_SIZE - 1; i++)
    {
        out += samples[i] / (BATTERY_ADC_SIZE - 2);
    }
    return out;
}

uint16_t bsp_battery_read_raw(void)
{
    uint16_t samples[BATTERY_ADC_SIZE];
    for (int i = 0; i < BATTERY_ADC_SIZE; i++)
    {
        samples[i] = analogRead(EXAMPLE_PIN_NUM_BAT_ADC);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return average_filter(samples); // 使用中位值滤波
}

void bsp_battery_read(float *voltage, uint16_t *adc_raw)
{
    uint16_t result = bsp_battery_read_raw();
    if (adc_raw)
    {
        *adc_raw = result;
    }
    if (voltage)
    {
        *voltage = (result * (3.3 / (1 << 12))) * 3.0;
    }
}



void app_system_init(void) {
  temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
  temperature_sensor_install(&temp_sensor_config, &temp_sensor);

  temperature_sensor_enable(temp_sensor);
}

void app_system_task(void *arg) {
  char str[20];
  float tsens_out;
  uint32_t flash_size;
  uint32_t psram_size;
  uint32_t sd_size;
  
  esp_flash_get_size(NULL, &flash_size);
  psram_size = esp_psram_get_size();


  if (bsp_spi_lock(-1)) {
    if (SD.begin(EXAMPLE_PIN_NUM_SD_CS, bsp_spi)) {
      uint8_t cardType = SD.cardType();
      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
      }

      Serial.print("SD Card Type: ");
      if (cardType == CARD_MMC) {
        Serial.println("MMC");
      } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
      } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
      } else {
        Serial.println("UNKNOWN");
      }
      sd_size = (uint32_t)(SD.cardSize() / (1024 * 1024));
    }
    bsp_spi_unlock();
  }
  if (lvgl_lock(-1)) {

    snprintf(str, sizeof(str), "%dM", flash_size / (uint32_t)(1024 * 1024));
    lv_label_set_text(label_flash, str);  // 初始值

    snprintf(str, sizeof(str), "%dM", psram_size / (uint32_t)(1024 * 1024));
    lv_label_set_text(label_psram, str);  // 初始值

    snprintf(str, sizeof(str), "%dM", sd_size);
    lv_label_set_text(label_sd, str);  // 初始值

    lvgl_unlock();
  }
  float voltage;
  while (1) {
    
    bsp_battery_read(&voltage, NULL);
    temperature_sensor_get_celsius(temp_sensor, &tsens_out);
    if (lvgl_lock(-1)) {
      snprintf(str, sizeof(str), "%.1fC", tsens_out);
      lv_label_set_text(label_chip_temp, str);  // 初始值

      snprintf(str, sizeof(str), "%.2fV", voltage);
      lv_label_set_text(label_battery, str);  // 初始值
      lvgl_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void app_system_run(void) {
  xTaskCreate(app_system_task, "system_task", 4096, NULL, 0, NULL);
}