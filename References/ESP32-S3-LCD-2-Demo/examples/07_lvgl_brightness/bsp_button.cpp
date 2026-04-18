#include "bsp_button.h"
#include "OneButton.h"

#define PIN_INPUT 0

OneButton button(PIN_INPUT, true);

uint32_t button_value = 0;

static void click(void) {
  button_value = 1;
  Serial.println("click");
}  // do

static void doubleClick(void) {
  button_value = 2;
  Serial.println("doubleClick");
}  // do

static void longPressStart(void) {
  button_value = 2;
  Serial.println("longPressStart");
}  // do

static void longPressStop(void) {
  Serial.println("longPressStop");
}  // do


static void task(void *param)
{
  while(1)
  {
    button.tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}



uint32_t bsp_button_read(void)
{
  uint32_t value = button_value;
  button_value = 0;
  return value;
}

void bsp_button_init(void) {
  button.attachClick(click);
  button.attachLongPressStart(longPressStart);
  button.attachLongPressStop(longPressStop);
  xTaskCreate(task, "bsp_button_task", 1024, NULL, 5, NULL);
}