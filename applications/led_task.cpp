#include "cmsis_os.h"
#include "gpio.h"

namespace
{
constexpr uint32_t LED_STEP_INTERVAL_MS = 200;
constexpr uint16_t LED_PINS[] = {GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_10};
}  // namespace

extern "C" void led_task(void const * argument)
{
  (void)argument;
  while (true) {
    for (uint16_t led_pin : LED_PINS) {
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOH, led_pin, GPIO_PIN_SET);
      osDelay(LED_STEP_INTERVAL_MS);
    }
  }
}
