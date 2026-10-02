#include <cstdio>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "usart.h"

namespace
{
constexpr float BOARD_TO_ROBOT[3][3] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}};
constexpr uint32_t IMU_SAMPLE_INTERVAL_MS = 10;
constexpr uint32_t PRINT_INTERVAL_MS = 100;

sp::BMI088 imu(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, BOARD_TO_ROBOT);
}  // namespace

extern "C" void imu_task(void const * argument)
{
  (void)argument;
  imu.init();
  uint32_t last_print_ms = 0;

  while (true) {
    imu.update();
    const uint32_t now_ms = osKernelSysTick();
    if (now_ms - last_print_ms >= PRINT_INTERVAL_MS) {
      char line[160];
      const int length = std::snprintf(
        line, sizeof(line), "acc_mps2=%.3f,%.3f,%.3f gyro_radps=%.3f,%.3f,%.3f\r\n",
        static_cast<double>(imu.acc[0]), static_cast<double>(imu.acc[1]),
        static_cast<double>(imu.acc[2]), static_cast<double>(imu.gyro[0]),
        static_cast<double>(imu.gyro[1]), static_cast<double>(imu.gyro[2]));
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
        HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(line), length, 20);
      }
      last_print_ms = now_ms;
    }
    osDelay(IMU_SAMPLE_INTERVAL_MS);
  }
}
