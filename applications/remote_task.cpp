#include "remote_task.hpp"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "task.h"

namespace
{
sp::DBus remote(&huart3, false);
volatile bool remote_link_alive = false;
}  // namespace

namespace app
{
RemoteStatus get_remote_status()
{
  taskENTER_CRITICAL();
  const bool alive = remote.is_alive(osKernelSysTick());
  const bool right_switch_down = alive && remote.sw_r == sp::DBusSwitchMode::DOWN;
  const bool linkage_enabled = alive && remote.sw_r == sp::DBusSwitchMode::MID;
  const bool reset_requested = alive && remote.sw_r == sp::DBusSwitchMode::UP;
  const float motor_b_ratio = remote.sw_l == sp::DBusSwitchMode::DOWN
                                ? 0.5f
                                : (remote.sw_l == sp::DBusSwitchMode::MID ? -1.0f : 3.0f);
  taskEXIT_CRITICAL();
  return {alive, right_switch_down, linkage_enabled, motor_b_ratio, reset_requested};
}
}  // namespace app

extern "C" void remote_task(void const * argument)
{
  (void)argument;
  remote.request();

  while (true) {
    // Watch remote_link_alive and remote in the debugger to verify controls.
    remote_link_alive = remote.is_alive(osKernelSysTick());
    osDelay(20);
  }
}

extern "C" void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * uart, uint16_t size)
{
  if (uart == &huart3) {
    remote.update(size, osKernelSysTick());
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * uart)
{
  if (uart == &huart3) remote.request();
}
