#include "motor_task.hpp"

#include "board_io.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"
#include "remote_task.hpp"
#include "task.h"

namespace
{
constexpr uint32_t MOTOR_TASK_INTERVAL_MS = 1;
constexpr uint32_t MOTOR_COMMAND_INTERVAL_MS = 10;
constexpr uint32_t MOTOR_FEEDBACK_TIMEOUT_MS = 100;
// Orange steady LEDs with voltage commands indicate current-control mode.
// Both motors must use the same configured control mode.
constexpr bool MOTOR_CURRENT_CONTROL = true;
constexpr uint32_t MOTOR_COMMAND_IDS[] = {
  MOTOR_CURRENT_CONTROL ? 0x1FEu : 0x1FFu,
  MOTOR_CURRENT_CONTROL ? 0x2FEu : 0x2FFu};
constexpr uint32_t MOTOR_A_FEEDBACK_ID = 0x205;
constexpr uint32_t MOTOR_B_FEEDBACK_ID = 0x206;
constexpr uint32_t MOTOR_FEEDBACK_FIFOS[] = {CAN_RX_FIFO0, CAN_RX_FIFO1};

app::MotorStatus motor_status = {true, false, false, false, false, 0};

bool send_zero_command(uint32_t identifier)
{
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return false;
  CAN_TxHeaderTypeDef header = {};
  header.StdId = identifier;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  uint8_t data[8] = {};
  uint32_t mailbox;
  return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK;
}
}  // namespace

namespace app
{
MotorStatus get_motor_status()
{
  taskENTER_CRITICAL();
  const MotorStatus status = motor_status;
  taskEXIT_CRITICAL();
  return status;
}
}  // namespace app

extern "C" void motor_task(void const * argument)
{
  (void)argument;
  CAN_FilterTypeDef filter = {};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = MOTOR_A_FEEDBACK_ID << 5;
  filter.FilterMaskIdHigh = 0xFFE0;
  filter.FilterMaskIdLow = 0x0006;  // Match standard data frames.
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) Error_Handler();
  filter.FilterBank = 1;
  filter.FilterIdHigh = MOTOR_B_FEEDBACK_ID << 5;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK || HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  uint32_t last_a_feedback_ms = 0;
  uint32_t last_b_feedback_ms = 0;
  bool has_a_feedback = false;
  bool has_b_feedback = false;
  uint32_t transmit_failures = 0;
  uint32_t last_command_ms = osKernelSysTick() - MOTOR_COMMAND_INTERVAL_MS;

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    // Separate FIFOs prevent one motor's traffic from replacing the other's feedback.
    for (uint32_t fifo : MOTOR_FEEDBACK_FIFOS) {
      for (uint8_t frame_index = 0; frame_index < 16; ++frame_index) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, fifo) == 0) break;
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(&hcan1, fifo, &header, data) != HAL_OK) break;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) {
          continue;
        }
        if (header.StdId == MOTOR_A_FEEDBACK_ID) {
          has_a_feedback = true;
          last_a_feedback_ms = now_ms;
        }
        if (header.StdId == MOTOR_B_FEEDBACK_ID) {
          has_b_feedback = true;
          last_b_feedback_ms = now_ms;
        }
      }
    }

    const app::RemoteStatus remote_status = app::get_remote_status();
    // DOWN and link loss require zero output. Other modes remain disabled until implemented.
    if (now_ms - last_command_ms >= MOTOR_COMMAND_INTERVAL_MS) {
      for (uint32_t identifier : MOTOR_COMMAND_IDS) {
        if (!send_zero_command(identifier)) ++transmit_failures;
      }
      last_command_ms = now_ms;
    }

    taskENTER_CRITICAL();
    motor_status = {
      true, remote_status.right_switch_down, remote_status.alive,
      has_a_feedback && now_ms - last_a_feedback_ms < MOTOR_FEEDBACK_TIMEOUT_MS,
      has_b_feedback && now_ms - last_b_feedback_ms < MOTOR_FEEDBACK_TIMEOUT_MS,
      transmit_failures};
    taskEXIT_CRITICAL();
    osDelay(MOTOR_TASK_INTERVAL_MS);
  }
}
