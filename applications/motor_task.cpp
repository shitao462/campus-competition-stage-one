#include "motor_task.hpp"

#include <cmath>

#include "FreeRTOS.h"
#include "board_io.h"
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "linkage_controller.hpp"
#include "main.h"
#include "remote_task.hpp"
#include "task.h"

namespace
{
constexpr uint32_t MOTOR_TASK_INTERVAL_MS = 1;
constexpr uint32_t MOTOR_COMMAND_INTERVAL_MS = 5;
constexpr uint32_t MOTOR_FEEDBACK_TIMEOUT_MS = 100;
// Orange steady LEDs with voltage commands indicate current-control mode.
// Both motors must use the same configured control mode.
constexpr bool MOTOR_CURRENT_CONTROL = true;
constexpr uint32_t MOTOR_COMMAND_IDS[] = {
  MOTOR_CURRENT_CONTROL ? 0x1FEu : 0x1FFu, MOTOR_CURRENT_CONTROL ? 0x2FEu : 0x2FFu};
constexpr uint32_t MOTOR_A_FEEDBACK_ID = 0x205;
constexpr uint32_t MOTOR_B_FEEDBACK_ID = 0x206;
constexpr uint32_t MOTOR_FEEDBACK_FIFOS[] = {CAN_RX_FIFO0, CAN_RX_FIFO1};
constexpr float ENCODER_TO_RAD = 2.0f * sp::SP_PI / 8192.0f;
constexpr float RPM_TO_RADPS = 2.0f * sp::SP_PI / 60.0f;
// Viewed from the output shaft: encoder CCW is positive, as is board yaw.
constexpr float MOTOR_A_DIRECTION = 1.0f;
constexpr float MOTOR_B_DIRECTION = 1.0f;
constexpr uint32_t IMU_TIMEOUT_MS = 50;
constexpr uint8_t MOTOR_TEMPERATURE_LIMIT_C = 80;

struct MotorFeedback
{
  sp::AngleUnwrapper unwrapper;
  bool received = false;
  uint32_t stamp_ms = 0;
  float angle = 0;
  float speed = 0;
  uint8_t temperature_c = 0;
};

app::MotorStatus motor_status = {};

bool send_current_command(uint32_t identifier, float current_a, float current_b)
{
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return false;
  CAN_TxHeaderTypeDef header = {};
  header.StdId = identifier;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  uint8_t data[8] = {};
  const int16_t commands[] = {
    static_cast<int16_t>(current_a * MOTOR_A_DIRECTION * 16384.0f / 3.0f),
    static_cast<int16_t>(current_b * MOTOR_B_DIRECTION * 16384.0f / 3.0f)};
  for (unsigned index = 0; index < 2; ++index) {
    const uint16_t command = static_cast<uint16_t>(commands[index]);
    data[index * 2] = command >> 8;
    data[index * 2 + 1] = command & 0xFF;
  }
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

  MotorFeedback feedback_a;
  MotorFeedback feedback_b;
  app::LinkageController controller;
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
        MotorFeedback * feedback =
          header.StdId == MOTOR_A_FEEDBACK_ID
            ? &feedback_a
            : (header.StdId == MOTOR_B_FEEDBACK_ID ? &feedback_b : nullptr);
        if (feedback != nullptr) {
          const uint16_t encoder = (static_cast<uint16_t>(data[0]) << 8) | data[1];
          if (encoder > 8191) continue;
          const float direction = feedback == &feedback_a ? MOTOR_A_DIRECTION : MOTOR_B_DIRECTION;
          const int16_t speed_rpm =
            static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
          if (!feedback->received || now_ms - feedback->stamp_ms >= MOTOR_FEEDBACK_TIMEOUT_MS) {
            feedback->unwrapper.reset();
          }
          feedback->angle = direction * feedback->unwrapper.update(encoder * ENCODER_TO_RAD);
          feedback->speed = direction * speed_rpm * RPM_TO_RADPS;
          feedback->temperature_c = data[6];
          feedback->received = true;
          feedback->stamp_ms = now_ms;
        }
      }
    }

    const app::RemoteStatus remote_status = app::get_remote_status();
    const app::ImuStatus imu_status = app::get_imu_status();
    const bool alive_a =
      feedback_a.received && now_ms - feedback_a.stamp_ms < MOTOR_FEEDBACK_TIMEOUT_MS;
    const bool alive_b =
      feedback_b.received && now_ms - feedback_b.stamp_ms < MOTOR_FEEDBACK_TIMEOUT_MS;
    const bool imu_ready = imu_status.ready && now_ms - imu_status.stamp_ms < IMU_TIMEOUT_MS;
    const bool enabled = MOTOR_CURRENT_CONTROL && remote_status.linkage_enabled && alive_a &&
                         alive_b && imu_ready &&
                         feedback_a.temperature_c < MOTOR_TEMPERATURE_LIMIT_C &&
                         feedback_b.temperature_c < MOTOR_TEMPERATURE_LIMIT_C;
    if (!enabled) controller.reset();
    if (now_ms - last_command_ms >= MOTOR_COMMAND_INTERVAL_MS) {
      // A missed control deadline releases output and captures a fresh origin on recovery.
      const bool on_time = now_ms - last_command_ms <= 2 * MOTOR_COMMAND_INTERVAL_MS;
      const app::LinkageOutput output = controller.update(
        {enabled && on_time, imu_status.yaw, imu_status.yaw_rate, feedback_a.angle,
         feedback_b.angle, feedback_a.speed, feedback_b.speed, remote_status.motor_b_ratio});
      if (!send_current_command(MOTOR_COMMAND_IDS[0], output.current_a, output.current_b)) {
        ++transmit_failures;
      }
      if (!send_current_command(MOTOR_COMMAND_IDS[1], 0, 0)) ++transmit_failures;
      last_command_ms = now_ms;
      taskENTER_CRITICAL();
      motor_status = {
        !output.enabled,
        remote_status.right_switch_down,
        remote_status.alive,
        alive_a,
        alive_b,
        transmit_failures,
        imu_ready,
        imu_status.yaw,
        feedback_a.angle,
        feedback_b.angle,
        output.target_a,
        output.target_b,
        output.current_a,
        output.current_b,
        remote_status.motor_b_ratio,
        output.reference_yaw,
        output.manual_source};
      taskEXIT_CRITICAL();
    }
    osDelay(MOTOR_TASK_INTERVAL_MS);
  }
}
