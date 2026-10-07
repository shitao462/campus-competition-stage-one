#include "imu_task.hpp"

#include <cmath>
#include <cstdio>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "motor_task.hpp"
#include "task.h"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usart.h"

namespace
{
constexpr float BOARD_TO_ROBOT[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
constexpr uint32_t IMU_SAMPLE_INTERVAL_MS = 10;
constexpr uint32_t PRINT_INTERVAL_MS = 100;
constexpr uint32_t CALIBRATION_SAMPLES = 200;
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 1000;
constexpr float RAD_TO_DEG = 180.0f / sp::SP_PI;

app::ImuStatus imu_status = {};

sp::BMI088 imu(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, BOARD_TO_ROBOT);
}  // namespace

namespace app
{
ImuStatus get_imu_status()
{
  taskENTER_CRITICAL();
  const ImuStatus status = imu_status;
  taskEXIT_CRITICAL();
  return status;
}
}  // namespace app

extern "C" void imu_task(void const * argument)
{
  (void)argument;
  imu.init();
  uint32_t last_print_ms = 0;
  bool print_control_trace = false;
  uint32_t last_status_print_ms = 0;
  uint32_t calibration_samples = 0;
  float gyro_bias[3] = {};
  sp::Mahony attitude(IMU_SAMPLE_INTERVAL_MS * 0.001f);
  sp::AngleUnwrapper yaw_unwrapper;
  bool attitude_initialized = false;
  uint32_t last_attitude_ms = osKernelSysTick();
  float last_yaw = 0;

  while (true) {
    imu.update();
    const uint32_t now_ms = osKernelSysTick();
    const float gravity =
      std::sqrt(imu.acc[0] * imu.acc[0] + imu.acc[1] * imu.acc[1] + imu.acc[2] * imu.acc[2]);
    const bool valid = std::isfinite(gravity) && gravity > 7.0f && gravity < 12.0f &&
                       std::isfinite(imu.gyro[0]) && std::isfinite(imu.gyro[1]) &&
                       std::isfinite(imu.gyro[2]);
    app::ImuStatus sample = {false, last_yaw, 0, now_ms};
    if (calibration_samples < CALIBRATION_SAMPLES) {
      const bool still = valid && std::abs(imu.gyro[0]) < 0.08f && std::abs(imu.gyro[1]) < 0.08f &&
                         std::abs(imu.gyro[2]) < 0.08f;
      if (!still) {
        calibration_samples = 0;
        for (float & bias : gyro_bias) bias = 0;
      }
      else {
        for (unsigned axis = 0; axis < 3; ++axis) gyro_bias[axis] += imu.gyro[axis];
        if (++calibration_samples == CALIBRATION_SAMPLES) {
          for (float & bias : gyro_bias) bias /= CALIBRATION_SAMPLES;
        }
      }
      last_attitude_ms = now_ms;
    }
    else if (valid && now_ms - last_attitude_ms <= 50) {
      float corrected_gyro[3];
      for (unsigned axis = 0; axis < 3; ++axis) {
        corrected_gyro[axis] = imu.gyro[axis] - gyro_bias[axis];
      }
      if (!attitude_initialized) {
        attitude.update(imu.acc, corrected_gyro);
        attitude_initialized = true;
      }
      const uint32_t steps = (now_ms - last_attitude_ms) / IMU_SAMPLE_INTERVAL_MS;
      const float previous_yaw = last_yaw;
      for (uint32_t step = 0; step < steps; ++step) {
        attitude.update(imu.acc, corrected_gyro);
        last_yaw = yaw_unwrapper.update(attitude.yaw);
      }
      last_attitude_ms += steps * IMU_SAMPLE_INTERVAL_MS;
      if (steps > 0) {
        sample = {
          std::isfinite(last_yaw) && std::abs(attitude.pitch) < 1.2f, last_yaw,
          (last_yaw - previous_yaw) / (steps * 0.01f), now_ms};
      }
      else {
        sample = app::get_imu_status();
      }
    }
    else {
      last_attitude_ms = now_ms;
    }
    taskENTER_CRITICAL();
    imu_status = sample;
    taskEXIT_CRITICAL();
    if (now_ms - last_print_ms >= PRINT_INTERVAL_MS) {
      // Alternate raw IMU and control trace rather than adding UART work to the sample loop.
      // Each remains 5 Hz; the IMU: three-angle plot below stays at 10 Hz.
      print_control_trace = !print_control_trace;
      if (print_control_trace) {
        const app::MotorStatus status = app::get_motor_status();
        char trace[160];
        const int trace_length = std::snprintf(
          trace, sizeof(trace), "CTRL:%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%u,%u,%u,%lu,%lu\r\n",
          static_cast<unsigned long>(now_ms), static_cast<double>(status.angle_a * RAD_TO_DEG),
          static_cast<double>(status.angle_b * RAD_TO_DEG),
          static_cast<double>(status.target_b * RAD_TO_DEG), static_cast<double>(status.speed_b),
          static_cast<double>(status.current_b), status.manual_source,
          static_cast<unsigned>(status.output_disabled), static_cast<unsigned>(status.reset_mode),
          static_cast<unsigned long>(status.late_commands),
          static_cast<unsigned long>(status.transmit_failures));
        if (trace_length > 0 && trace_length < static_cast<int>(sizeof(trace)))
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(trace), trace_length, 20);
      }
      else {
        char line[160];
        const int length = std::snprintf(
          line, sizeof(line), "acc_mps2=%.3f,%.3f,%.3f gyro_radps=%.3f,%.3f,%.3f\r\n",
          static_cast<double>(imu.acc[0]), static_cast<double>(imu.acc[1]),
          static_cast<double>(imu.acc[2]), static_cast<double>(imu.gyro[0]),
          static_cast<double>(imu.gyro[1]), static_cast<double>(imu.gyro[2]));
        if (length > 0 && length < static_cast<int>(sizeof(line))) {
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(line), length, 20);
        }
      }
      // SerialPlot includes only this prefix; existing diagnostic lines stay readable.
      // Wait for initialized attitude fields and use the same relative yaw as reset telemetry.
      if (attitude_initialized && sample.ready) {
        const app::MotorStatus status = app::get_motor_status();
        if (
          status.imu_ready && std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) &&
          std::isfinite(status.board_yaw_delta)) {
          char plot_line[80];
          const int plot_length = std::snprintf(
            plot_line, sizeof(plot_line), "IMU:%.3f,%.3f,%.3f\r\n",
            static_cast<double>(attitude.roll * RAD_TO_DEG),
            static_cast<double>(attitude.pitch * RAD_TO_DEG),
            static_cast<double>(status.board_yaw_delta * RAD_TO_DEG));
          if (plot_length > 0 && plot_length < static_cast<int>(sizeof(plot_line))) {
            HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(plot_line), plot_length, 20);
          }
        }
      }
      last_print_ms = now_ms;
    }
    if (now_ms - last_status_print_ms >= STATUS_PRINT_INTERVAL_MS) {
      const app::MotorStatus status = app::get_motor_status();
      char line[160];
      const int length = std::snprintf(
        line, sizeof(line),
        "motor_disabled=%u down=%u rc_alive=%u motor_a_alive=%u motor_b_alive=%u "
        "tx_failures=%lu\r\n",
        static_cast<unsigned>(status.output_disabled),
        static_cast<unsigned>(status.right_switch_down), static_cast<unsigned>(status.remote_alive),
        static_cast<unsigned>(status.motor_a_alive), static_cast<unsigned>(status.motor_b_alive),
        static_cast<unsigned long>(status.transmit_failures));
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
        HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(line), length, 20);
      }
      char linkage_line[256];
      const int linkage_length = std::snprintf(
        linkage_line, sizeof(linkage_line),
        "imu_ready=%u ratio_b=%.1f yaw_rel_deg=%.2f enc=%u,%u error_deg=%.2f,%.2f "
        "current_A=%.3f,%.3f manual=%u reset=%u done=%u\r\n",
        static_cast<unsigned>(status.imu_ready), static_cast<double>(status.ratio_b),
        static_cast<double>(status.board_yaw_delta * RAD_TO_DEG),
        static_cast<unsigned>(status.encoder_a), static_cast<unsigned>(status.encoder_b),
        static_cast<double>((status.target_a - status.angle_a) * RAD_TO_DEG),
        static_cast<double>((status.target_b - status.angle_b) * RAD_TO_DEG),
        static_cast<double>(status.current_a), static_cast<double>(status.current_b),
        status.manual_source, static_cast<unsigned>(status.reset_mode),
        static_cast<unsigned>(status.reset_complete));
      if (linkage_length > 0 && linkage_length < static_cast<int>(sizeof(linkage_line))) {
        HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(linkage_line), linkage_length, 40);
      }
      last_status_print_ms = now_ms;
    }
    osDelay(IMU_SAMPLE_INTERVAL_MS);
  }
}
