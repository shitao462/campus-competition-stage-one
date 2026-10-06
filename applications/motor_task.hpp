#ifndef MOTOR_TASK_HPP
#define MOTOR_TASK_HPP

#include <cstdint>

namespace app
{
struct MotorStatus
{
  bool output_disabled = true;
  bool right_switch_down;
  bool remote_alive;
  bool motor_a_alive;
  bool motor_b_alive;
  uint32_t transmit_failures;
  bool imu_ready;
  float yaw;
  float angle_a;
  float angle_b;
  float target_a;
  float target_b;
  float current_a;
  float current_b;
  float ratio_b;
  float reference_yaw;
  unsigned manual_source;
};

MotorStatus get_motor_status();
}  // namespace app

#endif  // MOTOR_TASK_HPP
