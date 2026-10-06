#ifndef MOTOR_TASK_HPP
#define MOTOR_TASK_HPP

#include <cstdint>

namespace app
{
struct MotorStatus
{
  bool output_disabled;
  bool right_switch_down;
  bool remote_alive;
  bool motor_a_alive;
  bool motor_b_alive;
  uint32_t transmit_failures;
};

MotorStatus get_motor_status();
}  // namespace app

#endif  // MOTOR_TASK_HPP
