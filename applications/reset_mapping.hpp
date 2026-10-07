#ifndef RESET_MAPPING_HPP
#define RESET_MAPPING_HPP

#include <cstdint>

#include "tools/math_tools/math_tools.hpp"

namespace app
{
// Raw encoder counts when each R mark points along the board's startup R direction.
// Latest confirmed aligned pose: yaw_delta=-64.12/-64.09 deg, A=4006, B=3061/3062.
// Mean yaw_delta=-64.105 deg; startup-direction offsets are A=5465, B=4520.
// Earlier aligned poses support equal signs but have inconsistent alignment offsets.
// Use the latest pose as the reference; verify physical alignment after loading.
// Subtract yaw_delta * 8192 / 360 and round to obtain the startup-direction offsets.
// Valid for the measured stator installation and board startup direction.
constexpr uint16_t MOTOR_A_R_ALIGNMENT_ENCODER = 5465;
constexpr uint16_t MOTOR_B_R_ALIGNMENT_ENCODER = 4520;
static_assert(MOTOR_A_R_ALIGNMENT_ENCODER < 8192 && MOTOR_B_R_ALIGNMENT_ENCODER < 8192);

inline float mapped_reset_direction(
  float board_yaw_delta, uint16_t alignment_encoder, float motor_direction)
{
  return board_yaw_delta + motor_direction * alignment_encoder * (2.0f * sp::SP_PI / 8192.0f);
}
}  // namespace app

#endif  // RESET_MAPPING_HPP
