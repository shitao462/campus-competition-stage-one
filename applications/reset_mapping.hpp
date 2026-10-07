#ifndef RESET_MAPPING_HPP
#define RESET_MAPPING_HPP

#include <cstdint>

#include "tools/math_tools/math_tools.hpp"

namespace app
{
// Raw encoder counts when each R mark points along the board's startup R direction.
// Zero is a trial hypothesis, not a manufacturer-specified R-mark alignment.
// Replace these independent constants with measured values for the fixed installation.
constexpr uint16_t MOTOR_A_R_ALIGNMENT_ENCODER = 0;
constexpr uint16_t MOTOR_B_R_ALIGNMENT_ENCODER = 0;
static_assert(MOTOR_A_R_ALIGNMENT_ENCODER < 8192 && MOTOR_B_R_ALIGNMENT_ENCODER < 8192);

inline float mapped_reset_direction(
  float board_yaw_delta, uint16_t alignment_encoder, float motor_direction)
{
  return board_yaw_delta + motor_direction * alignment_encoder * (2.0f * sp::SP_PI / 8192.0f);
}
}  // namespace app

#endif  // RESET_MAPPING_HPP
