#ifndef RESET_MAPPING_HPP
#define RESET_MAPPING_HPP

#include <cstdint>

#include "tools/math_tools/math_tools.hpp"

namespace app
{
// Raw encoder counts when each R mark points along the board's startup R direction.
// New startup pose is the latest user-aligned R direction: A=3310, B=3455.
// The capture's old yaw_delta=-17.69 deg is replaced by zero on loading at this pose.
// Use raw aligned counts directly for this new startup reference.
// Keep the board direction and stator orientations fixed through loading/initialization.
// Valid for the measured stator installation and board startup direction.
constexpr uint16_t MOTOR_A_R_ALIGNMENT_ENCODER = 3310;
constexpr uint16_t MOTOR_B_R_ALIGNMENT_ENCODER = 3455;
static_assert(MOTOR_A_R_ALIGNMENT_ENCODER < 8192 && MOTOR_B_R_ALIGNMENT_ENCODER < 8192);

inline float mapped_reset_direction(
  float board_yaw_delta, uint16_t alignment_encoder, float motor_direction)
{
  return board_yaw_delta + motor_direction * alignment_encoder * (2.0f * sp::SP_PI / 8192.0f);
}
}  // namespace app

#endif  // RESET_MAPPING_HPP
