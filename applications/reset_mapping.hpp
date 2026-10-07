#ifndef RESET_MAPPING_HPP
#define RESET_MAPPING_HPP

#include <cstdint>

#include "tools/math_tools/math_tools.hpp"

namespace app
{
// Encoder directions relative to the existing board startup reference.
// Correct-pose capture: yaw_delta=28.28 deg, A=5294, B=4370.
// Subtract yaw_delta * 8192 / 360: A=4650, B=3726 (rounded counts).
// Keep the existing board startup direction and measured stator orientations.
constexpr uint16_t MOTOR_A_R_ALIGNMENT_ENCODER = 4650;
constexpr uint16_t MOTOR_B_R_ALIGNMENT_ENCODER = 3726;
static_assert(MOTOR_A_R_ALIGNMENT_ENCODER < 8192 && MOTOR_B_R_ALIGNMENT_ENCODER < 8192);

inline float mapped_reset_direction(
  float board_yaw_delta, uint16_t alignment_encoder, float motor_direction)
{
  return board_yaw_delta + motor_direction * alignment_encoder * (2.0f * sp::SP_PI / 8192.0f);
}
}  // namespace app

#endif  // RESET_MAPPING_HPP
