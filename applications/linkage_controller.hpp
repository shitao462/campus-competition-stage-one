#ifndef LINKAGE_CONTROLLER_HPP
#define LINKAGE_CONTROLLER_HPP

#include "tools/pid/pid.hpp"

namespace app
{
constexpr float LINKAGE_PERIOD_S = 0.005f;
constexpr float MOTOR_CURRENT_LIMIT_A = 0.25f;
constexpr float MOTOR_CURRENT_SLEW_A_PER_S = 0.8f;
constexpr float MOTOR_ACCELERATION_RADPS2 = 1.8f;

struct LinkageInput
{
  bool enabled;
  float yaw;
  float yaw_rate;
  float angle_a;
  float angle_b;
  float speed_a;
  float speed_b;
  float ratio_b;
  bool reset_requested = false;
  float reset_direction_a = 0;
  float reset_direction_b = 0;
};

struct LinkageOutput
{
  bool enabled;
  float target_a;
  float target_b;
  float current_a;
  float current_b;
  float reference_yaw;
  unsigned manual_source;  // 0: board, 1: motor A, 2: motor B.
  bool reset_complete = false;
};

class LinkageController
{
public:
  LinkageController();
  LinkageOutput update(const LinkageInput & input);
  void reset();

private:
  struct LoopState
  {
    float speed_reference = 0;
    float current = 0;
    float filtered_speed = 0;
    bool integral_paused = false;
    bool target_initialized = false;
    float previous_target = 0;
    float target_speed = 0;
    float settled_s = 0;
    float previous_error = 0;
    bool arrival_latched = false;
  };
  float calculate_current(
    sp::PID & position, sp::PID & speed, LoopState & state, float target, float angle,
    float measured_speed, float motion_scale = 1.0f, bool circular_target = false,
    bool stabilize_arrival = false);
  LoopState state_a_;
  LoopState state_b_;
  sp::PID position_a_;
  sp::PID position_b_;
  sp::PID speed_a_;
  sp::PID speed_b_;
  sp::PID reset_speed_a_;
  sp::PID reset_speed_b_;
  bool active_ = false;
  bool reset_active_ = false;
  float reset_settled_s_ = 0;
  float origin_a_ = 0;
  float origin_b_ = 0;
  float yaw_origin_ = 0;
  float offset_ = 0;
  float ratio_b_ = 1;
  float detection_a_s_ = 0;
  float detection_b_s_ = 0;
  float release_s_ = 0;
  unsigned manual_source_ = 0;
};
}  // namespace app

#endif  // LINKAGE_CONTROLLER_HPP
