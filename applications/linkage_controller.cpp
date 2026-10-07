#include "linkage_controller.hpp"

#include <cmath>

namespace
{
constexpr float POSITION_KP = 2.5f;
constexpr float SPEED_LIMIT_RADPS = 1.0f;
constexpr float SPEED_KP = 0.13f;
constexpr float SPEED_KI = 0.2f;
constexpr float INTEGRAL_LIMIT_A = 0.05f;
// Reset must overcome static load even when the remaining position error is small.
constexpr float RESET_SPEED_KI = 0.4f;
constexpr float RESET_INTEGRAL_LIMIT_A = 0.15f;
constexpr float SPEED_FILTER_ALPHA = 0.2f;
constexpr float SETTLED_ERROR_RAD = 0.008f;
constexpr float SETTLED_SPEED_RADPS = 0.05f;
constexpr float TARGET_SPEED_FILTER_ALPHA = app::LINKAGE_PERIOD_S / (0.04f + app::LINKAGE_PERIOD_S);
constexpr float SETTLED_TARGET_SPEED_RADPS = 0.01f;
constexpr float SETTLED_DWELL_S = 0.1f;
constexpr float MANUAL_ERROR_RAD = 0.025f;
constexpr float MANUAL_SPEED_RADPS = 0.08f;
constexpr float BOARD_STILL_RADPS = 0.05f;
constexpr float MANUAL_DETECTION_S = 0.04f;
constexpr float MANUAL_RELEASE_S = 0.3f;

bool moving_away(float angle, float target, float speed)
{
  const float error = angle - target;
  return std::abs(error) > MANUAL_ERROR_RAD && std::abs(speed) > MANUAL_SPEED_RADPS &&
         error * speed > 0;
}

float approach(float previous, float target, float maximum_step)
{
  const float delta = target - previous;
  if (delta > maximum_step) return previous + maximum_step;
  if (delta < -maximum_step) return previous - maximum_step;
  return target;
}
}  // namespace

namespace app
{
LinkageController::LinkageController()
: position_a_(LINKAGE_PERIOD_S, POSITION_KP, 0, 0, SPEED_LIMIT_RADPS, 0),
  position_b_(LINKAGE_PERIOD_S, POSITION_KP, 0, 0, 3.0f * SPEED_LIMIT_RADPS, 0),
  speed_a_(LINKAGE_PERIOD_S, SPEED_KP, SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, INTEGRAL_LIMIT_A),
  speed_b_(LINKAGE_PERIOD_S, SPEED_KP, SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, INTEGRAL_LIMIT_A),
  reset_speed_a_(
    LINKAGE_PERIOD_S, SPEED_KP, RESET_SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, RESET_INTEGRAL_LIMIT_A),
  reset_speed_b_(
    LINKAGE_PERIOD_S, SPEED_KP, RESET_SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, RESET_INTEGRAL_LIMIT_A)
{
}

void LinkageController::reset()
{
  active_ = false;
  reset_active_ = false;
  reset_settled_s_ = 0;
  manual_source_ = 0;
  detection_a_s_ = detection_b_s_ = release_s_ = 0;
  position_a_.clear();
  position_b_.clear();
  speed_a_.clear();
  speed_b_.clear();
  reset_speed_a_.clear();
  reset_speed_b_.clear();
  state_a_ = {};
  state_b_ = {};
}

float LinkageController::calculate_current(
  sp::PID & position, sp::PID & speed, LoopState & state, float target, float angle,
  float measured_speed, float motion_scale)
{
  state.filtered_speed += SPEED_FILTER_ALPHA * (measured_speed - state.filtered_speed);
  // Differentiate the final target so board and hand inputs share the same smoothing.
  // Initialize at the current target to avoid a derivative kick on entry or source release.
  const float target_step = state.target_initialized ? target - state.previous_target : 0;
  state.previous_target = target;
  state.target_initialized = true;
  const float raw_target_speed = sp::limit_max(target_step / LINKAGE_PERIOD_S, 6.0f);
  state.target_speed += TARGET_SPEED_FILTER_ALPHA * (raw_target_speed - state.target_speed);
  position.calc(target, angle);
  const float speed_limit = SPEED_LIMIT_RADPS * motion_scale;
  const float position_speed = sp::limit_max(position.out, speed_limit);
  const float requested_speed = sp::limit_max(position_speed + state.target_speed, speed_limit);
  state.speed_reference = approach(
    state.speed_reference, requested_speed,
    MOTOR_ACCELERATION_RADPS2 * motion_scale * LINKAGE_PERIOD_S);
  const bool settled_candidate = std::abs(target - angle) < SETTLED_ERROR_RAD &&
                                 std::abs(measured_speed) < SETTLED_SPEED_RADPS &&
                                 std::abs(state.target_speed) < SETTLED_TARGET_SPEED_RADPS &&
                                 std::abs(target_step) < 0.00001f;
  state.settled_s = settled_candidate ? state.settled_s + LINKAGE_PERIOD_S : 0;
  const bool settled = state.settled_s >= SETTLED_DWELL_S;
  if (settled) {
    // Remove residual integral torque once near the target; retain position correction outside it.
    speed.clear();
    state.speed_reference = 0;
  }
  else {
    // Freeze integration while the actuator cannot follow the requested current.
    const float integral_threshold = state.integral_paused ? 0 : MOTOR_CURRENT_LIMIT_A * 0.8f;
    speed.calc(state.speed_reference, state.filtered_speed, integral_threshold);
  }
  const float requested_current = speed.out;
  state.current =
    approach(state.current, requested_current, MOTOR_CURRENT_SLEW_A_PER_S * LINKAGE_PERIOD_S);
  state.integral_paused = std::abs(state.current - requested_current) > 0.00001f ||
                          std::abs(requested_current) >= MOTOR_CURRENT_LIMIT_A * 0.99f;
  return state.current;
}

LinkageOutput LinkageController::update(const LinkageInput & input)
{
  if (
    !input.enabled || !std::isfinite(input.yaw) || !std::isfinite(input.angle_a) ||
    !std::isfinite(input.angle_b) || !std::isfinite(input.speed_a) ||
    !std::isfinite(input.speed_b) || !std::isfinite(input.yaw_rate) ||
    (input.ratio_b != 0.5f && input.ratio_b != -1.0f && input.ratio_b != 3.0f)) {
    reset();
    return {};
  }
  if (input.reset_requested) {
    if (!std::isfinite(input.reset_direction_a) || !std::isfinite(input.reset_direction_b)) {
      reset();
      return {};
    }
    if (!reset_active_) {
      reset();
      reset_active_ = true;
      reset_input_a_ = input.reset_direction_a;
      reset_input_b_ = input.reset_direction_b;
      // Select a nearest equivalent revolution once; keep that branch until leaving UP.
      reset_target_a_ = input.angle_a + sp::limit_angle(input.reset_direction_a - input.angle_a);
      reset_target_b_ = input.angle_b + sp::limit_angle(input.reset_direction_b - input.angle_b);
      state_a_.filtered_speed = input.speed_a;
      state_b_.filtered_speed = input.speed_b;
    }
    const float target_a = reset_target_a_ + input.reset_direction_a - reset_input_a_;
    const float target_b = reset_target_b_ + input.reset_direction_b - reset_input_b_;
    const float current_a = calculate_current(
      position_a_, reset_speed_a_, state_a_, target_a, input.angle_a, input.speed_a);
    const float current_b = calculate_current(
      position_b_, reset_speed_b_, state_b_, target_b, input.angle_b, input.speed_b);
    const bool settled =
      std::abs(target_a - input.angle_a) < 0.015f && std::abs(target_b - input.angle_b) < 0.015f &&
      std::abs(input.speed_a) < SETTLED_SPEED_RADPS &&
      std::abs(input.speed_b) < SETTLED_SPEED_RADPS && std::abs(input.yaw_rate) < BOARD_STILL_RADPS;
    reset_settled_s_ = settled ? reset_settled_s_ + LINKAGE_PERIOD_S : 0;
    return {true, target_a, target_b, current_a, current_b, input.yaw, 0, reset_settled_s_ >= 0.2f};
  }
  // Entering MID or changing ratio starts from the current positions: no sudden return.
  if (!active_ || input.ratio_b != ratio_b_) {
    reset();
    active_ = true;
    origin_a_ = input.angle_a;
    origin_b_ = input.angle_b;
    yaw_origin_ = input.yaw;
    offset_ = 0;
    ratio_b_ = input.ratio_b;
    state_a_.filtered_speed = input.speed_a;
    state_b_.filtered_speed = input.speed_b;
  }
  float reference = input.yaw - yaw_origin_ + offset_;
  float target_a = origin_a_ + reference;
  float target_b = origin_b_ + ratio_b_ * reference;

  if (manual_source_ == 0 && std::abs(input.yaw_rate) < BOARD_STILL_RADPS) {
    detection_a_s_ =
      moving_away(input.angle_a, target_a, input.speed_a) ? detection_a_s_ + LINKAGE_PERIOD_S : 0;
    detection_b_s_ =
      moving_away(input.angle_b, target_b, input.speed_b) ? detection_b_s_ + LINKAGE_PERIOD_S : 0;
    if (detection_a_s_ >= MANUAL_DETECTION_S || detection_b_s_ >= MANUAL_DETECTION_S) {
      manual_source_ = detection_a_s_ >= MANUAL_DETECTION_S ? 1 : 2;
      release_s_ = 0;
      speed_a_.clear();
      speed_b_.clear();
    }
  }
  else if (manual_source_ == 0) {
    detection_a_s_ = detection_b_s_ = 0;
  }

  if (manual_source_ != 0) {
    reference =
      manual_source_ == 1 ? input.angle_a - origin_a_ : (input.angle_b - origin_b_) / ratio_b_;
    // This changes only the virtual linkage reference, never the measured IMU yaw.
    offset_ = reference - (input.yaw - yaw_origin_);
    target_a = origin_a_ + reference;
    target_b = origin_b_ + ratio_b_ * reference;
    const float source_speed = manual_source_ == 1 ? input.speed_a : input.speed_b;
    release_s_ = std::abs(source_speed) < MANUAL_SPEED_RADPS ? release_s_ + LINKAGE_PERIOD_S : 0;
    if (release_s_ >= MANUAL_RELEASE_S) {
      manual_source_ = 0;
      detection_a_s_ = detection_b_s_ = 0;
      speed_a_.clear();
      speed_b_.clear();
    }
  }

  float current_a =
    calculate_current(position_a_, speed_a_, state_a_, target_a, input.angle_a, input.speed_a);
  // A 1:3 destination needs three times the velocity and acceleration capacity.
  const float motion_scale_b = ratio_b_ == 3.0f ? 3.0f : 1.0f;
  float current_b = calculate_current(
    position_b_, speed_b_, state_b_, target_b, input.angle_b, input.speed_b, motion_scale_b);
  if (manual_source_ == 1) {
    speed_a_.clear();
    state_a_ = {0, 0, input.speed_a, false};
    current_a = 0;
  }
  if (manual_source_ == 2) {
    speed_b_.clear();
    state_b_ = {0, 0, input.speed_b, false};
    current_b = 0;
  }
  return {true, target_a, target_b, current_a, current_b, yaw_origin_ - offset_, manual_source_};
}
}  // namespace app
