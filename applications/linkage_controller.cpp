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
constexpr float RESET_B_SPEED_KI = 0.2f;
constexpr float RESET_B_RESTART_ERROR_RAD = 0.015f;
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
// After a confirmed rest, identify a fresh hand movement before the position loop fights it.
constexpr float MANUAL_TAKEOVER_ERROR_RAD = 0.006f;
constexpr float MANUAL_TAKEOVER_SPEED_RADPS = 0.06f;
// Dedicated, gentler B loops for MID 1:3; the angular target still has ratio 3.
constexpr float B_RATIO_THREE_POSITION_KP = 2.2f;
constexpr float B_RATIO_THREE_SPEED_KI = 0.1f;
constexpr float B_RATIO_THREE_INTEGRAL_LIMIT_A = 0.03f;
constexpr float B_RATIO_THREE_MOTION_SCALE = 1.5f;
constexpr float B_RATIO_THREE_ACCELERATION_RADPS2 = 2.5f;

bool moving_away(
  float angle, float target, float speed, float error_gate = MANUAL_ERROR_RAD,
  float speed_gate = MANUAL_SPEED_RADPS)
{
  const float error = angle - target;
  return std::abs(error) > error_gate && std::abs(speed) > speed_gate && error * speed > 0;
}

float approach(float previous, float target, float maximum_step)
{
  const float delta = target - previous;
  if (delta > maximum_step) return previous + maximum_step;
  if (delta < -maximum_step) return previous - maximum_step;
  return target;
}

float wrap_direction(float angle)
{
  const float wrapped = std::remainder(angle, 2.0f * sp::SP_PI);
  return wrapped <= -sp::SP_PI ? wrapped + 2.0f * sp::SP_PI : wrapped;
}
}  // namespace

namespace app
{
LinkageController::LinkageController()
: position_a_(LINKAGE_PERIOD_S, POSITION_KP, 0, 0, SPEED_LIMIT_RADPS, 0),
  position_b_(LINKAGE_PERIOD_S, POSITION_KP, 0, 0, 3.0f * SPEED_LIMIT_RADPS, 0),
  position_b_three_(LINKAGE_PERIOD_S, B_RATIO_THREE_POSITION_KP, 0, 0, 3.0f * SPEED_LIMIT_RADPS, 0),
  speed_a_(LINKAGE_PERIOD_S, SPEED_KP, SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, INTEGRAL_LIMIT_A),
  speed_b_(LINKAGE_PERIOD_S, SPEED_KP, SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, INTEGRAL_LIMIT_A),
  speed_b_three_(
    LINKAGE_PERIOD_S, SPEED_KP, B_RATIO_THREE_SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A,
    B_RATIO_THREE_INTEGRAL_LIMIT_A),
  reset_speed_a_(
    LINKAGE_PERIOD_S, SPEED_KP, RESET_SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, RESET_INTEGRAL_LIMIT_A),
  reset_speed_b_(
    LINKAGE_PERIOD_S, SPEED_KP, RESET_B_SPEED_KI, 0, MOTOR_CURRENT_LIMIT_A, RESET_INTEGRAL_LIMIT_A)
{
}

void LinkageController::reset()
{
  active_ = false;
  reset_active_ = false;
  reset_settled_s_ = 0;
  manual_source_ = 0;
  manual_ready_a_ = manual_ready_b_ = false;
  detection_a_s_ = detection_b_s_ = release_s_ = 0;
  position_a_.clear();
  position_b_.clear();
  position_b_three_.clear();
  speed_a_.clear();
  speed_b_.clear();
  speed_b_three_.clear();
  reset_speed_a_.clear();
  reset_speed_b_.clear();
  state_a_ = {};
  state_b_ = {};
}

float LinkageController::calculate_current(
  sp::PID & position, sp::PID & speed, LoopState & state, float target, float angle,
  float measured_speed, float motion_scale, bool circular_target, bool stabilize_arrival,
  bool brake_on_crossing, bool stop_feedforward, bool target_stopped, float acceleration_limit)
{
  state.filtered_speed += SPEED_FILTER_ALPHA * (measured_speed - state.filtered_speed);
  // Differentiate the final target so board and hand inputs share the same smoothing.
  // Initialize at the current target to avoid a derivative kick on entry or source release.
  float target_step = state.target_initialized ? target - state.previous_target : 0;
  // Equivalent revolution changes in reset must not create a 2-pi feedforward kick.
  if (circular_target) target_step = wrap_direction(target_step);
  state.previous_target = target;
  state.target_initialized = true;
  const float raw_target_speed = sp::limit_max(target_step / LINKAGE_PERIOD_S, 6.0f);
  state.target_speed += TARGET_SPEED_FILTER_ALPHA * (raw_target_speed - state.target_speed);
  // Integer-RPM feedback can read zero during slow hand input. Confirm that the
  // final target has also stopped rather than repeatedly removing moving-goal feedforward.
  const bool target_still = target_stopped && std::abs(target_step) < 0.00001f;
  state.target_still_s = target_still ? state.target_still_s + LINKAGE_PERIOD_S : 0;
  if (stop_feedforward && state.target_still_s >= 0.1f) state.target_speed = 0;
  position.calc(target, angle);
  const float error = target - angle;
  // Discard old integral and catch-up speed when an axis crosses its goal.
  if (
    (stabilize_arrival || brake_on_crossing || stop_feedforward) &&
    error * state.previous_error < 0) {
    speed.clear();
    // A hand-input follower must not keep the approach speed after passing its goal.
    // Actual output remains subject to the current slew limit.
    if (brake_on_crossing || stop_feedforward)
      state.speed_reference = stop_feedforward ? state.target_speed : 0;
  }
  state.previous_error = error;
  const float speed_limit = SPEED_LIMIT_RADPS * motion_scale;
  const float position_speed = sp::limit_max(position.out, speed_limit);
  const float requested_speed = sp::limit_max(position_speed + state.target_speed, speed_limit);
  state.speed_reference =
    approach(state.speed_reference, requested_speed, acceleration_limit * LINKAGE_PERIOD_S);
  const bool settled_candidate = std::abs(target - angle) < SETTLED_ERROR_RAD &&
                                 std::abs(measured_speed) < SETTLED_SPEED_RADPS &&
                                 std::abs(state.target_speed) < SETTLED_TARGET_SPEED_RADPS &&
                                 (circular_target || std::abs(target_step) < 0.00001f);
  state.settled_s = settled_candidate ? state.settled_s + LINKAGE_PERIOD_S : 0;
  bool settled = state.settled_s >= SETTLED_DWELL_S;
  if (stabilize_arrival) {
    if (settled) state.arrival_latched = true;
    if (
      std::abs(error) >= RESET_B_RESTART_ERROR_RAD ||
      std::abs(measured_speed) >= SETTLED_SPEED_RADPS ||
      std::abs(state.target_speed) >= SETTLED_TARGET_SPEED_RADPS)
      state.arrival_latched = false;
    settled = state.arrival_latched;
  }
  if (settled) {
    // Remove residual integral torque once near the target; retain position correction outside it.
    speed.clear();
    state.speed_reference = 0;
  }
  else {
    // Freeze integration while the actuator cannot follow the requested current.
    const bool accelerating_reference =
      std::abs(state.speed_reference - requested_speed) > 0.00001f;
    const float integral_threshold =
      (state.integral_paused || accelerating_reference) ? 0 : MOTOR_CURRENT_LIMIT_A * 0.8f;
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
      state_a_.filtered_speed = input.speed_a;
      state_b_.filtered_speed = input.speed_b;
    }
    // UP tracks only the calibrated board direction, independent of MID ratio and hand history.
    // Encoder displacement is feedback: pushing either motor cannot redefine its target.
    const float target_a = input.angle_a + wrap_direction(input.reset_direction_a - input.angle_a);
    const float target_b = input.angle_b + wrap_direction(input.reset_direction_b - input.angle_b);
    if (input.control_paused) {
      reset_speed_a_.clear();
      reset_speed_b_.clear();
      speed_b_three_.clear();
      state_a_ = {};
      state_b_ = {};
      state_a_.filtered_speed = input.speed_a;
      state_b_.filtered_speed = input.speed_b;
      reset_settled_s_ = 0;
      return {true, target_a, target_b, 0, 0, input.yaw, 0, false};
    }
    const float current_a = calculate_current(
      position_a_, reset_speed_a_, state_a_, target_a, input.angle_a, input.speed_a, 1.0f, true);
    const float current_b = calculate_current(
      position_b_, reset_speed_b_, state_b_, target_b, input.angle_b, input.speed_b, 1.0f, true,
      true);
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
    board_detection_yaw_ = input.yaw;
    offset_ = 0;
    ratio_b_ = input.ratio_b;
    state_a_.filtered_speed = input.speed_a;
    state_b_.filtered_speed = input.speed_b;
  }
  float reference = input.yaw - yaw_origin_ + offset_;
  float target_a = origin_a_ + reference;
  float target_b = origin_b_ + ratio_b_ * reference;

  const bool board_moved = std::abs(input.yaw - board_detection_yaw_) > 0.005f;
  if (board_moved) board_detection_yaw_ = input.yaw;
  // Require actual board rotation as well as displacement. Stationary yaw drift must
  // not repeatedly cancel a hand source and clear the follower's speed integral.
  if (manual_source_ != 0 && board_moved && std::abs(input.yaw_rate) >= BOARD_STILL_RADPS) {
    manual_source_ = 0;
    detection_a_s_ = detection_b_s_ = release_s_ = 0;
    speed_a_.clear();
    speed_b_.clear();
    speed_b_three_.clear();
  }
  const bool board_still = std::abs(input.yaw_rate) < BOARD_STILL_RADPS;

  if (manual_source_ == 0 && board_still) {
    // Retain rest qualification while a fresh movement builds the small displacement gate.
    // A servo correction outside the arrival band or a moving target cancels it.
    if (state_a_.settled_s >= SETTLED_DWELL_S) manual_ready_a_ = true;
    if (state_b_.settled_s >= SETTLED_DWELL_S) manual_ready_b_ = true;
    if (
      std::abs(state_a_.target_speed) >= SETTLED_TARGET_SPEED_RADPS ||
      (std::abs(target_a - input.angle_a) >= SETTLED_ERROR_RAD &&
       (target_a - input.angle_a) * input.speed_a > 0))
      manual_ready_a_ = false;
    if (
      std::abs(state_b_.target_speed) >= SETTLED_TARGET_SPEED_RADPS ||
      (std::abs(target_b - input.angle_b) >= SETTLED_ERROR_RAD &&
       (target_b - input.angle_b) * input.speed_b > 0))
      manual_ready_b_ = false;
  }
  else {
    manual_ready_a_ = manual_ready_b_ = false;
  }
  if (
    (manual_source_ == 0 || release_s_ >= MANUAL_RELEASE_S) && board_still &&
    !input.control_paused) {
    const bool rested_a = manual_ready_a_;
    const bool rested_b = manual_ready_b_;
    detection_a_s_ =
      manual_source_ != 1 && moving_away(
                               input.angle_a, target_a, input.speed_a,
                               rested_a ? MANUAL_TAKEOVER_ERROR_RAD : MANUAL_ERROR_RAD,
                               rested_a ? MANUAL_TAKEOVER_SPEED_RADPS : MANUAL_SPEED_RADPS)
        ? detection_a_s_ + LINKAGE_PERIOD_S
        : 0;
    detection_b_s_ =
      manual_source_ != 2 && moving_away(
                               input.angle_b, target_b, input.speed_b,
                               rested_b ? MANUAL_TAKEOVER_ERROR_RAD : MANUAL_ERROR_RAD,
                               rested_b ? MANUAL_TAKEOVER_SPEED_RADPS : MANUAL_SPEED_RADPS)
        ? detection_b_s_ + LINKAGE_PERIOD_S
        : 0;
    if (detection_a_s_ >= MANUAL_DETECTION_S || detection_b_s_ >= MANUAL_DETECTION_S) {
      manual_source_ = detection_a_s_ >= MANUAL_DETECTION_S ? 1 : 2;
      manual_ready_a_ = manual_ready_b_ = false;
      release_s_ = 0;
      detection_a_s_ = detection_b_s_ = 0;
      speed_a_.clear();
      speed_b_.clear();
      speed_b_three_.clear();
    }
  }
  else {
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
  }

  if (input.control_paused) {
    // Release torque on a late tick without capturing new origins on recovery.
    position_a_.clear();
    position_b_.clear();
    position_b_three_.clear();
    speed_a_.clear();
    speed_b_.clear();
    speed_b_three_.clear();
    state_a_ = {};
    state_b_ = {};
    state_a_.filtered_speed = input.speed_a;
    state_b_.filtered_speed = input.speed_b;
    detection_a_s_ = detection_b_s_ = 0;
    manual_ready_a_ = manual_ready_b_ = false;
    return {true, target_a, target_b, 0, 0, yaw_origin_ - offset_, manual_source_};
  }
  const bool stop_feedforward = ratio_b_ == 3.0f;
  const float source_rate = manual_source_ == 1   ? input.speed_a
                            : manual_source_ == 2 ? input.speed_b / ratio_b_
                                                  : input.yaw_rate;
  const bool target_stopped = std::abs(source_rate) < 0.01f;
  float current_a = calculate_current(
    position_a_, speed_a_, state_a_, target_a, input.angle_a, input.speed_a, 1.0f, false, false,
    manual_source_ == 2 || ratio_b_ == 3.0f, stop_feedforward, target_stopped);
  const float motion_scale_b = ratio_b_ == 3.0f ? B_RATIO_THREE_MOTION_SCALE : 1.0f;
  float current_b = calculate_current(
    ratio_b_ == 3.0f ? position_b_three_ : position_b_,
    ratio_b_ == 3.0f ? speed_b_three_ : speed_b_, state_b_, target_b, input.angle_b, input.speed_b,
    motion_scale_b, false, false, manual_source_ == 1 || ratio_b_ == 3.0f, stop_feedforward,
    target_stopped,
    ratio_b_ == 3.0f ? B_RATIO_THREE_ACCELERATION_RADPS2 : MOTOR_ACCELERATION_RADPS2);
  // Release a hand candidate during confirmation rather than resisting for another 40 ms.
  // Targets are retained until confirmation; cancellation restarts the servo from zero current.
  if (manual_source_ == 1 || detection_a_s_ > 0) {
    speed_a_.clear();
    state_a_ = {0, 0, input.speed_a, false};
    current_a = 0;
  }
  if (manual_source_ == 2 || detection_b_s_ > 0) {
    speed_b_.clear();
    speed_b_three_.clear();
    state_b_ = {0, 0, input.speed_b, false};
    current_b = 0;
  }
  // Releasing the hand restores input detection without waiting for the follower to catch up.
  if (manual_source_ != 0 && release_s_ >= MANUAL_RELEASE_S) {
    manual_source_ = 0;
    release_s_ = 0;
  }
  return {true, target_a, target_b, current_a, current_b, yaw_origin_ - offset_, manual_source_};
}
}  // namespace app
