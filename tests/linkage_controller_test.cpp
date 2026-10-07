#include "linkage_controller.hpp"

#include <cmath>
#include <initializer_list>

#include "reset_mapping.hpp"
#include "tools/mahony/mahony.hpp"

namespace
{
bool near(float actual, float expected) { return std::abs(actual - expected) < 0.002f; }
}  // namespace

// Hardware-independent tests; a nonzero return value identifies the failed scenario.
extern "C" int run_tests()
{
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, ratio};
    controller.update(input);
    input.yaw = sp::SP_PI / 3;
    input.yaw_rate = 0.5f;
    auto output = controller.update(input);
    if (!near(output.target_a, input.yaw) || !near(output.target_b, ratio * input.yaw)) {
      return 1;
    }
    if (
      std::abs(output.current_a) > app::MOTOR_CURRENT_LIMIT_A ||
      std::abs(output.current_b) > app::MOTOR_CURRENT_LIMIT_A)
      return 2;
    input.enabled = false;
    output = controller.update(input);
    if (output.enabled || output.current_a != 0 || output.current_b != 0) return 3;
  }

  for (unsigned source : {1u, 2u}) {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, -1};
    controller.update(input);
    for (unsigned step = 0; step < 50; ++step) {
      if (source == 1) {
        input.angle_a += 0.005f;
        input.speed_a = 1;
      }
      else {
        input.angle_b += 0.005f;
        input.speed_b = 1;
      }
      controller.update(input);
    }
    auto output = controller.update(input);
    if (output.manual_source != source) return 4;
    if (!near(output.target_b, -output.target_a)) return 5;
    if ((source == 1 ? output.current_a : output.current_b) != 0) return 6;
    input.angle_a = output.target_a;
    input.angle_b = output.target_b;
    input.speed_a = input.speed_b = 0;
    for (unsigned step = 0; step < 160; ++step) output = controller.update(input);
    const float retained_a = output.target_a;
    if (output.manual_source != 0 || !near(output.target_a, input.angle_a)) return 7;
    input.yaw += 0.2f;
    input.yaw_rate = 0.5f;
    output = controller.update(input);
    if (!near(output.target_a, retained_a + 0.2f)) return 8;
    input.ratio_b = 3;
    output = controller.update(input);
    if (!near(output.target_a, input.angle_a) || !near(output.target_b, input.angle_b)) return 9;
  }

  sp::AngleUnwrapper encoder;
  const float before = encoder.update(6.27f);
  const float after = encoder.update(0.01f);
  if (!near(after - before, 2 * sp::SP_PI + 0.01f - 6.27f)) return 10;
  sp::Mahony attitude(0.01f);
  attitude.update(0, 0, 9.8f, 0, 0, 1);
  for (unsigned step = 0; step < 100; ++step) attitude.update(0, 0, 9.8f, 0, 0, 1);
  if (!near(attitude.yaw, 1)) return 11;
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3};
    controller.update(input);
    float previous_current_a = 0;
    float previous_current_b = 0;
    for (unsigned step = 0; step < 600; ++step) {
      input.yaw = step < 300 ? 1 : -1;
      input.yaw_rate = 0.5f;
      input.speed_a = step < 300 ? 20 : -20;
      input.speed_b = input.speed_a;
      const auto output = controller.update(input);
      const float allowed_step = app::MOTOR_CURRENT_SLEW_A_PER_S * app::LINKAGE_PERIOD_S;
      if (
        std::abs(output.current_a - previous_current_a) > allowed_step + 0.00001f ||
        std::abs(output.current_b - previous_current_b) > allowed_step + 0.00001f)
        return 12;
      if (
        std::abs(output.current_a) > app::MOTOR_CURRENT_LIMIT_A + 0.00001f ||
        std::abs(output.current_b) > app::MOTOR_CURRENT_LIMIT_A + 0.00001f)
        return 13;
      previous_current_a = output.current_a;
      previous_current_b = output.current_b;
    }
    input.angle_a = -1;
    input.angle_b = -3;
    input.speed_a = input.speed_b = 0;
    app::LinkageOutput output;
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    if (output.current_a != 0 || output.current_b != 0) return 14;
    input.enabled = false;
    output = controller.update(input);
    if (output.current_a != 0 || output.current_b != 0) return 15;
  }
  // Slow tracking inside the former settling deadband must retain drive torque.
  // Yaw updates every 10 ms while the controller runs every 5 ms.
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, ratio};
    controller.update(input);
    app::LinkageOutput output;
    const float direction_b = ratio > 0 ? 1.0f : -1.0f;
    for (unsigned step = 0; step < 160; ++step) {
      if (step % 2 == 0) input.yaw += 0.0004f;
      input.yaw_rate = 0.04f;
      input.angle_a = input.yaw - 0.004f;
      input.angle_b = ratio * input.yaw - direction_b * 0.004f;
      input.speed_a = 0.04f;
      input.speed_b = ratio * input.speed_a;
      output = controller.update(input);
      if (output.manual_source != 0) return 16;
      if (step > 100 && output.current_a <= 0.0001f) return 17;
      if (step > 100 && output.current_b * direction_b <= 0.0001f) return 18;
      if (!near(output.target_a, input.yaw) || !near(output.target_b, ratio * input.yaw)) return 19;
    }
    input.angle_a = output.target_a;
    input.angle_b = output.target_b;
    input.yaw_rate = input.speed_a = input.speed_b = 0;
    for (unsigned step = 0; step < 150; ++step) output = controller.update(input);
    if (output.current_a != 0 || output.current_b != 0) return 20;
  }
  if (!near(app::mapped_reset_direction(0.2f, 2048, 1), 0.2f + sp::SP_PI / 2)) return 21;
  if (!near(app::mapped_reset_direction(0.2f, 2048, -1), 0.2f - sp::SP_PI / 2)) return 22;
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    app::LinkageController controller;
    // Arbitrary startup rotor angles must return to the fixed mapping, not startup positions.
    app::LinkageInput input = {true, 0, 0, 4 * sp::SP_PI + 1, -4 * sp::SP_PI - 1, 0, 0, ratio,
                               true, 0, 0};
    auto output = controller.update(input);
    if (!near(output.target_a, 4 * sp::SP_PI) || !near(output.target_b, -4 * sp::SP_PI)) return 23;
    if (output.current_a >= 0 || output.current_b <= 0 || output.manual_source != 0) return 24;
    const float target_a = output.target_a;
    const float target_b = output.target_b;
    input.angle_a += 0.1f;
    input.speed_a = 1;
    for (unsigned step = 0; step < 20; ++step) output = controller.update(input);
    if (
      output.manual_source != 0 || !near(output.target_a, target_a) ||
      !near(output.target_b, target_b))
      return 25;
    input.yaw = 0.2f;
    input.yaw_rate = 0.2f;
    input.reset_direction_a += 0.2f;
    input.reset_direction_b += 0.2f;
    output = controller.update(input);
    if (!near(output.target_a, target_a + 0.2f) || !near(output.target_b, target_b + 0.2f))
      return 26;
    input.angle_a = output.target_a;
    input.angle_b = output.target_b;
    input.speed_a = input.speed_b = input.yaw_rate = 0;
    for (unsigned step = 0; step < 150; ++step) output = controller.update(input);
    if (!output.reset_complete || output.current_a != 0 || output.current_b != 0) return 27;
    input.reset_requested = false;
    output = controller.update(input);
    if (
      !near(output.target_a, input.angle_a) || !near(output.target_b, input.angle_b) ||
      output.reset_complete)
      return 28;
    input.enabled = false;
    output = controller.update(input);
    if (output.enabled || output.current_a != 0 || output.current_b != 0) return 29;
    // A CAN reconnection can restart encoder unwrapping; fixed mapping still selects the same direction.
    input.enabled = true;
    input.reset_requested = true;
    input.angle_a = 6.1f;
    input.angle_b = 0.1f;
    input.reset_direction_a = input.reset_direction_b = 0;
    output = controller.update(input);
    if (!near(output.target_a, 2 * sp::SP_PI) || !near(output.target_b, 0)) return 30;
  }
  {
    app::LinkageController controller;
    // Reproduce the stationary residual errors observed on the real reset trial.
    app::LinkageInput input = {true, 0, 0, -0.188f, 0.024f, 0, 0, -1, true, 0, 0};
    app::LinkageOutput output;
    float previous_a = 0;
    float previous_b = 0;
    for (unsigned step = 0; step < 4000; ++step) {
      output = controller.update(input);
      if (output.reset_complete) return 31;
      const float allowed_step = app::MOTOR_CURRENT_SLEW_A_PER_S * app::LINKAGE_PERIOD_S;
      if (
        std::abs(output.current_a - previous_a) > allowed_step + 0.00001f ||
        std::abs(output.current_b - previous_b) > allowed_step + 0.00001f)
        return 32;
      if (
        std::abs(output.current_a) > app::MOTOR_CURRENT_LIMIT_A + 0.00001f ||
        std::abs(output.current_b) > app::MOTOR_CURRENT_LIMIT_A + 0.00001f)
        return 33;
      previous_a = output.current_a;
      previous_b = output.current_b;
    }
    if (output.current_a < 0.19f || output.current_b > -0.14f) return 34;
    input.angle_a = output.target_a;
    input.angle_b = output.target_b;
    for (unsigned step = 0; step < 150; ++step) output = controller.update(input);
    if (!output.reset_complete || output.current_a != 0 || output.current_b != 0) return 35;
    input.enabled = false;
    output = controller.update(input);
    if (output.current_a != 0 || output.current_b != 0) return 36;
  }
  {
    app::LinkageController controller;
    // Continuous tracking below the reduced B 1.2 rad/s limit must still drive forward.
    app::LinkageInput input = {true, 0, 0.3f, 0, 0, 0.3f, 0.9f, 3};
    app::LinkageOutput output = controller.update(input);
    float previous_b = output.current_b;
    for (unsigned step = 0; step < 400; ++step) {
      input.yaw += 0.0015f;
      input.angle_a = input.yaw;
      input.angle_b = 3 * input.yaw - 0.1f;
      output = controller.update(input);
      if (std::abs(output.current_b) > app::MOTOR_CURRENT_LIMIT_A + 0.00001f) return 37;
      if (
        std::abs(output.current_b - previous_b) >
        app::MOTOR_CURRENT_SLEW_A_PER_S * app::LINKAGE_PERIOD_S + 0.00001f)
        return 38;
      if (step > 300 && output.current_b <= 0) return 39;
      previous_b = output.current_b;
    }
    // Stopping the input must retain the 1:3 destination until the follower arrives.
    input.yaw_rate = input.speed_a = input.speed_b = 0;
    input.angle_a = output.target_a;
    input.angle_b = output.target_b;
    for (unsigned step = 0; step < 300; ++step) output = controller.update(input);
    if (!near(output.target_b, 3 * input.yaw) || output.current_b != 0) return 40;
    input.enabled = false;
    output = controller.update(input);
    if (output.current_a != 0 || output.current_b != 0) return 41;
  }
  {
    app::LinkageController controller;
    // Finish a multi-turn 1:3 run and enter UP while the board is far from startup yaw.
    app::LinkageInput input = {true, 1.1f, 0, 8 * sp::SP_PI + 0.3f, -12 * sp::SP_PI + 2.0f,
                               0,    0,    3};
    controller.update(input);
    input.reset_requested = true;
    input.reset_direction_a =
      app::mapped_reset_direction(1.1f, app::MOTOR_A_R_ALIGNMENT_ENCODER, 1);
    input.reset_direction_b =
      app::mapped_reset_direction(1.1f, app::MOTOR_B_R_ALIGNMENT_ENCODER, 1);
    auto output = controller.update(input);
    if (
      std::abs(output.target_a - input.angle_a) > sp::SP_PI + 0.00001f ||
      std::abs(output.target_b - input.angle_b) > sp::SP_PI + 0.00001f)
      return 42;
    // Reset aligns a direction, not the old revolution index after continued rotation.
    input.angle_a = output.target_a + 2 * sp::SP_PI;
    input.angle_b = output.target_b - 4 * sp::SP_PI;
    for (unsigned step = 0; step < 150; ++step) output = controller.update(input);
    if (!output.reset_complete || output.current_a != 0 || output.current_b != 0) return 43;
    if (
      std::abs(output.target_a - input.angle_a) > 0.002f ||
      std::abs(output.target_b - input.angle_b) > 0.002f)
      return 44;
  }
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, -0.1f, -0.1f, 0, 0, 3, true, 0, 0};
    app::LinkageOutput output;
    for (unsigned step = 0; step < 300; ++step) output = controller.update(input);
    // Tiny alternating IMU target steps must not preserve an old reset integral forever.
    for (unsigned step = 0; step < 300; ++step) {
      input.reset_direction_a = input.reset_direction_b = (step / 2) % 2 ? 0.00004f : 0;
      input.angle_a = input.angle_b = input.reset_direction_a;
      output = controller.update(input);
    }
    if (!output.reset_complete || output.current_a != 0 || output.current_b != 0) return 45;
  }
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3, true, 0, 0};
    app::LinkageOutput output;
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    // B stays released through small arrival noise, but a real displacement resumes correction.
    input.angle_b = 0.01f;
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    if (output.current_b != 0 || !output.reset_complete) return 46;
    input.angle_b = 0.04f;
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    if (output.current_b >= 0 || output.reset_complete) return 47;
    // A target crossing sheds B's previous positive integral and commands braking.
    input.angle_b = -0.1f;
    for (unsigned step = 0; step < 500; ++step) output = controller.update(input);
    input.angle_b = 0.03f;
    input.speed_b = 0.2f;
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    if (output.current_b >= 0) return 48;
  }
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    app::LinkageController controller;
    const float alignment_a = app::mapped_reset_direction(0, app::MOTOR_A_R_ALIGNMENT_ENCODER, 1);
    const float alignment_b = app::mapped_reset_direction(0, app::MOTOR_B_R_ALIGNMENT_ENCODER, 1);
    app::LinkageInput input = {true, 0,     0,    alignment_a, alignment_b, 0,
                               0,    ratio, true, alignment_a, alignment_b};
    auto output = controller.update(input);
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    if (
      !output.reset_complete || !near(output.target_a, alignment_a) ||
      !near(output.target_b, alignment_b))
      return 77;
    // Force both shafts away. Their measured positions must never become command inputs.
    input.angle_a += 0.1f;
    input.angle_b -= 0.1f;
    input.speed_a = 0.2f;
    input.speed_b = -0.2f;
    for (unsigned step = 0; step < 50; ++step) {
      output = controller.update(input);
      if (
        output.manual_source != 0 || output.reset_complete || !near(output.target_a, alignment_a) ||
        !near(output.target_b, alignment_b))
        return 78;
    }
    if (output.current_a >= 0 || output.current_b <= 0) return 79;
    // Both targets follow C by the same positive/negative angle in every ratio.
    for (float direction : {1.0f, -1.0f}) {
      input.yaw = direction * sp::SP_PI / 2;
      input.yaw_rate = direction * 0.5f;
      input.reset_direction_a = alignment_a + input.yaw;
      input.reset_direction_b = alignment_b + input.yaw;
      output = controller.update(input);
      if (
        !near(output.target_a, alignment_a + input.yaw) ||
        !near(output.target_b, alignment_b + input.yaw) || output.manual_source != 0)
        return 80;
    }
  }
  {
    // Recover the supplied correct pose without replacing the existing startup yaw.
    const float yaw = 28.28f * sp::SP_PI / 180;
    const float count_to_rad = 2 * sp::SP_PI / 8192;
    const float target_a = app::mapped_reset_direction(yaw, app::MOTOR_A_R_ALIGNMENT_ENCODER, 1);
    const float target_b = app::mapped_reset_direction(yaw, app::MOTOR_B_R_ALIGNMENT_ENCODER, 1);
    if (
      std::abs(target_a - 5294 * count_to_rad) > 0.5f * count_to_rad ||
      std::abs(target_b - 4370 * count_to_rad) > 0.5f * count_to_rad)
      return 81;
    app::LinkageController controller;
    app::LinkageInput input = {
      true, yaw, 0, 5294 * count_to_rad, 4370 * count_to_rad, 0, 0, 3, true, target_a, target_b};
    app::LinkageOutput output;
    for (unsigned step = 0; step < 100; ++step) output = controller.update(input);
    if (
      !output.reset_complete || output.manual_source != 0 || output.current_a != 0 ||
      output.current_b != 0)
      return 82;
  }
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    for (unsigned source : {1u, 2u}) {
      app::LinkageController controller;
      app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, ratio};
      controller.update(input);
      app::LinkageOutput output;
      // More than four turns with a follower that never catches up.
      for (unsigned step = 0; step < 600; ++step) {
        if (source == 1) {
          input.angle_a += 0.05f;
          input.speed_a = 10;
        }
        else {
          input.angle_b += 0.05f;
          input.speed_b = 10;
        }
        output = controller.update(input);
        if (
          step > 10 && (output.manual_source != source ||
                        (source == 1 ? output.current_a : output.current_b) != 0))
          return 83;
        if (step > 10 && !near(output.target_b, ratio * output.target_a)) return 84;
      }
      input.speed_a = input.speed_b = 0;
      for (unsigned step = 0; step < 70; ++step) output = controller.update(input);
      // Swap while the previous follower is still far from its goal: no settling gate.
      const unsigned next = 3 - source;
      const float hand_direction = next == 1 ? (input.angle_a >= output.target_a ? 1.0f : -1.0f)
                                             : (input.angle_b >= output.target_b ? 1.0f : -1.0f);
      for (unsigned step = 0; step < 20; ++step) {
        if (next == 1) {
          input.angle_a += hand_direction * 0.005f;
          input.speed_a = hand_direction;
        }
        else {
          input.angle_b += hand_direction * 0.005f;
          input.speed_b = hand_direction;
        }
        output = controller.update(input);
      }
      if (output.manual_source != next || (next == 1 ? output.current_a : output.current_b) != 0)
        return 85;
    }
  }
  for (float direction : {1.0f, -1.0f}) {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3};
    controller.update(input);
    app::LinkageOutput output;
    // A low/quantized source speed must not cancel feedforward while its target keeps moving.
    for (unsigned step = 0; step < 300; ++step) {
      input.yaw += direction * 0.00006f;
      input.yaw_rate = direction * 0.008f;
      input.angle_a = input.yaw - direction * 0.004f;
      input.angle_b = 3 * input.yaw - direction * 0.004f;
      input.speed_a = direction * 0.012f;
      input.speed_b = direction * 0.036f;
      output = controller.update(input);
      if (step > 200 && output.current_b * direction <= 0.0001f) return 86;
      if (output.manual_source != 0 || !near(output.target_b, 3 * input.yaw)) return 87;
    }
    input.angle_a = output.target_a;
    input.angle_b = output.target_b;
    input.yaw_rate = input.speed_a = input.speed_b = 0;
    for (unsigned step = 0; step < 150; ++step) output = controller.update(input);
    if (output.current_b != 0) return 88;
  }
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3};
    controller.update(input);
    app::LinkageOutput output;
    // Stationary-board yaw drift crosses the old 0.005-rad displacement gate repeatedly.
    for (unsigned step = 0; step < 1200; ++step) {
      input.yaw += 0.00001f;
      input.yaw_rate = 0.002f;
      input.angle_a += 0.003f;
      input.speed_a = 0.6f;
      input.angle_b = 3 * input.angle_a - 0.1f;
      input.speed_b = 1.2f;
      output = controller.update(input);
      if (
        step > 20 && (output.manual_source != 1 || output.current_a != 0 ||
                      !near(output.target_b, 3 * input.angle_a)))
        return 89;
    }
    // Deliberate board rotation must still take control from the hand source.
    input.yaw += 0.1f;
    input.yaw_rate = 0.5f;
    output = controller.update(input);
    if (output.manual_source != 0) return 90;
  }
  return 0;
}
