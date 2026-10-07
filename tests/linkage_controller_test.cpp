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
    // Continuous tracking below the reduced B 2.4 rad/s limit must still drive forward.
    app::LinkageInput input = {true, 0, 0.7f, 0, 0, 0.7f, 2.1f, 3};
    app::LinkageOutput output = controller.update(input);
    float previous_b = output.current_b;
    for (unsigned step = 0; step < 400; ++step) {
      input.yaw += 0.0035f;
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
    for (unsigned source : {1u, 2u}) {
      app::LinkageController controller;
      app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, ratio};
      controller.update(input);
      app::LinkageOutput output;
      for (unsigned step = 0; step < 50; ++step) {
        if (source == 1) {
          input.angle_a += 0.005f;
          input.speed_a = 1;
        }
        else {
          input.angle_b += 0.005f;
          input.speed_b = 1;
        }
        output = controller.update(input);
      }
      const float retained_a = output.target_a;
      const float retained_b = output.target_b;
      input.speed_a = input.speed_b = 0;
      // The follower continues away from its goal after the operator stops the source.
      for (unsigned step = 0; step < 120; ++step) {
        if (source == 1) {
          input.angle_b = retained_b + 0.1f + step * 0.001f;
          input.speed_b = 0.2f;
        }
        else {
          input.angle_a = retained_a + 0.1f + step * 0.001f;
          input.speed_a = 0.2f;
        }
        output = controller.update(input);
        if (output.manual_source != source) return 49;
        if (!near(output.target_a, retained_a) || !near(output.target_b, retained_b)) return 50;
        if ((source == 1 ? output.current_a : output.current_b) != 0) return 51;
      }
      // Only a settled follower permits a new hand-input source to take over.
      input.angle_a = retained_a;
      input.angle_b = retained_b;
      input.speed_a = input.speed_b = 0;
      for (unsigned step = 0; step < 200; ++step) output = controller.update(input);
      if (output.manual_source != 0) return 52;
      for (unsigned step = 0; step < 50; ++step) {
        if (source == 1) {
          input.angle_b += 0.005f;
          input.speed_b = 1;
        }
        else {
          input.angle_a += 0.005f;
          input.speed_a = 1;
        }
        output = controller.update(input);
      }
      if (output.manual_source != 3 - source) return 53;
    }
  }
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3};
    controller.update(input);
    app::LinkageOutput output;
    for (unsigned step = 0; step < 50; ++step) {
      input.angle_a += 0.005f;
      input.speed_a = 1;
      output = controller.update(input);
    }
    const float retained_a = output.target_a;
    const float retained_b = output.target_b;
    input.speed_a = 0;
    input.angle_b = retained_b + 0.2f;
    input.speed_b = 0.2f;
    for (unsigned step = 0; step < 80; ++step) output = controller.update(input);
    // Board motion must work even while B is still recovering from a hand-input overshoot.
    input.yaw = 0.1f;
    input.yaw_rate = 0.5f;
    output = controller.update(input);
    if (output.manual_source != 0 || !near(output.target_a, retained_a + 0.1f)) return 54;
    if (!near(output.target_b, retained_b + 0.3f)) return 55;
    input.yaw_rate = 0;
    input.angle_b = output.target_b + 0.2f;
    for (unsigned step = 0; step < 50; ++step) {
      input.angle_b += 0.001f;
      output = controller.update(input);
      if (output.manual_source != 0 || !near(output.target_a, retained_a + 0.1f)) return 56;
    }
    input.enabled = false;
    output = controller.update(input);
    if (output.enabled || output.current_a != 0 || output.current_b != 0) return 57;
  }
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, ratio};
    controller.update(input);
    app::LinkageOutput output;
    for (unsigned step = 0; step < 50; ++step) {
      input.angle_a += 0.005f;
      input.speed_a = 1;
      output = controller.update(input);
    }
    input.speed_a = 0;
    const float goal = output.target_b;
    // Build a negative approach command while B is far above a fixed destination.
    input.angle_b = goal + 1;
    for (unsigned step = 0; step < 180; ++step) output = controller.update(input);
    input.angle_b = goal - 0.05f;
    for (unsigned step = 0; step < 80; ++step) output = controller.update(input);
    if (output.manual_source != 1 || output.current_a != 0 || output.current_b <= 0) return 58;
    if (!near(output.target_b, goal)) return 59;
  }
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3};
    controller.update(input);
    app::LinkageOutput output;
    for (unsigned step = 0; step < 50; ++step) {
      input.angle_a += 0.005f;
      input.speed_a = 1;
      output = controller.update(input);
    }
    input.speed_a = 0;
    const float goal = output.target_b;
    input.angle_b = goal - 2;
    for (unsigned step = 0; step < 180; ++step) output = controller.update(input);
    // B is still approaching, only 1.15 degrees short, at 1 rad/s. Brake before crossing.
    input.angle_b = goal - 0.02f;
    input.speed_b = 1;
    float previous_current = output.current_b;
    for (unsigned step = 0; step < 70; ++step) {
      output = controller.update(input);
      if (
        std::abs(output.current_b - previous_current) >
        app::MOTOR_CURRENT_SLEW_A_PER_S * app::LINKAGE_PERIOD_S + 0.00001f)
        return 60;
      if (std::abs(output.current_b) > app::MOTOR_CURRENT_LIMIT_A + 0.00001f) return 61;
      previous_current = output.current_b;
    }
    if (output.current_b >= 0 || output.manual_source != 1 || output.current_a != 0) return 62;
    if (!near(output.target_b, goal)) return 63;
  }
  for (float ratio : {0.5f, -1.0f, 3.0f}) {
    for (unsigned source : {1u, 2u}) {
      app::LinkageController controller;
      app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, ratio};
      controller.update(input);
      app::LinkageOutput output;
      for (unsigned step = 0; step < 50; ++step) {
        if (source == 1) {
          input.angle_a += 0.005f;
          input.speed_a = 1;
        }
        else {
          input.angle_b += 0.005f;
          input.speed_b = 1;
        }
        output = controller.update(input);
      }
      input.speed_a = input.speed_b = 0;
      // The follower can be stationary short of its goal because of friction or a hand hold.
      if (source == 1)
        input.angle_b = output.target_b + 0.04f;
      else
        input.angle_a = output.target_a + 0.04f;
      for (unsigned step = 0; step < 160; ++step) output = controller.update(input);
      for (unsigned step = 0; step < 50; ++step) {
        if (source == 1) {
          input.angle_b += 0.005f;
          input.speed_b = 1;
        }
        else {
          input.angle_a += 0.005f;
          input.speed_a = 1;
        }
        output = controller.update(input);
      }
      if (output.manual_source != 3 - source) return 64;
      if ((source == 1 ? output.current_b : output.current_a) != 0) return 65;
      if (!near(output.target_b, ratio * output.target_a)) return 66;
    }
  }
  {
    app::LinkageController controller;
    app::LinkageInput input = {true, 0, 0, 0, 0, 0, 0, 3};
    controller.update(input);
    app::LinkageOutput output;
    for (unsigned step = 0; step < 50; ++step) {
      input.angle_a += 0.005f;
      input.speed_a = 1;
      output = controller.update(input);
    }
    const float goal = output.target_b;
    input.angle_b = goal + 0.04f;
    input.speed_a = input.speed_b = 0;
    for (unsigned step = 0; step < 160; ++step) output = controller.update(input);
    // A servo movement toward the goal consumes the rest-based handover opportunity.
    input.speed_b = -0.2f;
    for (unsigned step = 0; step < 20; ++step) {
      input.angle_b -= 0.001f;
      output = controller.update(input);
    }
    // Its subsequent uninterrupted overshoot must not become a new B hand input.
    for (unsigned step = 0; step < 100; ++step) {
      input.angle_b -= 0.001f;
      output = controller.update(input);
      if (output.manual_source != 1 || output.current_a != 0 || !near(output.target_b, goal))
        return 67;
    }
  }
  return 0;
}
