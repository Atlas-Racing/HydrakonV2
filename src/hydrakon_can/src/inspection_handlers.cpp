/**
 * @file inspection_handlers.cpp
 * @brief Hardcoded inspection and autonomous demo mission handlers
 *
 * Authored by: @abdulmaajidaga
 */

#include <hydrakon_can/hydrakon_can.hpp>
#include <algorithm>
#include <cmath>

void HydrakonCanInterface::handleStaticInspectionA() {
  if (!driving_flag_ || inspection_completed_) return;

  auto now = this->now();
  if (!inspection_started_) {
    inspection_started_ = true;
    inspection_stage_ = 0;
    stage_start_time_ = inspection_start_time_ = now;
    RCLCPP_INFO(get_logger(), "Static Inspection A mission started.");
  }

  double t = (now - stage_start_time_).seconds();
  auto next = [&]() { inspection_stage_++; stage_start_time_ = now; };

  switch (inspection_stage_) {
    case 0: {
      float target = -MAX_STEERING_ANGLE_DEG_;
      float delta = t * STEERING_RAMP_RATE;
      steering_ = std::max(target, 0.0f - delta);
      if (steering_ <= target) {
        steering_ = target;
        next();
      }
      break;
    }

    case 1: {
      float target = MAX_STEERING_ANGLE_DEG_;
      float delta = t * STEERING_RAMP_RATE;
      steering_ = std::min(target, -MAX_STEERING_ANGLE_DEG_ + delta);
      if (steering_ >= target) {
        steering_ = target;
        next();
      }
      break;
    }

    case 2: {
      float delta = t * STEERING_RAMP_RATE;
      steering_ = std::max(0.0f, MAX_STEERING_ANGLE_DEG_ - delta);
      if (steering_ <= 0.0f) {
        steering_ = 0.0f;
        next();
      }
      break;
    }

    case 3: {
      float acceleration = (5.0f * M_PI * WHEEL_RADIUS_) / 9.0f;
      float smooth_acc = std::min(acceleration, static_cast<float>(t * acceleration / 3.0f));

      auto msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
      msg->header.stamp = this->now();
      msg->header.frame_id = "";
      msg->drive.steering_angle = steering_ * M_PI / 180.0f;
      msg->drive.acceleration = smooth_acc;
      msg->drive.speed = 0.0f;
      msg->drive.steering_angle_velocity = 0.0f;
      msg->drive.jerk = 0.0f;

      this->commandCallback(msg);

      rpm_request_ = std::min(200.0f, static_cast<float>(t * RPM_RAMP_RATE));

      if (t >= 10.0) next();
      break;
    }

    case 4: {
      rpm_request_ = 0.0f;
      torque_ = 0.0f;
      braking_ = 60.0f;
      if (t >= 5.0) next();
      break;
    }

    case 5: {
      auto msg = std::make_shared<std_msgs::msg::Bool>();
      msg->data = true;
      flagCallback(msg);

      // as_state_ is overwritten by VCU data every loop — this has no effect.
      // AS_FINISHED is triggered on the VCU by sending MISSION_FINISHED via AI2VCU_MISSION_STATUS,
      // which getMissionStatus() returns automatically once mission_complete_ is true.
      // as_state_ = fs_ai_api_as_state_e::AS_FINISHED;
      inspection_completed_ = true;
      RCLCPP_INFO(get_logger(), "Static Inspection A mission complete.");
      break;
    }
  }

  last_cmd_message_time_ = this->now().seconds();
}


void HydrakonCanInterface::handleStaticInspectionB() {
  if (!driving_flag_ || inspection_completed_) return;

  auto now = this->now();
  if (!inspection_started_) {
    inspection_started_ = true;
    inspection_stage_ = 0;
    stage_start_time_ = inspection_start_time_ = now;
    RCLCPP_INFO(get_logger(), "Static Inspection B mission started.");
  }

  double t = (now - stage_start_time_).seconds();
  auto next = [&]() { inspection_stage_++; stage_start_time_ = now; };

  switch (inspection_stage_) {
    case 0: {
      float acceleration = (5.0f * M_PI * WHEEL_RADIUS_) / 9.0f;
      float smooth_acc = std::min(acceleration, static_cast<float>(t * acceleration / 3.0f));

      auto msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
      msg->header.stamp = this->now();
      msg->header.frame_id = "";
      msg->drive.steering_angle = steering_ * M_PI / 180.0f;
      msg->drive.acceleration = smooth_acc;
      msg->drive.speed = 0.0f;
      msg->drive.steering_angle_velocity = 0.0f;
      msg->drive.jerk = 0.0f;

      this->commandCallback(msg);

      rpm_request_ = std::min(50.0f, static_cast<float>(t * RPM_RAMP_RATE));
      torque_ = TOTAL_MASS_ * smooth_acc * WHEEL_RADIUS_;

      if (t >= 3.0) next();
      break;
    }

    case 1: {  // Trigger EBS and set AS to EMERGENCY
      ebs_state_ = fs_ai_api_estop_request_e::ESTOP_YES;
      // AS_EMERGENCY_BRAKE is triggered on the VCU by sending ESTOP_YES via AI2VCU_ESTOP_REQUEST.
      // as_state_ is overwritten by VCU data every loop — this has no effect.
      // as_state_ = fs_ai_api_as_state_e::AS_EMERGENCY_BRAKE;
      inspection_completed_ = true;
      RCLCPP_WARN(get_logger(), "Static Inspection B: EBS Triggered");
      break;
    }
  }

  last_cmd_message_time_ = this->now().seconds();
}


void HydrakonCanInterface::handleAutonomousDemo() {
  if (!driving_flag_ || inspection_completed_) return;

  auto now = this->now();
  if (!inspection_started_) {
    inspection_started_ = true;
    inspection_stage_ = 0;
    stage_start_time_ = inspection_start_time_ = now;
    RCLCPP_INFO(get_logger(), "Autonomous Demo mission started.");
  }

  double t = (now - stage_start_time_).seconds();
  auto next = [&]() { inspection_stage_++; stage_start_time_ = now; };

  switch (inspection_stage_) {
    case 0:  // Sweep right (-ve = right)
      steering_ = -std::min(MAX_STEERING_ANGLE_DEG_, static_cast<float>(t * STEERING_RAMP_RATE));
      if (steering_ <= -MAX_STEERING_ANGLE_DEG_) next();
      break;

    case 1:  // Sweep left (+ve = left) — start from -MAX so the full arc is covered
      steering_ = std::min(MAX_STEERING_ANGLE_DEG_, -MAX_STEERING_ANGLE_DEG_ + static_cast<float>(t * STEERING_RAMP_RATE));
      if (steering_ >= MAX_STEERING_ANGLE_DEG_) next();
      break;

    case 2: {  // Return to center
      steering_ = std::max(0.0f, MAX_STEERING_ANGLE_DEG_ - static_cast<float>(t * STEERING_RAMP_RATE));
      if (steering_ <= 0.0f) { steering_ = 0.0f; next(); }
      break;
    }

    case 3:  // Re-accelerate to 15kph
      rpm_request_ = std::min(1500.0f, static_cast<float>(t * RPM_RAMP_RATE));
      torque_ = 195.0f; braking_ = 0.0f;
      if (rpm_request_ >= 1500.0f || t >= 4.0) next();
      break;

    case 4:  // Brake to stop
      rpm_request_ = 0.0f; torque_ = 0.0f; braking_ = 60.0f;
      if (t >= 3.0) next();
      break;

    case 5:  // Re-accelerate to 15kph again
      rpm_request_ = std::min(3000.0f, static_cast<float>(t * RPM_RAMP_RATE));
      torque_ = 195.0f; braking_ = 0.0f;
      if (rpm_request_ >= 3000.0f || t >= 4.0) next();
      break;

    case 6:  // EBS deploy
      ebs_state_ = fs_ai_api_estop_request_e::ESTOP_YES;
      // AS_EMERGENCY_BRAKE is triggered on the VCU by sending ESTOP_YES via AI2VCU_ESTOP_REQUEST.
      // as_state_ is overwritten by VCU data every loop — this has no effect.
      // as_state_ = fs_ai_api_as_state_e::AS_EMERGENCY_BRAKE;
      inspection_completed_ = true;
      RCLCPP_WARN(get_logger(), "Autonomous Demo complete. EBS triggered.");
      break;
  }

  last_cmd_message_time_ = this->now().seconds();
}
