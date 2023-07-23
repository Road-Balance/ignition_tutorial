// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#ifndef QUAD_DRIVE_CONTROLLER__QUAD_DRIVE_CONTROLLER_HPP_
#define QUAD_DRIVE_CONTROLLER__QUAD_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "quad_drive_controller/odometry.hpp"
#include "quad_drive_controller/speed_limiter.hpp"
#include "quad_drive_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

// auto-generated by generate_parameter_library
#include "quad_drive_controller_parameters.hpp"

namespace quad_drive_controller
{
class QuadDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  QUAD_DRIVE_CONTROLLER_PUBLIC
  QuadDriveController();

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  QUAD_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };
  struct SteeringHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command;
  };

  const char * feedback_type() const;
  controller_interface::CallbackReturn configure_side(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);

  controller_interface::CallbackReturn configure_hinge(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<SteeringHandle> & registered_handles);

  // std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_front_wheel_handles_;
  // std::vector<WheelHandle> registered_right_wheel_handles_;
  std::vector<WheelHandle> registered_rear_wheel_handles_;

  std::vector<SteeringHandle> registered_front_hinge_handles_;
  std::vector<SteeringHandle> registered_rear_hinge_handles_;

  // Parameters from ROS for diff_drive_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  Odometry odometry_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_x_;
  SpeedLimiter limiter_linear_y_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace quad_drive_controller
#endif  // QUAD_DRIVE_CONTROLLER__QUAD_DRIVE_CONTROLLER_HPP_
