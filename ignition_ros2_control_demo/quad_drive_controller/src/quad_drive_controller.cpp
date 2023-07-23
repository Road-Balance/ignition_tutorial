// Copyright 2023 Road Balance
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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "quad_drive_controller/quad_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace quad_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

QuadDriveController::QuadDriveController() : controller_interface::ControllerInterface() {}

const char * QuadDriveController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn QuadDriveController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration QuadDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.front_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.front_hinge_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.rear_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.rear_hinge_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

  InterfaceConfiguration QuadDriveController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : params_.front_wheel_names)
    {
      conf_names.push_back(joint_name + "/" + feedback_type());
    }
    for (const auto & joint_name : params_.front_hinge_names)
    {
      conf_names.push_back(joint_name + "/" + feedback_type());
    }
    for (const auto & joint_name : params_.rear_wheel_names)
    {
      conf_names.push_back(joint_name + "/" + feedback_type());
    }
    for (const auto & joint_name : params_.rear_hinge_names)
    {
      conf_names.push_back(joint_name + "/" + feedback_type());
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

controller_interface::return_type QuadDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  // if (age_of_last_command > cmd_vel_timeout_)
  // {
  //   last_command_msg->twist.linear.x = 0.0;
  //   last_command_msg->twist.angular.z = 0.0;
  // }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & linear_command_x = command.twist.linear.x;
  double & linear_command_y = command.twist.linear.y;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  // const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double wheel_track = params_.wheel_track_multiplier * params_.wheel_track;
  const double front_wheel_radius = params_.front_wheel_radius_multiplier * params_.wheel_radius;
  const double rear_wheel_radius = params_.rear_wheel_radius_multiplier * params_.wheel_radius;

  // No odom now
  // if (params_.open_loop)
  // {
  //   odometry_.updateOpenLoop(linear_command_x, linear_command_y, angular_command, time);
  // }
  // else
  // {
  //   double front_feedback_mean = 0.0;
  //   double rear_feedback_mean = 0.0;
  //   double front_hinge_feedback_mean = 0.0;
  //   double rear_hinge_feedback_mean = 0.0;
  //   for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
  //   {
  //     const double front_feedback = registered_front_wheel_handles_[index].feedback.get().get_value();
  //     const double rear_feedback =
  //       registered_rear_wheel_handles_[index].feedback.get().get_value();

  //     const double front_hinge_feedback = registered_front_hinge_handles_[index].position_state.get().get_value();
  //     const double rear_hinge_feedback =
  //       registered_rear_hinge_handles_[index].position_state.get().get_value();

  //     if (std::isnan(front_feedback) || std::isnan(rear_feedback))
  //     {
  //       RCLCPP_ERROR(
  //         logger, "Either the left or right wheel %s is invalid for index [%zu]", feedback_type(),
  //         index);
  //       return controller_interface::return_type::ERROR;
  //     }

  //     front_feedback_mean += front_feedback;
  //     rear_feedback_mean += rear_feedback;
  //     front_hinge_feedback_mean += front_hinge_feedback;
  //     rear_hinge_feedback_mean += rear_hinge_feedback;
  //   }
  //   front_feedback_mean /= static_cast<double>(params_.wheels_per_side);
  //   rear_feedback_mean /= static_cast<double>(params_.wheels_per_side);
  //   front_hinge_feedback_mean /= static_cast<double>(params_.wheels_per_side);
  //   rear_hinge_feedback_mean /= static_cast<double>(params_.wheels_per_side);

  //   if (params_.position_feedback)
  //   {
  //     odometry_.update(front_feedback_mean, rear_feedback_mean, front_hinge_feedback_mean, rear_hinge_feedback_mean, time);
  //   }
  //   else
  //   {
  //     odometry_.updateFromVelocity(
  //       front_feedback_mean * front_wheel_radius * period.seconds(),
  //       rear_feedback_mean * rear_wheel_radius * period.seconds(),
  //       front_hinge_feedback_mean * period.seconds(),
  //       rear_hinge_feedback_mean * period.seconds(), time);
  //   }
  // }

  // tf2::Quaternion orientation;
  // orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  // bool should_publish = false;
  // try
  // {
  //   if (previous_publish_timestamp_ + publish_period_ < time)
  //   {
  //     previous_publish_timestamp_ += publish_period_;
  //     should_publish = true;
  //   }
  // }
  // catch (const std::runtime_error &)
  // {
  //   // Handle exceptions when the time source changes and initialize publish timestamp
  //   previous_publish_timestamp_ = time;
  //   should_publish = true;
  // }

  // if (should_publish)
  // {
  //   if (realtime_odometry_publisher_->trylock())
  //   {
  //     auto & odometry_message = realtime_odometry_publisher_->msg_;
  //     odometry_message.header.stamp = time;
  //     odometry_message.pose.pose.position.x = odometry_.getX();
  //     odometry_message.pose.pose.position.y = odometry_.getY();
  //     odometry_message.pose.pose.orientation.x = orientation.x();
  //     odometry_message.pose.pose.orientation.y = orientation.y();
  //     odometry_message.pose.pose.orientation.z = orientation.z();
  //     odometry_message.pose.pose.orientation.w = orientation.w();
  //     odometry_message.twist.twist.linear.x = odometry_.getLinear();
  //     odometry_message.twist.twist.angular.z = odometry_.getAngular();
  //     realtime_odometry_publisher_->unlockAndPublish();
  //   }

  //   if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
  //   {
  //     auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
  //     transform.header.stamp = time;
  //     transform.transform.translation.x = odometry_.getX();
  //     transform.transform.translation.y = odometry_.getY();
  //     transform.transform.rotation.x = orientation.x();
  //     transform.transform.rotation.y = orientation.y();
  //     transform.transform.rotation.z = orientation.z();
  //     transform.transform.rotation.w = orientation.w();
  //     realtime_odometry_transform_publisher_->unlockAndPublish();
  //   }
  // }

  // auto & last_command = previous_commands_.back().twist;
  // auto & second_to_last_command = previous_commands_.front().twist;
  // limiter_linear_x_.limit(
  //   linear_command_x, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  // limiter_linear_y_.limit(
  //   linear_command_y, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
  // limiter_angular_.limit(
  //   angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  RCLCPP_INFO(logger, "linear_command_x %f", linear_command_x);
  RCLCPP_INFO(logger, "linear_command_y %f", linear_command_y);

  // previous_commands_.pop();
  // previous_commands_.emplace(command);

  // // Publish limited velocity
  // if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  // {
  //   auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
  //   limited_velocity_command.header.stamp = time;
  //   limited_velocity_command.twist = command.twist;
  //   realtime_limited_velocity_publisher_->unlockAndPublish();
  // }

  // Compute wheels velocities:
  double theta1 = std::atan((linear_command_y + wheel_track*angular_command/2) / linear_command_x);
  double theta2 = std::atan((linear_command_y - wheel_track*angular_command/2) / linear_command_x);
  
  if(std::abs(theta1) < 0.01)
    theta1 = 0.0;
  if( std::isnan(theta1) )
    theta1 = 0.0;

  if(std::abs(theta2) < 0.01)
    theta2 = 0.0;
  if( std::isnan(theta2) )
    theta2 = 0.0;

  double sin1 = std::sin(theta1); 
  double cos1 = std::cos(theta1);
  double sin2 = std::sin(theta2); 
  double cos2 = std::cos(theta2);

  const double velocity_front = (linear_command_x*cos1 + linear_command_y*sin1 + angular_command*sin1*wheel_track/2 ) / front_wheel_radius;
  const double velocity_rear = (linear_command_x*cos2 + linear_command_y*sin2 - angular_command*sin2*wheel_track/2 ) / rear_wheel_radius;

  // Set wheels velocities:
  for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
  {
    registered_front_wheel_handles_[index].velocity.get().set_value(velocity_front);
    registered_rear_wheel_handles_[index].velocity.get().set_value(velocity_rear);

    registered_front_hinge_handles_[index].position_command.get().set_value(theta1);
    registered_rear_hinge_handles_[index].position_command.get().set_value(theta2);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn QuadDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.front_wheel_names.size() != params_.rear_wheel_names.size())
  {
    RCLCPP_ERROR(
      logger, "The number of front wheels [%zu] and the number of rear wheels [%zu] are different",
      params_.front_wheel_names.size(), params_.rear_wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.front_wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  // const double wheel_track = params_.wheel_track_multiplier * params_.wheel_track;
  const double front_wheel_radius = params_.front_wheel_radius_multiplier * params_.wheel_radius;
  const double rear_wheel_radius = params_.rear_wheel_radius_multiplier * params_.wheel_radius;

  odometry_.setWheelParams(wheel_separation, front_wheel_radius, rear_wheel_radius);
  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  publish_limited_velocity_ = params_.publish_limited_velocity;
  use_stamped_vel_ = params_.use_stamped_vel;

  limiter_linear_x_ = SpeedLimiter(
    params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
    params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
    params_.linear.x.max_jerk);

  limiter_linear_y_ = SpeedLimiter(
    params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
    params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity, params_.linear.y.max_velocity,
    params_.linear.y.min_acceleration, params_.linear.y.max_acceleration, params_.linear.y.min_jerk,
    params_.linear.y.max_jerk);

  limiter_angular_ = SpeedLimiter(
    params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
    params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
    params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  params_.wheels_per_side = params_.front_wheel_names.size();

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  // previous_commands_.emplace(empty_twist);
  // previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  std::string controller_namespace = std::string(get_node()->get_namespace());

  if (controller_namespace == "/")
  {
    controller_namespace = "";
  }
  else
  {
    controller_namespace = controller_namespace + "/";
  }

  const auto odom_frame_id = controller_namespace + params_.odom_frame_id;
  const auto base_frame_id = controller_namespace + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = controller_namespace + odom_frame_id;
  odometry_message.child_frame_id = controller_namespace + base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto front_result =
    configure_side("front_wheel", params_.front_wheel_names, registered_front_wheel_handles_);
  const auto rear_result =
    configure_side("rear_wheel", params_.rear_wheel_names, registered_rear_wheel_handles_);

  const auto front_hinge_result =
    configure_hinge("front_hinge", params_.front_hinge_names, registered_front_hinge_handles_);
  const auto rear_hinge_result =
    configure_hinge("rear_hinge", params_.rear_hinge_names, registered_rear_hinge_handles_);

  if (
    front_result == controller_interface::CallbackReturn::ERROR ||
    rear_result == controller_interface::CallbackReturn::ERROR ||
    front_hinge_result == controller_interface::CallbackReturn::ERROR ||
    rear_hinge_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_front_wheel_handles_.empty() || registered_rear_wheel_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Either front wheel interfaces, rear wheel interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  registered_front_wheel_handles_.clear();
  registered_rear_wheel_handles_.clear();
  registered_front_hinge_handles_.clear();
  registered_rear_hinge_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadDriveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool QuadDriveController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_front_wheel_handles_.clear();
  registered_rear_wheel_handles_.clear();
  registered_front_hinge_handles_.clear();
  registered_rear_hinge_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn QuadDriveController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void QuadDriveController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  const auto halt_hinges = [](auto & hinge_handles)
  {
    for (const auto & wheel_handle : hinge_handles)
    {
      wheel_handle.position_command.get().set_value(0.0);
    }
  };

  halt_wheels(registered_front_wheel_handles_);
  halt_wheels(registered_rear_wheel_handles_);
  halt_hinges(registered_front_hinge_handles_);
  halt_hinges(registered_rear_hinge_handles_);
}

controller_interface::CallbackReturn QuadDriveController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadDriveController::configure_hinge(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<SteeringHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_POSITION;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace quad_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  quad_drive_controller::QuadDriveController, controller_interface::ControllerInterface)
