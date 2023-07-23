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
 * Author: Enrique Fernández
 */

#include "quad_drive_controller/odometry.hpp"

namespace quad_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_track_(0.0),
  front_wheel_radius_(0.0),
  rear_wheel_radius_(0.0),
  front_wheel_old_pos_(0.0),
  rear_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double front_pos, double rear_pos, double front_hinge_pos, double rear_hinge_pos, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double front_wheel_cur_pos = front_pos * front_wheel_radius_;
  const double rear_wheel_cur_pos = rear_pos * rear_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double front_wheel_est_vel = front_wheel_cur_pos - front_wheel_old_pos_;
  const double rear_wheel_est_vel = rear_wheel_cur_pos - rear_wheel_old_pos_;

  // Update old position with current:
  front_wheel_old_pos_ = front_wheel_cur_pos;
  rear_wheel_old_pos_ = rear_wheel_cur_pos;

  updateFromVelocity(front_wheel_est_vel, rear_wheel_est_vel, front_hinge_pos, rear_hinge_pos, time);

  return true;
}

bool Odometry::updateFromVelocity(double front_vel, double rear_vel, double front_hinge_pos, double rear_hinge_pos, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // Compute linear and angular diff:
  // const double linear = (front_vel + rear_vel) * 0.5;
  const double linear_x = (front_vel*std::cos(front_hinge_pos) + rear_vel*std::cos(rear_hinge_pos)) * 0.5;
  const double linear_y = (front_vel*std::sin(front_hinge_pos) + rear_vel*std::sin(rear_hinge_pos)) * 0.5;
  // Now there is a bug about scout angular velocity
  const double angular = (front_vel - rear_vel) / wheel_track_;

  // Integrate odometry:
  integrateExact(linear_x, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear_x / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_x_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear_x_ * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_track, double front_wheel_radius, double rear_wheel_radius)
{
  wheel_track_ = wheel_track;
  wheel_track_ = wheel_track;

  front_wheel_radius_ = front_wheel_radius;
  rear_wheel_radius_ = rear_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear_x_ * cos(direction);
  y_ += linear_y_ * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace quad_drive_controller
