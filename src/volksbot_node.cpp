/*
 * Copyright (c) 2013, 2023 Osnabrueck University, Fulda University of Applied Sciences
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name Osnabrueck University or Fulda University of Applied Sciences 
 *   nor the names of theis contributors may be used to endorse or promote products 
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <chrono>


#include <tf2/convert.h>

#include "rclcpp/rclcpp.hpp"
#include "volksbot_driver/volksbot_node.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

VolksbotNode::VolksbotNode() : rclcpp::Node("volksbot_base")
{
  // Save current time
  last_cmd_vel_time_ = now();
  
  // Create timer and register callback
  timer_ = this->create_wall_timer(100ms, std::bind(&VolksbotNode::cmdVelWatchdog, this));

  // Subscibe to cmd_vel
  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>( "cmd_vel", 10, std::bind(&VolksbotNode::cmdVelCallback, this, _1));
  
  // Create odometry publisher
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Create publisher for joint states
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  // Create tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Update configuration parameters
  updateParams();

  // Initialize controllers
  init_motor_controllers();
}

VolksbotNode::~VolksbotNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down motor controllers");
  epos2_left_->setTargetVelocity(0.0);
  epos2_right_->setTargetVelocity(0.0);
  epos2_left_->close();
  epos2_right_->close();
}

void VolksbotNode::updateParams()
{
  // Declare parameters
  declare_parameter("wheel_radius", params_.wheel_radius);
  declare_parameter("axis_length", params_.axis_length);
  declare_parameter("gear_ratio", params_.gear_ratio);
  declare_parameter("max_vel_l", params_.max_vel_l);
  declare_parameter("max_vel_r", params_.max_vel_r);
  declare_parameter("max_acc_l", params_.max_acc_l);
  declare_parameter("max_acc_r", params_.max_acc_r);
  declare_parameter("drive_backwards", params_.drive_backwards);
  declare_parameter("turning_adaptation", params_.turning_adaptation);
  declare_parameter("num_wheels", params_.num_wheels);
  declare_parameter("x_stddev", params_.sigma_x);
  declare_parameter("rotation_stddev", params_.sigma_theta);
  declare_parameter("cov_xy", params_.cov_x_y);
  declare_parameter("cov_xrotation", params_.cov_x_theta);
  declare_parameter("cov_yrotation", params_.cov_y_theta);
  declare_parameter("publish_tf", params_.publish_tf);
  declare_parameter("tf_prefix", params_.tf_prefix);

  //Get actual parameter values
  get_parameter_or("wheel_radius", params_.wheel_radius, params_.wheel_radius);
  get_parameter_or("axis_length", params_.axis_length, params_.axis_length);
  get_parameter_or("gear_ratio", params_.gear_ratio, params_.gear_ratio);
  get_parameter_or("max_vel_l", params_.max_vel_l, params_.max_vel_l);
  get_parameter_or("max_vel_r", params_.max_vel_r, params_.max_vel_r);
  get_parameter_or("max_acc_l", params_.max_acc_l, params_.max_acc_l);
  get_parameter_or("max_acc_r", params_.max_acc_r, params_.max_acc_r);
  get_parameter_or("drive_backwards", params_.drive_backwards, params_.drive_backwards);
  get_parameter_or("turning_adaptation", params_.turning_adaptation, params_.turning_adaptation);
  get_parameter_or("num_wheels", params_.num_wheels, params_.num_wheels);
  get_parameter_or("x_stddev", params_.sigma_x, params_.sigma_x);
  get_parameter_or("rotation_stddev", params_.sigma_theta, params_.sigma_theta);
  get_parameter_or("cov_xy", params_.cov_x_y, params_.cov_x_y);
  get_parameter_or("cov_xrotation", params_.cov_x_theta, params_.cov_x_theta);
  get_parameter_or("cov_yrotation", params_.cov_y_theta, params_.cov_y_theta);
  get_parameter_or("publish_tf", params_.publish_tf, params_.publish_tf);
  get_parameter_or("tf_prefix", params_.tf_prefix, params_.tf_prefix);

  // Try to get joint names from parameter
  rclcpp::Parameter joint_param("joint_names", std::vector<std::string>());
  this->get_parameter("joint_names", joint_param);
  joint_names_ = joint_param.as_string_array();

  if ((joint_names_.size() == 4) || (joint_names_.size() == 6))
  {
    RCLCPP_INFO(this->get_logger(), "Using names from parameters for %ld wheels.", joint_names_.size());
    
  }
  else
  {
    if (params_.num_wheels == 4 || params_.num_wheels == 6)
    {
      if (params_.num_wheels == 6)
      {
        joint_names_ =
            {
                "left_front_wheel_joint",
                "left_middle_wheel_joint",
                "left_rear_wheel_joint",
                "right_front_wheel_joint",
                "right_middle_wheel_joint",
                "right_rear_wheel_joint"};
        RCLCPP_INFO(this->get_logger(), "Using default joint names for six wheels.");
      }
      else
      {
        joint_names_ =
            {
                "left_front_wheel_joint",
                "left_rear_wheel_joint",
                "right_front_wheel_joint",
                "right_rear_wheel_joint"};
        RCLCPP_INFO(this->get_logger(), "Using default joint names for four wheels.");
      }
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "Wrong volksbot driver configuration: Only four or six wheels are supported! Set param \"num_wheels\" correctly.");
    }
  }
}

void VolksbotNode::init_motor_controllers()
{
  RCLCPP_INFO(this->get_logger(), "Initializing motor controllers");

  epos2_left_ = std::make_shared<CEpos2>(params_.drive_backwards ? 0x01 : 0x02);
  epos2_right_ = std::make_shared<CEpos2>(params_.drive_backwards ? 0x02 : 0x01);

  epos2_left_->init();
  epos2_right_->init();

  epos2_left_->enableController();
  epos2_right_->enableController();

  epos2_left_->enableMotor(epos2_left_->VELOCITY);
  epos2_right_->enableMotor(epos2_right_->VELOCITY);

  epos2_left_->setProfileData(
      0,                 // Velocity
      params_.max_vel_l, // Max Velocity
      0,                 // Acceleration
      0,                 // Deceleration
      0,                 // QS Decel
      params_.max_acc_l, // Max acc
      0                  // Type: Trapecoidal
  );

  epos2_right_->setProfileData(
      0,                 // Velocity
      params_.max_vel_r, // Max Velocity
      0,                 // Acceleration
      0,                 // Deceleration
      0,                 // QS Decel
      params_.max_acc_r, // Max acc
      0                  // Type: Trapecoidal
  );

  epos2_left_->setOperationMode(epos2_left_->VELOCITY);
  epos2_right_->setOperationMode(epos2_right_->VELOCITY);
}

void VolksbotNode::odometry()
{
  static double x = 0.0;
  static double y = 0.0;
  static double theta = 0.0;
  static long enc_left_last = epos2_left_->readEncoderCounter();
  static long enc_right_last = epos2_right_->readEncoderCounter();
  static long enc_per_turn_left = 4 * epos2_left_->getEncoderPulseNumber() * params_.gear_ratio;
  static long enc_per_turn_right = 4 * epos2_right_->getEncoderPulseNumber() * params_.gear_ratio;

  long enc_left = epos2_left_->readEncoderCounter();
  long enc_right = epos2_right_->readEncoderCounter();
  long wheel_l = enc_left - enc_left_last;
  long wheel_r = enc_right - enc_right_last;

  // handle overflow (> 10000 required to ensure we don't do this on zero crossings)
  if ((abs(enc_left) > 10000) && (std::signbit(enc_left) != std::signbit(enc_left_last)))
  {
    if (std::signbit(enc_left))
      wheel_l = std::numeric_limits<int32_t>::max() - enc_left_last - std::numeric_limits<int32_t>::min() + enc_left;
    else
      wheel_l = std::numeric_limits<int32_t>::max() - enc_left - std::numeric_limits<int32_t>::min() + enc_left_last;
  }

  if ((abs(enc_right) > 10000) && (std::signbit(enc_right) != std::signbit(enc_right_last)))
  {
    if (std::signbit(enc_right))
      wheel_r = std::numeric_limits<int32_t>::max() - enc_right_last - std::numeric_limits<int32_t>::min() + enc_right;
    else
      wheel_r = std::numeric_limits<int32_t>::max() - enc_right - std::numeric_limits<int32_t>::min() + enc_right_last;
  }

  enc_left_last = enc_left;
  enc_right_last = enc_right;

  double rotation_L = 2.0 * M_PI * static_cast<double>(wheel_l) / static_cast<double>(enc_per_turn_left);
  double rotation_R = -2.0 * M_PI * static_cast<double>(wheel_r) / static_cast<double>(enc_per_turn_right);

  if (!std::isfinite(rotation_L))
  {
    RCLCPP_INFO(this->get_logger(), "Error rotation_L: %f", rotation_L);
  }

  if (!std::isfinite(rotation_R))
  {
    RCLCPP_INFO(this->get_logger(), "Error rotation_R: %f", rotation_R);
  }

  double wheel_L = params_.wheel_radius * rotation_L;
  double wheel_R = params_.wheel_radius * rotation_R;

  if (!std::isfinite(wheel_L))
  {
    RCLCPP_INFO(this->get_logger(), "Error wheel_L: %f", wheel_L);
  }

  if (!std::isfinite(wheel_R))
  {
    RCLCPP_INFO(this->get_logger(), "Error wheel_R: %f", wheel_R);
  }

  double dtheta = (wheel_R - wheel_L) / params_.axis_length * params_.turning_adaptation;
  double hypothenuse = 0.5 * (wheel_L + wheel_R);

  // update state
  x += hypothenuse * cos(theta + dtheta * 0.5);
  y += hypothenuse * sin(theta + dtheta * 0.5);
  theta += dtheta;

  if (theta > M_PI)
    theta -= 2.0 * M_PI;
  if (theta < -M_PI)
    theta += 2.0 * M_PI;

  rotation_l_ += rotation_L;
  rotation_r_ += rotation_R;

  double v_left = epos2_left_->readVelocity() / 60.0 / params_.gear_ratio * 2.0 * M_PI * params_.wheel_radius;
  double v_right = -epos2_right_->readVelocity() / 60.0 / params_.gear_ratio * 2.0 * M_PI * params_.wheel_radius;
  double v_x = (v_left + v_right) * 0.5;
  double v_theta = (v_right - v_left) / params_.axis_length * params_.turning_adaptation;

  double wheelpos_l = 2.0 * M_PI * (enc_left % enc_per_turn_left) / enc_per_turn_left;
  if (wheelpos_l > M_PI)
    wheelpos_l -= 2.0 * M_PI;
  if (wheelpos_l < -M_PI)
    wheelpos_l += 2.0 * M_PI;

  double wheelpos_r = 2.0 * M_PI * (enc_right % enc_per_turn_right) / enc_per_turn_right;
  if (wheelpos_r > M_PI)
    wheelpos_r -= 2.0 * M_PI;
  if (wheelpos_r < -M_PI)
    wheelpos_r += 2.0 * M_PI;

  publish_odometry(x, y, theta, v_x, v_theta);
  publish_rotations(rotation_l_, rotation_r_);
}

void VolksbotNode::publish_odometry(double x, double y, double theta, double v_x, double v_theta)
{
  nav_msgs::msg::Odometry odom;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  odom.header.stamp = now();
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  // Have to convert manually because tf2::toMsg() isn't working
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = v_x;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = v_theta;

  // Compute covriances and add to odom
  populateCovariance(odom, v_x, v_theta);

  odometry_publisher_->publish(odom);

  if (params_.publish_tf)
  {
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom_combined";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.header.stamp = now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    tf2::Quaternion rot;
    rot.setRPY(0, 0, theta);

    odom_trans.transform.rotation.x = rot.x();
    odom_trans.transform.rotation.y = rot.y();
    odom_trans.transform.rotation.z = rot.z();
    odom_trans.transform.rotation.w = rot.w();

    tf_broadcaster_->sendTransform(odom_trans);
  }
}

void VolksbotNode::populateCovariance(nav_msgs::msg::Odometry &msg, double v_x, double v_theta)
{
  double odom_multiplier = 1.0;

  if (fabs(v_x) <= 1e-8 && fabs(v_theta) <= 1e-8)
  {
    // nav_msgs::msg::Odometryhas a 6x6 covariance matrix
    msg.twist.covariance[0] = 1e-12;
    msg.twist.covariance[35] = 1e-12;

    msg.twist.covariance[30] = 1e-12;
    msg.twist.covariance[5] = 1e-12;
  }
  else
  {
    // nav_msgs::msg::Odometryhas a 6x6 covariance matrix
    msg.twist.covariance[0] = odom_multiplier * pow(params_.sigma_x, 2);
    msg.twist.covariance[35] = odom_multiplier * pow(params_.sigma_theta, 2);

    msg.twist.covariance[30] = odom_multiplier * params_.cov_x_theta;
    msg.twist.covariance[5] = odom_multiplier * params_.cov_x_theta;
  }

  msg.twist.covariance[7] = DBL_MAX;
  msg.twist.covariance[14] = DBL_MAX;
  msg.twist.covariance[21] = DBL_MAX;
  msg.twist.covariance[28] = DBL_MAX;

  msg.pose.covariance = msg.twist.covariance;

  if (fabs(v_x) <= 1e-8 && fabs(v_theta) <= 1e-8)
  {
    msg.pose.covariance[7] = 1e-12;

    msg.pose.covariance[1] = 1e-12;
    msg.pose.covariance[6] = 1e-12;

    msg.pose.covariance[31] = 1e-12;
    msg.pose.covariance[11] = 1e-12;
  }
  else
  {
    msg.pose.covariance[7] = odom_multiplier * pow(params_.sigma_x, 2) * pow(params_.sigma_theta, 2);

    msg.pose.covariance[1] = odom_multiplier * params_.cov_x_y;
    msg.pose.covariance[6] = odom_multiplier * params_.cov_x_y;

    msg.pose.covariance[31] = odom_multiplier * params_.cov_y_theta;
    msg.pose.covariance[11] = odom_multiplier * params_.cov_y_theta;
  }
}

void VolksbotNode::publish_rotations(double rotation_R, double rotation_L)
{
   sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name.resize(params_.num_wheels);
  joint_state.position.resize(params_.num_wheels);
  joint_state.name = joint_names_;

  if(params_.num_wheels == 6)
  {
    joint_state.position[0] = joint_state.position[1] = joint_state.position[2] = rotation_L;
    joint_state.position[3] = joint_state.position[4] = joint_state.position[5] = rotation_R;
  }
  else
  {
    joint_state.position[0] = joint_state.position[1] = rotation_L;
    joint_state.position[2] = joint_state.position[3] = rotation_R; 
  }

  joint_state_publisher_->publish(joint_state);
}


double VolksbotNode::get_max_vel() const
{
  return (params_.max_vel_r + params_.max_vel_l) * M_PI * params_.wheel_radius / (60.0 * params_.gear_ratio);
}

void VolksbotNode::set_wheel_speed(double _v_l_target, double _v_r_target)
{
  epos2_left_->setTargetVelocity(_v_l_target / (2.0 * M_PI * params_.wheel_radius) * 60.0 * params_.gear_ratio);
  epos2_right_->setTargetVelocity(-1.0 * _v_r_target / (2.0 * M_PI * params_.wheel_radius) * 60.0 * params_.gear_ratio);
}

void VolksbotNode::cmdVelWatchdog()
{
  if (now() - last_cmd_vel_time_ > rclcpp::Duration::from_seconds(0.6))
  {
    set_wheel_speed(0.0, 0.0);
  }
}

void VolksbotNode::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_time_ = now();
  double max_vel = get_max_vel();
  double velocity = msg->linear.x;

  velocity = std::min(max_vel, velocity);
  velocity = std::max(-max_vel, velocity);

  if((abs(velocity) + params_.axis_length * abs(msg->angular.z) * 0.5) > max_vel)
  {
    double diff = (abs(velocity) + params_.axis_length * abs(msg->angular.z) * 0.5) - max_vel;
    velocity -= std::copysign(diff,velocity);
  }

  set_wheel_speed(velocity - params_.axis_length * msg->angular.z * 0.5, velocity + params_.axis_length * msg->angular.z * 0.5);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<VolksbotNode> node = std::make_shared<VolksbotNode>();

  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->odometry();
  }

  return 0;
}