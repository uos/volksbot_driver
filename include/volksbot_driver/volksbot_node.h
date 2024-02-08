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

#pragma once
#include <epos2_motor_controller/Epos2.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <iostream>

struct VolksbotParameters
{
    double wheel_radius = 0.0985;
    double axis_length = 0.41;
    int64_t gear_ratio = 74;
    int64_t max_vel_l = 8250;
    int64_t max_vel_r = 8400;
    int64_t max_acc_l = 10000;
    int64_t max_acc_r = 10000;
    bool drive_backwards = false;
    double turning_adaptation = 0.85;
    int64_t num_wheels = 4;
    double sigma_x = 0.002;
    double sigma_theta = 0.017;
    double cov_x_y = 0.0;
    double cov_x_theta = 0.0;
    double cov_y_theta = 0.0;
    bool publish_tf = false;
    std::string tf_prefix = "";
};

class VolksbotNode : public rclcpp::Node
{
public:
    VolksbotNode();

    ~VolksbotNode();

    /// @brief  Updates the current odometry estimation
    void odometry();

    /// @brief  Sets the speeds for both wheels
    /// @param _v_l_target              Left wheel speed
    /// @param _v_r_target              Right wheel speed
    void set_wheel_speed(double _v_l_target, double _v_r_target);

    /// @brief  Returns the maximum driving velocity
    double get_max_vel() const;

private:

    /// @brief  Timed callback to set the velocity to zero (i.e. stop) when no
    ///         new commands were received
    void cmdVelWatchdog();

    /// @brief  Callback for velocity commands
    /// @param twist                    Twist message setting the current velocity
    void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr twist);

    /// @brief  Override default parameters with given parameters
    void updateParams();

    void init_motor_controllers();

    /// @brief  Publish current odometry estimation and update joint states
    /// @param x                        x position
    /// @param y                        y position
    /// @param theta                    Rotation
    /// @param v_x                      Forward velocity
    /// @param v_theta                  Rotation velocity
    void publish_odometry(double x, double y, double theta, double v_x, double v_theta);

    /// @brief  Publish joints for all six / four wheels
    /// @param rotation_r               Rotation state of the right wheel
    /// @param rotation_l               Rotation state of the left wheel
    void publish_rotations(double rotation_r, double rotation_l);

    /// @brief  Compute covariance matrix for odometry
    /// @param msg                      Odometry message whose covariance is filled
    /// @param v_x                      Current linear velocity
    /// @param v_theta                  Current angular velocity
    void populateCovariance(nav_msgs::msg::Odometry& msg, double v_x, double v_theta);

    /// @brief  Volksbot configuration paramters
    VolksbotParameters params_;

    /// @brief  Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    /// @brief  Joint state publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    /// @brief  Timer for odom reset 
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_cmd_vel_time_;

    /// @brief  Subsciption to cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    /// @brief  TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// @brief  Current rotation state of the left wheel
    double rotation_l_ = 0.0;

    /// @brief  Current rotation state of the right wheel
    double rotation_r_ = 0.0;

    /// @brief  EPOS2 interface of left controller
    std::shared_ptr<CEpos2> epos2_left_;

    /// @brief  EPOS2 interface of right controller
    std::shared_ptr<CEpos2> epos2_right_;

    /// @brief  The joint names for all wheels
    std::vector<std::string> joint_names_;
};