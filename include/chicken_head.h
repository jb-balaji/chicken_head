/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CHICKEN_HEAD_H
#define CHICKEN_HEAD_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "champ_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

class ChickenHead : public rclcpp::Node
{
public:
    ChickenHead();  // Constructor

private:
    using Vector3d = Eigen::Vector3d;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    // Parameters
    float nominal_height_;

    // Internal state
    champ_msgs::msg::Pose req_pose_;

    // Geometry constants
    Vector3d initial_pos_ = Vector3d(0.391, 0.0, 0.165); 
    Vector3d base_to_lower_arm_ = Vector3d(0.070, 0.000, 0.100); 
    Vector3d lower_to_upper_arm_ = Vector3d(0.195, 0.000, 0.000); 
    Vector3d upper_arm_to_wrist1_ = Vector3d(0.195, 0.000, 0.000); 
    Vector3d wrist1_to_wrist2_ = Vector3d(0.065, 0.000, 0.000);

    float l1_;
    float l2_;

    // Helper methods
    Vector3d rotate(const Vector3d pos, float alpha, float phi, float beta);
    void cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg);
    void controlLoop_();
};

#endif  // CHICKEN_HEAD_H

