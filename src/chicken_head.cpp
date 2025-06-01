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

#include "chicken_head.h"

ChickenHead::ChickenHead(const rclcpp::NodeOptions &options)
    : rclcpp::Node("chicken_head_node",options),
      l1_(lower_to_upper_arm_[0]),
      l2_(upper_arm_to_wrist1_[0])
{
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("arm/joint_states", 100);
    cmd_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "body_pose", 1, std::bind(&ChickenHead::cmdPoseCallback_, this, std::placeholders::_1));

    this->declare_parameter<float>("gait/nominal_height", 0.0);
    this->get_parameter("gait/nominal_height", nominal_height_);

    loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5), std::bind(&ChickenHead::controlLoop_, this));

    req_pose_.roll = 0.0;
    req_pose_.pitch = 0.0;
    req_pose_.yaw = 0.0;
    req_pose_.z = nominal_height_;
}


Eigen::Vector3d ChickenHead::rotate(const Vector3d &pos, const float alpha, const float phi, const float beta)
{
    Eigen::Matrix3d Rx, Ry, Rz;
    Vector3d xformed_pos;

    // Rotation matrices for each axis
    Rz << cos(beta), -sin(beta), 0, sin(beta), cos(beta), 0, 0, 0, 1;
    xformed_pos = (Rz * pos).eval();

    Ry << cos(phi), 0, sin(phi), 0, 1, 0, -sin(phi), 0, cos(phi);
    xformed_pos = (Ry * xformed_pos).eval();

    Rx << 1, 0, 0, 0, cos(alpha), -sin(alpha), 0, sin(alpha), cos(alpha);
    xformed_pos = (Rx * xformed_pos).eval();

    return xformed_pos;
}

void ChickenHead::controlLoop_()
{
    // Rotate initial position based on the required pose (roll, pitch)
    Vector3d target_pos = rotate(initial_pos_, -req_pose_.roll, -req_pose_.pitch, 0);
    target_pos[2] += nominal_height_ - req_pose_.z;
    target_pos -= base_to_lower_arm_;  // Adjust by the arm geometry

    // Rotation based on yaw for wrist position
    Vector3d temp_pos = rotate(initial_pos_, -req_pose_.roll, -req_pose_.pitch, -req_pose_.yaw);
    temp_pos -= wrist1_to_wrist2_;  // Adjust by wrist geometry

    // Compute the joint positions for the robotic arm
    float base_joint = atan2(temp_pos[1], temp_pos[0]);

    float x = target_pos[0];
    float y = target_pos[2];

    float upper_joint = acos((pow(x, 2) + pow(y, 2) - pow(l1_, 2) - pow(l2_, 2)) / (2 * l1_ * l2_));
    float lower_joint = -atan(y / x) - atan((l2_ * sin(upper_joint)) / (l1_ + (l2_ * cos(upper_joint))));

    float alpha = M_PI - upper_joint;
    float beta = M_PI - abs(alpha) - abs(lower_joint);

    float wrist1_joint = -beta - req_pose_.pitch;
    float wrist2_joint = -req_pose_.roll;

    // Joint state message to publish
    std::vector<std::string> joint_names = {
        "base_joint", "lower_arm_joint", "upper_arm_joint", "wrist1_joint", "wrist2_joint"
    };

    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = this->get_clock()->now();
    joint_states.name = joint_names;
    joint_states.position = { base_joint, lower_joint, upper_joint, wrist1_joint, wrist2_joint };

    // Check for NaN values in the joint positions
    for (const auto& pos : joint_states.position)
    {
        if(std::isnan(pos))
        {
            return;  // Avoid publishing NaN values
        }
    }

    // Publish joint states
    joint_state_publisher_->publish(joint_states);
}

void ChickenHead::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Extract the quaternion from the message
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    // Create a tf2::Matrix3x3 from the quaternion and extract RPY
    tf2::Matrix3x3 m(quat);  // Convert quaternion to rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  // Extract RPY from the rotation matrix

    // Now you can assign the angles to your request pose
    req_pose_.roll = roll;
    req_pose_.pitch = pitch;
    req_pose_.yaw = yaw;

    // Update the requested z pose (height) based on the message
    req_pose_.z = msg->position.z + nominal_height_;
}


