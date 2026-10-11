// Copyright (c) 2024，D-Robotics.
// 融合自追踪包，适配OriginCar智能小车
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

#ifndef ROBOT_CTRL_NODE_H_
#define ROBOT_CTRL_NODE_H_
#include <string>

#include "common.h"

class RobotCmdVelNode : public rclcpp::Node {
 public:
  explicit RobotCmdVelNode(const std::string &node_name) : Node(node_name) {
    RCLCPP_INFO(rclcpp::get_logger("RobotCmdVelNode"),
                "RobotCmdVelNode construct");

    // 获取发布话题名称参数（适配origincar_base订阅的/cmd_vel）
    this->declare_parameter<std::string>("twist_pub_topic_name", "/cmd_vel");
    this->get_parameter<std::string>("twist_pub_topic_name", twist_pub_topic_name_);

    twist_publisher_ = this->create_publisher<Twist>(twist_pub_topic_name_, 10);

    RCLCPP_INFO(rclcpp::get_logger("RobotCmdVelNode"),
                "Publishing twist commands to topic: %s", twist_pub_topic_name_.c_str());
  }

  void RobotCtl(const Twist &msg) const {
    std::stringstream ss;
    ss << "RobotCtl"
       << ", angular: " << msg.angular.x << " " << msg.angular.y << " "
       << msg.angular.z << ", linear: " << msg.linear.x << " " << msg.linear.y
       << " " << msg.linear.z
       << ", pub twist ts: " << TimeHelper::GetCurrentTimestampMicroSec();
    RCLCPP_INFO(rclcpp::get_logger("RobotCmdVelNode"), "%s", ss.str().data());

    twist_publisher_->publish(msg);
  }

 private:
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher_ = nullptr;
  std::string twist_pub_topic_name_ = "/cmd_vel";
};

#endif