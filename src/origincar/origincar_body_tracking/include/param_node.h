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

#ifndef PARA_NODE_H_
#define PARA_NODE_H_
#include <string>

#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ParametersClass : public rclcpp::Node {
 public:
  ParametersClass(
      TrackCfg *cfg,
      const std::string &node_name = "tracking_strategy_parameter_node")
      : Node(node_name) {
    cfg_ = cfg;
    if (cfg_) {
      this->declare_parameter<int>("activate_wakeup_gesture",
                                   cfg_->activate_wakeup_gesture);
      this->declare_parameter<int>("track_body", cfg_->track_body);
      this->declare_parameter<int>("track_serial_lost_num_thr",
                                   cfg_->track_serial_lost_num_thr);
      this->declare_parameter<int>("activate_robot_rotate_thr",
                                   cfg_->activate_robot_rotate_thr);
      this->declare_parameter<int>("track_lost_protection_angel_thr",
                                   cfg_->track_lost_protection_angel_thr);
      this->declare_parameter<int>("track_overmoving_protection_angel_thr",
                                   cfg_->track_overmoving_protection_angel_thr);
      this->declare_parameter<int>("activate_robot_move_thr",
                                   cfg_->activate_robot_move_thr);
      this->declare_parameter<int>("stop_robot_move_to_top_thr",
                                   cfg_->stop_robot_move_to_top_thr);
      this->declare_parameter<double>("linear_velocity", cfg_->move_step);
      this->declare_parameter<double>("angular_velocity", cfg_->rotate_step);
      this->declare_parameter<double>("stop_move_rect_width_ratio_thr",
                                     cfg_->stop_move_rect_width_ratio_thr);
      this->declare_parameter<double>("start_move_rect_width_ratio_thr",
                                     cfg_->start_move_rect_width_ratio_thr);
      this->declare_parameter<int>("img_width", cfg_->img_width);
      this->declare_parameter<int>("img_height", cfg_->img_height);

      timer_ = this->create_wall_timer(
          1000ms, std::bind(&ParametersClass::Respond, this));
    }
  }

  void Respond() {
    if (!cfg_) return;
    this->get_parameter("track_body", cfg_->track_body);
    this->get_parameter("activate_wakeup_gesture",
                        cfg_->activate_wakeup_gesture);
    this->get_parameter("track_serial_lost_num_thr",
                        cfg_->track_serial_lost_num_thr);
    this->get_parameter("activate_robot_rotate_thr",
                        cfg_->activate_robot_rotate_thr);
    this->get_parameter("track_lost_protection_angel_thr",
                        cfg_->track_lost_protection_angel_thr);
    this->get_parameter("track_overmoving_protection_angel_thr",
                        cfg_->track_overmoving_protection_angel_thr);
    this->get_parameter("activate_robot_move_thr",
                        cfg_->activate_robot_move_thr);
    this->get_parameter("stop_robot_move_to_top_thr",
                        cfg_->stop_robot_move_to_top_thr);

    double linear_vel, angular_vel;
    this->get_parameter("linear_velocity", linear_vel);
    cfg_->move_step = linear_vel;
    this->get_parameter("angular_velocity", angular_vel);
    cfg_->rotate_step = angular_vel;

    double stop_ratio, start_ratio;
    this->get_parameter("stop_move_rect_width_ratio_thr", stop_ratio);
    cfg_->stop_move_rect_width_ratio_thr = stop_ratio;
    this->get_parameter("start_move_rect_width_ratio_thr", start_ratio);
    cfg_->start_move_rect_width_ratio_thr = start_ratio;

    this->get_parameter("img_width", cfg_->img_width);
    this->get_parameter("img_height", cfg_->img_height);

    std::stringstream ss;
    ss << "activate_wakeup_gesture: " << cfg_->activate_wakeup_gesture
       << std::endl;
    ss << "track_serial_lost_num_thr: " << cfg_->track_serial_lost_num_thr
       << std::endl;
    ss << "activate_robot_rotate_thr: " << cfg_->activate_robot_rotate_thr
       << std::endl;
    ss << "activate_robot_move_thr: " << cfg_->activate_robot_move_thr
       << std::endl;
    ss << "linear_velocity: " << cfg_->move_step << std::endl;
    ss << "angular_velocity: " << cfg_->rotate_step << std::endl;
    ss << "img_width: " << cfg_->img_width << std::endl;
    ss << "img_height: " << cfg_->img_height << std::endl;

    if (first_dump_config_) {
      first_dump_config_ = false;
      RCLCPP_WARN(rclcpp::get_logger("ParametersClass"),
                  "TrackCfg param are\n%s",
                  ss.str().data());
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("ParametersClass"),
                   "TrackCfg param are\n%s",
                   ss.str().data());
    }

    if (cfg_->stop_move_rect_width_ratio_thr <
        cfg_->start_move_rect_width_ratio_thr) {
      RCLCPP_ERROR(rclcpp::get_logger("ParametersClass"),
                   "stop_move_rect_width_ratio_thr and "
                   "start_move_rect_width_ratio_thr params are abnormal\n");
    }
  }

 private:
  TrackCfg *cfg_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  bool first_dump_config_ = true;
};
#endif