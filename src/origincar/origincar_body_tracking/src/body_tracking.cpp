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

#include "body_tracking.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "time_helper.h"
#include "util.h"

std::shared_ptr<TrackingManager> TrackingManager::Instance() {
  static std::shared_ptr<TrackingManager> inst = nullptr;
  if (!inst) {
    inst = std::shared_ptr<TrackingManager>(new TrackingManager());
  }
  return inst;
}

TrackingManager::TrackingManager() {
  start_ = true;

  track_info_.is_movectrl_running = false;

  param_node_ = std::make_shared<ParametersClass>(&track_cfg_);

  robot_cmdvel_node_ =
      std::make_shared<RobotCmdVelNode>("horizon_tracking_RobotCmdVel");

  last_frame_done_ = true;

  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (start_ && rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(smart_queue_mtx_);
        smart_queue_condition_.wait_for(lg, std::chrono::seconds(1), [&]() {
          return !smart_queue_.empty() || !rclcpp::ok() || !start_;
        });
        if (smart_queue_.empty() || !rclcpp::ok() || !start_) {
          continue;
        }
        auto smart_frame = std::move(smart_queue_.front());
        smart_queue_.pop();
        lg.unlock();
        RunTrackingStrategy(smart_frame);
      }
    });
  }
}

TrackingManager::~TrackingManager() {}

void TrackingManager::Release() {
  RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "TrackingManager release");
  start_ = false;

  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }

  param_node_ = nullptr;
  robot_cmdvel_node_ = nullptr;
}

void TrackingManager::UpdateTrackAngle() {
  if (track_info_.present_rect.empty()) return;

  cv::Point2f fit_center(track_cfg_.img_width / 2, track_cfg_.img_height);
  cv::Point2f start_pt(track_cfg_.img_width, track_cfg_.img_height);
  cv::Point2f end_pt(
      (track_info_.present_rect[0] + track_info_.present_rect[2]) / 2,
      (track_info_.present_rect[1] + track_info_.present_rect[3]) / 2);

  track_info_.angel_with_robot_ =
      CalAngelOfTwoVector(fit_center, start_pt, end_pt);

  {
    std::stringstream ss;
    ss << "frame_ts: " << track_info_.frame_ts
       << ", track_id: " << track_info_.track_id << ", angel_with_robot: "
       << std::abs(track_info_.angel_with_robot_ - 90)
       << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "UpdateTrackAngle: %s",
                ss.str().data());
  }

  auto transform_y_angle_to_x_radian =
      [this](float robot_y_negtive_angel) -> float {
    float robot_x_positive_angel = robot_y_negtive_angel;
    bool kps_is_clock_wise = true;
    if (robot_y_negtive_angel < 90) {
      kps_is_clock_wise = true;
      robot_x_positive_angel = 90 - robot_x_positive_angel;
    } else if (robot_y_negtive_angel > 90) {
      kps_is_clock_wise = false;
      robot_x_positive_angel = robot_x_positive_angel - 90;
    }
    float robot_x_positive_radian = robot_x_positive_angel * PI / 180.0;
    if (kps_is_clock_wise) {
      robot_x_positive_radian = (-1.0) * robot_x_positive_radian;
    }

    std::stringstream ss;
    ss << "robot robot_y_negtive_angel: " << robot_y_negtive_angel
       << ", robot_x_positive_angel: " << robot_x_positive_angel
       << ", robot_x_positive_radian: " << robot_x_positive_radian
       << ", kps_is_clock_wise: " << kps_is_clock_wise;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    return robot_x_positive_radian;
  };

  track_info_.robot_x_positive_radian_with_track_ =
      transform_y_angle_to_x_radian(track_info_.angel_with_robot_);
  track_info_.robot_y_negtive_radian_with_track_ =
      track_info_.angel_with_robot_ * PI / 180.0;
}

bool TrackingManager::RotateSwitch() {
  if (!last_cmdvel_is_cancel_ && 0 == last_cmdvel_type_) return true;

  if (track_info_.serial_lost_num > track_cfg_.track_serial_lost_num_thr) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track_id: %d, serial_lost_num: %d, thr: %d, stop rotate!",
                track_info_.track_id,
                track_info_.serial_lost_num,
                track_cfg_.track_serial_lost_num_thr);
    return false;
  }

  // todo add lowpass strategy
  int activate_robot_rotate_thr = track_cfg_.activate_robot_rotate_thr;
  if (!track_info_.has_face_head) {
    activate_robot_rotate_thr = activate_robot_rotate_thr * 0.5;
  } else if (!last_cmdvel_is_cancel_ && 1 == last_cmdvel_type_) {
    activate_robot_rotate_thr = activate_robot_rotate_thr * 2;
  }
  if (std::abs(track_info_.angel_with_robot_ - 90) >=
      activate_robot_rotate_thr) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "RotateSwitchenable, activate_robot_rotate_thr: %d",
                activate_robot_rotate_thr);
    return true;
  }

  return false;
}

void TrackingManager::DoRotateMove() {
  if (!robot_cmdvel_node_) return;

  int direction = 1;
  float step = 0;
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;

  bool do_move = false;
  bool do_rotate = false;
  if (RotateSwitch()) {
    do_rotate = true;
    // transform angel to yaw shift and wise
    int yaw_shift = 0;
    bool rotate_clock_wise = true;
    if (track_info_.angel_with_robot_ <= 90) {
      yaw_shift = 90 - track_info_.angel_with_robot_;
      rotate_clock_wise = true;
    } else if (track_info_.angel_with_robot_ <= 180) {
      yaw_shift = track_info_.angel_with_robot_ - 90;
      rotate_clock_wise = false;
    } else if (track_info_.angel_with_robot_ <= 270) {
      yaw_shift = track_info_.angel_with_robot_ - 90;
      rotate_clock_wise = false;
    } else if (track_info_.angel_with_robot_ <= 360) {
      yaw_shift = 360 - track_info_.angel_with_robot_ + 90;
      rotate_clock_wise = true;
    }

    float rotate_step_ratio = 1.0f;
    if (yaw_shift > track_cfg_.activate_robot_rotate_thr * 2 ||
        !track_info_.has_face_head) {
      rotate_step_ratio = 2.0;
    }
    float rotate_step = track_cfg_.rotate_step * rotate_step_ratio;
    std::stringstream ss;
    ss << "frame_ts: " << track_info_.frame_ts << ", yaw_shift: " << yaw_shift
       << ", rotate_clock_wise: " << rotate_clock_wise
       << ", rotate_step_ratio: " << rotate_step_ratio
       << ", rotate_step: " << rotate_step;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "rotate switch on: %s",
                ss.str().data());

    if (rotate_clock_wise) direction = 0;
    step = rotate_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "rotate direction: %d, step: %f",
                direction,
                step);
    int direct = 1;
    if (0 == direction) {
      direct = -1;
    }
    twist->angular.z = direct * step;
  }

  if (TrackingSwitchWithVision()) {
    do_move = true;
    direction = track_info_.move_direction;
    step = track_info_.move_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "move switch on, direction: %d, step: %f",
                direction,
                step);
    if (0 == direction) {
      twist->linear.x += step;
    } else if (1 == direction) {
      twist->linear.x -= step;
    } else if (2 == direction) {
      twist->linear.y += step;
    } else if (3 == direction) {
      twist->linear.y -= step;
    }
  }

  if (do_move || do_rotate) {
    last_cmdvel_is_cancel_ = false;
    last_cmdvel_type_ = 2;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "Do rotate move, ts sec: %llu, nanosec: %llu",
                track_info_.frame_ts_sec,
                track_info_.frame_ts_nanosec);
    robot_cmdvel_node_->RobotCtl(*twist);
  } else {
    CancelMove();
  }
}

void TrackingManager::CancelMove() {
  if (last_cmdvel_is_cancel_) return;
  if (robot_cmdvel_node_) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "cancel move");
    auto twist = std::make_shared<Twist>();
    twist->linear.x = 0;
    twist->linear.y = 0;
    twist->linear.z = 0;
    twist->angular.x = 0;
    twist->angular.y = 0;
    twist->angular.z = 0;
    robot_cmdvel_node_->RobotCtl(*twist);
  }

  last_cmdvel_is_cancel_ = true;
  last_cmdvel_type_ = -1;
  last_move_step_ratio_ = 1.0;
}bool TrackingManager::TrackingSwitchWithVision() {
  if (track_info_.present_rect.empty()) {
    return false;
  }

  if (!track_info_.has_face_head) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track has no face and head, too close to robot");
    return false;
  }

  if (track_info_.present_rect[1] < track_cfg_.stop_robot_move_to_top_thr) {
    return false;
  }

  int body_rect_width =
      track_info_.present_rect[2] - track_info_.present_rect[0];
  if (body_rect_width >=
      track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr) {
    RCLCPP_INFO(
        rclcpp::get_logger("TrackingManager"),
        "track width: %d, exceeds %f of img_width_: %d, too close to robot",
        body_rect_width,
        track_cfg_.stop_move_rect_width_ratio_thr,
        track_cfg_.img_width);
    return false;
  }

  track_info_.move_step = track_cfg_.move_step;
  float move_step_ratio = 1.0;
  int body_rect_to_top = track_info_.present_rect[1];
  if (abs(track_info_.robot_x_positive_radian_with_track_ * 180 / PI) <
      track_cfg_.activate_robot_rotate_thr) {
    if (body_rect_to_top > track_cfg_.img_height * 0.5) {
      move_step_ratio = 4;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.45) {
      move_step_ratio = 3.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.4) {
      move_step_ratio = 3.0;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.35) {
      move_step_ratio = 2.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.3) {
      move_step_ratio = 2.0;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.25) {
      move_step_ratio = 1.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.2) {
      move_step_ratio = 1.2;
    }
  }

  if (move_step_ratio > (last_move_step_ratio_ + 0.5)) {
    move_step_ratio = (last_move_step_ratio_ + 0.5);
    last_move_step_ratio_ = move_step_ratio;
  }

  track_info_.move_step = move_step_ratio * track_cfg_.move_step;

  int to_top_thr = track_cfg_.activate_robot_move_thr;
  if (body_rect_width <
          track_cfg_.img_width * track_cfg_.start_move_rect_width_ratio_thr &&
      body_rect_to_top > to_top_thr) {
    std::stringstream ss;
    ss << "Do move! body_rect_width: " << body_rect_width << ", thr: "
       << track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr
       << ", move_step_ratio: " << move_step_ratio
       << ", body_rect_to_top: " << body_rect_to_top
       << ", img_height: " << track_cfg_.img_height
       << ", move_step: " << track_info_.move_step;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    return true;
  }
  return false;
}

void TrackingManager::ProcessSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }

  uint64_t frame_ts = (msg->header.stamp.sec % 1000) * 1000 +
                      msg->header.stamp.nanosec / 1000 / 1000;

  track_info_.frame_ts_sec = msg->header.stamp.sec;
  track_info_.frame_ts_nanosec = msg->header.stamp.nanosec;

  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "process smart frame_ts %llu",
              frame_ts);

  if (TrackingStatus::TRACKING == track_info_.tracking_sta) {
    bool find_track = false;
    std::vector<int> present_rect;
    int body_kps_5_y = -1;
    int body_kps_6_y = -1;
    int gesture = 0;
    for (const auto &target : msg->targets) {
      if (track_cfg_.track_body) {
        bool tar_has_body = false;
        std::vector<int> body_rect;
        for (const auto &roi : target.rois) {
          if ("body" == roi.type) {
            tar_has_body = true;
            body_rect.push_back(roi.rect.x_offset);
            body_rect.push_back(roi.rect.y_offset);
            body_rect.push_back(roi.rect.x_offset + roi.rect.width);
            body_rect.push_back(roi.rect.y_offset + roi.rect.height);
            break;
          }
        }

        if (!tar_has_body) continue;
        if (target.track_id == track_info_.track_id) {
          find_track = true;
          present_rect = body_rect;
          for (const auto &point : target.points) {
            if ("body_kps" == point.type) {
              if (point.point.size() > 6) {
                body_kps_5_y = point.point.at(5).y;
                body_kps_6_y = point.point.at(6).y;
              }
              break;
            }
          }
          break;
        }
      }
    }

    if (find_track) {
      track_info_.serial_lost_num = 0;
      track_info_.last_rect = track_info_.present_rect;
      track_info_.present_rect = present_rect;
      track_info_.frame_ts = frame_ts;
      track_info_.gesture = gesture;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                  "find track fail! frame_ts %llu",
                  frame_ts);

      track_info_.gesture = gesture;
      track_info_.serial_lost_num++;

      if (track_info_.serial_lost_num > 5 &&
          track_info_.serial_lost_num % 10 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                    "frame_ts %llu, track id: %d serial lost num: %d",
                    frame_ts,
                    track_info_.track_id,
                    track_info_.serial_lost_num);
      }

      if (track_info_.serial_lost_num >= track_cfg_.track_serial_lost_num_thr) {
        RCLCPP_ERROR(rclcpp::get_logger("TrackingManager"),
                     "track id: %d serial lost num exceed limit: %d",
                     track_info_.track_id,
                     track_cfg_.track_serial_lost_num_thr);
        memset(static_cast<void *>(&track_info_), 0, sizeof(TrackInfo));
        track_info_.tracking_sta = TrackingStatus::LOST;
      }
    }
  } else if (TrackingStatus::INITING == track_info_.tracking_sta ||
             TrackingStatus::LOST == track_info_.tracking_sta) {
    if (msg->targets.empty()) {
      return;
    }

    bool find_track = false;
    bool has_body = false;
    uint64_t track_id;
    std::vector<int> present_body_rect;
    int gesture = 0;

    for (const auto &target : msg->targets) {
      bool tar_has_body = false;
      std::vector<int> body_rect;
      for (const auto &roi : target.rois) {
        if ("body" == roi.type) {
          tar_has_body = true;
          body_rect.push_back(roi.rect.x_offset);
          body_rect.push_back(roi.rect.y_offset);
          body_rect.push_back(roi.rect.x_offset + roi.rect.width);
          body_rect.push_back(roi.rect.y_offset + roi.rect.height);
          break;
        }
      }

      if (!tar_has_body || body_rect.empty()) continue;
      const auto &rect = body_rect;
      if (!has_body) {
        has_body = true;
        track_id = target.track_id;
        present_body_rect = rect;
      } else if ((present_body_rect[2] - present_body_rect[0]) <
                 (rect[2] - rect[1])) {
        track_id = target.track_id;
        present_body_rect = rect;
      }
    }

    if (has_body) {
      find_track = true;
      memset(static_cast<void *>(&track_info_), 0, sizeof(TrackInfo));
      track_info_.tracking_sta = TrackingStatus::TRACKING;
      track_info_.track_id = track_id;
      track_info_.present_rect = present_body_rect;
      track_info_.gesture = gesture;
      track_info_.frame_ts = frame_ts;

      std::stringstream ss;
      ss << "Tracking body start!"
         << ", track_id: " << track_info_.track_id
         << ", frame_ts: " << track_info_.frame_ts
         << ", tracking_sta: " << static_cast<int>(track_info_.tracking_sta);
      RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "%s", ss.str().data());
    }
  }

  if (TrackingStatus::TRACKING == track_info_.tracking_sta) {
    track_info_.has_face_head = false;
    for (const auto &target : msg->targets) {
      auto match_rect = [](std::vector<int> body_rect,
                           std::vector<int> face_head_rect) -> bool {
        if (body_rect.size() < 4 || face_head_rect.size() < 4) return false;
        if (body_rect[0] <= face_head_rect[0] &&
            body_rect[1] <= face_head_rect[1] &&
            body_rect[2] >= face_head_rect[2] &&
            body_rect[3] >= face_head_rect[3]) {
          return true;
        }
        return false;
      };

      bool has_face = false;
      std::vector<int> face_rect;
      bool has_head = false;
      std::vector<int> head_rect;
      for (const auto &roi : target.rois) {
        if ("face" == roi.type) {
          has_face = true;
          face_rect.push_back(roi.rect.x_offset);
          face_rect.push_back(roi.rect.y_offset);
          face_rect.push_back(roi.rect.x_offset + roi.rect.width);
          face_rect.push_back(roi.rect.y_offset + roi.rect.height);
        }

        if ("head" == roi.type) {
          has_head = true;
          head_rect.push_back(roi.rect.x_offset);
          head_rect.push_back(roi.rect.y_offset);
          head_rect.push_back(roi.rect.x_offset + roi.rect.width);
          head_rect.push_back(roi.rect.y_offset + roi.rect.height);
        }

        if (has_face && has_head) {
          break;
        }
      }

      if (has_face && !track_info_.has_face_head) {
        auto &rect = face_rect;
        if (match_rect(track_info_.present_rect, rect)) {
          track_info_.has_face_head = true;
          break;
        }
      }
      if (has_head && !track_info_.has_face_head) {
        auto &rect = head_rect;
        if (match_rect(track_info_.present_rect, rect)) {
          track_info_.has_face_head = true;
          break;
        }
      }
    }

    if (track_info_.has_face_head) {
      track_info_.serial_lost_face_head_num = 0;
    } else {
      track_info_.serial_lost_face_head_num++;
    }
  }
}void TrackingManager::RunTrackLostProtectionStrategy() {
  if (track_info_.serial_lost_face_head_num >= 20) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track lost warnning! serial_lost_face_head_num: %d",
                track_info_.serial_lost_face_head_num);
    CancelMove();
  }
}

void TrackingManager::TrackingWithoutNavStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "Run TrackingWithoutNav strategy");

  if (!last_frame_done_) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "last frame is not done");
    return;
  }

  ProcessSmart(msg);

  last_frame_done_ = false;

  if (TrackingStatus::INITING == track_info_.tracking_sta) {
    CancelMove();
    last_frame_done_ = true;
    return;
  }

  uint64_t frame_ts = (msg->header.stamp.sec % 1000) * 1000 +
                      msg->header.stamp.nanosec / 1000 / 1000;
  {
    std::stringstream ss;
    ss << "track_id: " << track_info_.track_id
       << ", frame_ts: " << track_info_.frame_ts
       << ", tracking_sta: " << static_cast<int>(track_info_.tracking_sta)
       << ", y pixel to robot: "
       << (track_info_.present_rect.empty()
               ? -1
               : (track_cfg_.img_height - track_info_.present_rect[3]));
    if (!track_info_.present_rect.empty()) {
      ss << ", present_rect: " << track_info_.present_rect[0] << " "
         << track_info_.present_rect[1] << " " << track_info_.present_rect[2]
         << " " << track_info_.present_rect[3];
    }
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().data());
  }

  if (TrackingStatus::TRACKING != track_info_.tracking_sta) {
    RCLCPP_DEBUG(rclcpp::get_logger("TrackingManager"), "track is lost");
    CancelMove();
    track_info_.tracking_sta = TrackingStatus::INITING;
    last_frame_done_ = true;
    return;
  }

  if (frame_ts != track_info_.frame_ts) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "update smart fail! msg frame_ts %llu, track frame_ts %llu",
                frame_ts,
                track_info_.frame_ts);
    CancelMove();
    last_frame_done_ = true;
    return;
  }

  UpdateTrackAngle();

  RunTrackLostProtectionStrategy();
  DoRotateMove();
  last_frame_done_ = true;
  return;
}

void TrackingManager::RunTrackingStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  std::unique_lock<std::mutex> lg(robot_strategy_mtx_);

  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "Run TrackingStrategy");

  auto start_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();

  TrackingWithoutNavStrategy(msg);

  auto present_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "Run TrackingStrategy time ms cost: %llu",
              present_tracking_ts - start_tracking_ts);
}

void TrackingManager::FeedSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  if (!rclcpp::ok()) return;
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);
  smart_queue_.push(msg);
  if (smart_queue_.size() > queue_len_limit_) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "smart queue len exceed limit: %d",
                queue_len_limit_);
    smart_queue_.pop();
  }
  smart_queue_condition_.notify_one();
  lg.unlock();
}

std::vector<std::shared_ptr<rclcpp::Node>> TrackingManager::GetNodes() {
  std::vector<std::shared_ptr<rclcpp::Node>> node_ptrs;
  node_ptrs.push_back(param_node_);
  node_ptrs.push_back(robot_cmdvel_node_);
  return node_ptrs;
}

const TrackCfg &TrackingManager::GetTrackCfg() const { return track_cfg_; }