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

#include <fstream>
#include <memory>
#include <sstream>

#include "body_tracking.h"

int main(int argc, char *argv[]) {
  std::stringstream ss;
  ss << "\n\tThis is OriginCar body tracking package.\n\n"
     << "\t功能说明:\n"
     << "\t- 通过视觉检测人体并自动跟随\n"
     << "\t- 选择视野中面积最大的人体作为跟随目标\n"
     << "\t- 根据人体位置控制小车旋转和前进\n"
     << "\t- 当人体消失时停止运动并寻找新目标\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto nodes = TrackingManager::Instance()->GetNodes();
  for (auto &node : nodes) {
    exec.add_node(node);
  }

  auto smart_msg_subscriber = std::make_shared<SmartMsgSubscriber>(
      "ai_msg_sub_node",
      std::bind(&TrackingManager::FeedSmart,
                TrackingManager::Instance(),
                std::placeholders::_1));

  exec.add_node(smart_msg_subscriber);
  exec.spin();

  // release node before shutdown!
  TrackingManager::Instance()->Release();

  rclcpp::shutdown();

  std::cout << "OriginCar body tracking node exit!\n";
  return 0;
}