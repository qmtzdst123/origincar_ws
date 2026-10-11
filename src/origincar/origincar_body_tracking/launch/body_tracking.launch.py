# Copyright (c) 2024，D-Robotics.
# 融合自追踪包，适配OriginCar智能小车
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # 人体跟随节点（适配origincar_base）
    body_tracking_node = Node(
        package='origincar_body_tracking',
        executable='origincar_body_tracking',
        output='screen',
        parameters=[
            {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"},
            {"twist_pub_topic_name": "/cmd_vel"},  # 发布到origincar_base订阅的话题
            {"activate_wakeup_gesture": 0},  # 不启用手势唤醒，直接跟随
            {"track_body": 1},  # 跟踪人体
            {"img_width": 960},
            {"img_height": 544},
            {"track_serial_lost_num_thr": 30},  # 连续消失帧数阈值
            {"linear_velocity": 0.15},  # 前进速度
            {"angular_velocity": 0.3},  # 旋转速度
            {"activate_robot_rotate_thr": 20},  # 旋转激活角度阈值（度）
            {"activate_robot_move_thr": 5}  # 前进激活像素阈值
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 人体检测节点（需要根据实际环境配置）
    # 注意：这里假设人体检测节点已经单独启动，如果没有，需要添加人体检测节点的启动配置
    # mono2d_body_det_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('mono2d_body_detection'),
    #             'launch/mono2d_body_detection.launch.py')),
    #     launch_arguments={
    #         'smart_topic': '/hobot_mono2d_body_detection',
    #         'mono2d_body_pub_topic': '/hobot_mono2d_body_detection'
    #     }.items()
    # )

    return LaunchDescription([
        # mono2d_body_det_node,  # 如果需要人体检测，取消注释
        body_tracking_node
    ])