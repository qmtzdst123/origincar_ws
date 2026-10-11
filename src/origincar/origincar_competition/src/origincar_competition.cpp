#include "origincar_competition/origincar_competition.h"
#include <string>
#include <stdexcept> // For std::stoi exceptions
#include <cmath>     // For M_PI, sin, std::abs, std::min, std::max

CompleteControl::CompleteControl() : Node("complete_control_node"),
                                     is_teleoperation_active_(false),
                                     last_qr_code_command_(QR_NOTHING),
                                     current_behavior_state_(RobotBehaviorState::LINE_FOLLOWING),
                                     last_avoidance_turn_direction_(LastAvoidanceTurnDirection::NONE)
{
    last_line_detected_time_ = this->now();
    // 参数声明
    this->declare_parameter<double>("line_following_speed", 0.45);
    this->declare_parameter<double>("line_kp", 1.0 / 320.0);
    this->declare_parameter<double>("cone_avoidance_speed", 0.25);
    this->declare_parameter<double>("cone_detection_y_threshold", 220.0);
    this->declare_parameter<double>("cone_critical_y_threshold", 300.0);
    this->declare_parameter<double>("cone_avoidance_steering_gain", 1.2);
    this->declare_parameter<double>("cone_lateral_offset_threshold", 40.0); // 用于判断锥桶是否在“正前方”区域
    this->declare_parameter<double>("centered_cone_avoid_turn_bias", 1.0);  // 正前方锥桶默认向左避让 (1.0 for left, -1.0 for right)
    this->declare_parameter<double>("post_avoidance_forward_search_duration", 0.1); // 避障后向前短时直行
    this->declare_parameter<double>("post_avoidance_recovery_turn_duration", 0.0); // 0.0 表示无限恢复转向
    this->declare_parameter<double>("recovery_turn_linear_speed_ratio", 0.35);
    this->declare_parameter<double>("search_swing_frequency", 0.3);

    // 获取参数
    this->get_parameter("line_following_speed", line_following_speed_);
    this->get_parameter("line_kp", line_kp_);
    this->get_parameter("cone_avoidance_speed", cone_avoidance_speed_);
    this->get_parameter("cone_detection_y_threshold", cone_detection_y_threshold_);
    this->get_parameter("cone_critical_y_threshold", cone_critical_y_threshold_);
    this->get_parameter("cone_avoidance_steering_gain", cone_avoidance_steering_gain_);
    this->get_parameter("cone_lateral_offset_threshold", cone_lateral_offset_threshold_);
    this->get_parameter("centered_cone_avoid_turn_bias", centered_cone_avoid_turn_bias_);
    this->get_parameter("post_avoidance_forward_search_duration", post_avoidance_forward_duration_sec_);
    this->get_parameter("post_avoidance_recovery_turn_duration", post_avoidance_recovery_turn_duration_sec_);
    this->get_parameter("recovery_turn_linear_speed_ratio", recovery_turn_linear_speed_ratio_);
    this->get_parameter("search_swing_frequency", search_swing_frequency_);

    RCLCPP_INFO(this->get_logger(), "CompleteControl Node started.");
    RCLCPP_INFO(this->get_logger(), "  Centered cone avoid bias: %.1f (1=Left, -1=Right)", centered_cone_avoid_turn_bias_);
    RCLCPP_INFO(this->get_logger(), "  Post-Avoidance Forward Search: %.2fs", post_avoidance_forward_duration_sec_);


    perception_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "/origincar_competition", 10, std::bind(&CompleteControl::handleAiMsg, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    follower_line_publisher_ = this->create_publisher<std_msgs::msg::Int32>("follower_line", 5);
    stop_publisher_ = this->create_publisher<std_msgs::msg::Int32>("stop", 1);
    sign_to_upper_computer_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
    qr_code_raw_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sign", 10, std::bind(&CompleteControl::qrCodeRawCallback, this, std::placeholders::_1));
    teleop_signal_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/sign4return", 10, std::bind(&CompleteControl::teleopSignalCallback, this, std::placeholders::_1));
}

void CompleteControl::qrCodeRawCallback(const std_msgs::msg::String::SharedPtr msg)
{
    origincar_msg::msg::Sign sign_to_publish;
    std::string data_str = msg->data;
    bool valid_qr_command_received = false;
    try {
        //int number = std::stoi(data_str);
        int number = 2;
        if (number >= 1 && number <= 9999) {
            if (number % 2 != 0) {
                last_qr_code_command_ = QR_CLOCKWISE;
                sign_to_publish.sign_data = QR_CLOCKWISE;
            } else {
                last_qr_code_command_ = QR_ANTICLOCKWISE;
                sign_to_publish.sign_data = QR_ANTICLOCKWISE;
            }
            sign_to_upper_computer_pub_->publish(sign_to_publish);
            valid_qr_command_received = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "QR Number %d out of range.", number);
            last_qr_code_command_ = QR_NOTHING;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "QR data '%s' invalid: %s", data_str.c_str(), e.what());
        last_qr_code_command_ = QR_NOTHING;
    }

    if (valid_qr_command_received) {
        RCLCPP_INFO(this->get_logger(), "Valid QR command (%d) received. -> QR_CODE_WAITING (Stopping).", last_qr_code_command_);
        current_behavior_state_ = RobotBehaviorState::QR_CODE_WAITING;
        setSpeed(0.0, 0.0);
    }
}

void CompleteControl::teleopSignalCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data == SIG_START_TELEOPERATION) {
        RCLCPP_INFO(this->get_logger(), "Teleoperation STARTED.");
        is_teleoperation_active_ = true;
    } else if (msg->data == SIG_ENDING_TELEOPERATION) {
        RCLCPP_INFO(this->get_logger(), "Teleoperation ENDED. Attempting to resume LINE_FOLLOWING.");
        // 进入任务3 默认向右避障
        line_following_speed_ = 0.6;
        centered_cone_avoid_turn_bias_ = -1;
        is_teleoperation_active_ = false;
        last_qr_code_command_ = QR_NOTHING;
        current_behavior_state_ = RobotBehaviorState::LINE_FOLLOWING;
    }
}

void CompleteControl::setSpeed(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_x;
    cmd_vel_msg.angular.z = angular_z;
    cmd_vel_publisher_->publish(cmd_vel_msg);
}

void CompleteControl::publishFollowerLineState(bool is_following) {
    std_msgs::msg::Int32 msg;
    msg.data = is_following ? 1 : 0;
    follower_line_publisher_->publish(msg);
}

void CompleteControl::publishHardStop() {
    std_msgs::msg::Int32 msg;
    msg.data = 0;
    stop_publisher_->publish(msg);
    RCLCPP_ERROR(this->get_logger(), "HARD STOP SIGNAL PUBLISHED!");
}

void CompleteControl::handleAiMsg(const ai_msgs::msg::PerceptionTargets::SharedPtr msg_ptr)
{
    // 0. 最高优先级检查
    if (last_qr_code_command_ != QR_NOTHING) {
        if (current_behavior_state_ != RobotBehaviorState::QR_CODE_WAITING) {
            current_behavior_state_ = RobotBehaviorState::QR_CODE_WAITING;
            setSpeed(0.0, 0.0);
        }
        publishFollowerLineState(false); return;
    }
    if (is_teleoperation_active_) {
        publishFollowerLineState(false); return;
    }

    // 收集感知数据
    std::vector<const ai_msgs::msg::Roi*> line_rois;
    std::vector<const ai_msgs::msg::Roi*> cone_rois;
    for (const auto &target : msg_ptr->targets) {
        if (!target.rois.empty()) {
            if (target.type == "line") line_rois.push_back(&target.rois[0]);
            else if (target.type == "zt") cone_rois.push_back(&target.rois[0]);
        }
    }

    double target_linear_x = 0.0;
    double target_angular_z = 0.0;
    bool is_cone_threat_detected_this_cycle = false;

    // 1. 锥桶威胁检测 (仅更新标志和状态，具体避障动作在状态机内)
    if (current_behavior_state_ != RobotBehaviorState::QR_CODE_WAITING && !is_teleoperation_active_) {
        if (!cone_rois.empty()) {
            float max_cone_height = 0.0f;
            for (const auto* cone_roi_ptr : cone_rois) {
                max_cone_height = std::max(max_cone_height, static_cast<float>(cone_roi_ptr->rect.height));
            }
            if (max_cone_height > cone_detection_y_threshold_) {
                is_cone_threat_detected_this_cycle = true;
                if (current_behavior_state_ != RobotBehaviorState::CONE_AVOIDANCE_ACTIVE) {
                    RCLCPP_INFO(this->get_logger(), "Cone threat (H:%.1f). State %d -> CONE_AVOIDANCE_ACTIVE", max_cone_height, static_cast<int>(current_behavior_state_));
                    current_behavior_state_ = RobotBehaviorState::CONE_AVOIDANCE_ACTIVE;
                    last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::NONE; // 进入时重置
                }
            }
        }
    }

    // 2. 核心状态机
    RobotBehaviorState next_behavior_state = current_behavior_state_;

    switch (current_behavior_state_)
    {
    case RobotBehaviorState::QR_CODE_WAITING:
        target_linear_x = 0.0; target_angular_z = 0.0; // 明确停车
        RCLCPP_DEBUG(this->get_logger(), "State: QR_CODE_WAITING");
        break;

    case RobotBehaviorState::LINE_FOLLOWING:
        if (is_cone_threat_detected_this_cycle) {
            // 状态已在上面切换到 CONE_AVOIDANCE_ACTIVE
        } else if (!line_rois.empty()) 
        {
            // std::sort(line_rois.begin(), line_rois.end(), 
            //     [](const ai_msgs::msg::Roi* a, const ai_msgs::msg::Roi* b) {
            //                 return a->rect.y_offset < b->rect.y_offset;
            //     });
            
            // auto closest_line_it = std::min_element(line_rois.begin(), line_rois.end(),
            //     [](const ai_msgs::msg::Roi* a, const ai_msgs::msg::Roi* b) { return a->rect.y_offset < b->rect.y_offset;});
            // const auto& target_line = **closest_line_it;
            
            // 更新最后检测到线的情况
            last_line_detected_time_ = this->now();
            is_first_line_lost_ = true;


            std::sort(line_rois.begin(), line_rois.end(), 
                [](const ai_msgs::msg::Roi* a, const ai_msgs::msg::Roi* b) {
                    return a->rect.y_offset < b->rect.y_offset;
                });

            const ai_msgs::msg::Roi* target_line_ptr = nullptr;

            if(line_rois.size() > 2) {
                // If more than 2 elements, take the second smallest (index 1)
                target_line_ptr = line_rois[1];
            } else if (!line_rois.empty()) {
                // Otherwise take the smallest element
                target_line_ptr = line_rois[0];
            }

            const auto& target_line = *target_line_ptr;

            
            float line_center_x = static_cast<float>(target_line.rect.x_offset + target_line.rect.width / 2.0f);
            float line_following_error = line_center_x - IMAGE_CENTER_X;

            last_valid_line_error_ = line_following_error;  // 保存有效误差

            target_linear_x = line_following_speed_;
            target_angular_z = -line_kp_ * line_following_error;
            last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::NONE;
        } else { // 巡线时丢线，开始向前搜索
            // RCLCPP_WARN(this->get_logger(), "LINE_FOLLOWING -> No line! -> POST_AVOIDANCE_SEARCH_FORWARD.");
            // next_behavior_state = RobotBehaviorState::POST_AVOIDANCE_SEARCH_FORWARD;
            // state_transition_timestamp_ = this->now();
            // last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::NONE;

                 // 丢线处理：使用上次有效线路信息恢复
                rclcpp::Duration line_lost_duration = this->now() - last_line_detected_time_;
        
                if (line_lost_duration.seconds() < 1.0) 
                { // 1秒内尝试恢复
                    target_linear_x = 0.6;
                    target_angular_z = -line_kp_ * last_valid_line_error_;
                    RCLCPP_WARN(this->get_logger(), "LINE_FOLLOWING -> Using last valid line (error: %.2f)", last_valid_line_error_);
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "LINE_FOLLOWING -> No line! -> RECOVERY_TURN.");
                    next_behavior_state = RobotBehaviorState::LINE_RECOVERY_TURN;
                    state_transition_timestamp_ = this->now();
                    
                    // 根据最后有效误差确定转向方向
                    if (last_valid_line_error_ > 0) {
                        last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::LEFT;
                    } else {
                        last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::RIGHT;
                    }
                }
        }
        break;
    case RobotBehaviorState::LINE_RECOVERY_TURN:
    if (!line_rois.empty()) {
        RCLCPP_INFO(this->get_logger(), "Found line during RECOVERY_TURN -> LINE_FOLLOWING.");
        next_behavior_state = RobotBehaviorState::LINE_FOLLOWING;
        last_line_detected_time_ = this->now(); // 重置检测时间
    } else {
        rclcpp::Duration time_since_recovery_start = this->now() - state_transition_timestamp_;
        
        // if (time_since_recovery_start.seconds() > 10.0) {
        //     RCLCPP_ERROR(this->get_logger(), "RECOVERY_TURN timeout! -> SAFE_STOP");
        //     next_behavior_state = RobotBehaviorState::SAFE_STOP;
        // } else {
            target_linear_x = 0.4;
            
            // 根据最后有效线路误差确定转向方向
            if (last_avoidance_turn_direction_ == LastAvoidanceTurnDirection::LEFT || 
                last_valid_line_error_ > 0) {
                target_angular_z = -1.0; // 向右转寻找线路
                RCLCPP_INFO(this->get_logger(), "LINE_RECOVERY_TURN: Turning RIGHT");
            } else {
                target_angular_z = 1.0; // 向左转寻找线路
                RCLCPP_INFO(this->get_logger(), "LINE_RECOVERY_TURN: Turning LEFT");
            }
        // }
    }
    break;
    case RobotBehaviorState::CONE_AVOIDANCE_ACTIVE:
        if (is_cone_threat_detected_this_cycle && !cone_rois.empty()) {
            const ai_msgs::msg::Roi* most_threatening_cone = nullptr;
            float max_cone_height = 0.0f;
            for (const auto* cone_roi_ptr : cone_rois) {
                float current_h = static_cast<float>(cone_roi_ptr->rect.height);
                if (current_h > max_cone_height) {
                    max_cone_height = current_h;
                    most_threatening_cone = cone_roi_ptr;
                }
            }
            if(most_threatening_cone){
                const auto& cone = *most_threatening_cone;
                float cone_center_x = static_cast<float>(cone.rect.x_offset + cone.rect.width / 2.0f);
                target_linear_x = cone_avoidance_speed_;
                if (max_cone_height > cone_critical_y_threshold_) target_linear_x = cone_avoidance_speed_;
                float lateral_error_to_cone = cone_center_x - IMAGE_CENTER_X;

                if (std::abs(lateral_error_to_cone) < cone_lateral_offset_threshold_) { // 锥桶在正前方区域
                    target_angular_z = cone_avoidance_steering_gain_ * centered_cone_avoid_turn_bias_;
                    if (centered_cone_avoid_turn_bias_ >= 0) last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::LEFT;
                    else last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::RIGHT;
                    RCLCPP_INFO(this->get_logger(), "CONE_AVOIDANCE: Centered cone. Bias turn: %.1f. LastTurn: %d", centered_cone_avoid_turn_bias_, static_cast<int>(last_avoidance_turn_direction_));

                    if (max_cone_height > cone_critical_y_threshold_) {
                        target_linear_x = std::min(target_linear_x, 0.05);
                        target_angular_z = cone_avoidance_steering_gain_ * centered_cone_avoid_turn_bias_ * 1.5; 
                    }
                } else if (lateral_error_to_cone < 0) { // 锥桶在图像左侧，车应向右转
                    target_angular_z = -cone_avoidance_steering_gain_;
                    last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::RIGHT;
                } else { // 锥桶在图像右侧，车应向左转
                    target_angular_z = cone_avoidance_steering_gain_;
                    last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::LEFT;
                }
                RCLCPP_DEBUG(this->get_logger(), "CONE_AVOIDANCE_ACTIVE: H:%.1f, LX:%.2f, AZ:%.2f", max_cone_height, target_linear_x, target_angular_z);
            } else {
                 is_cone_threat_detected_this_cycle = false;
                 target_linear_x = 0.0; target_angular_z = 0.0;
            }
        }
        // 避障结束判断
        if (!is_cone_threat_detected_this_cycle) {
            RCLCPP_INFO(this->get_logger(), "Exiting CONE_AVOIDANCE_ACTIVE. Last turn: %d.", static_cast<int>(last_avoidance_turn_direction_));
            if (!line_rois.empty()) {
                RCLCPP_INFO(this->get_logger(), "Line DETECTED immediately after avoidance. -> LINE_FOLLOWING.");
                next_behavior_state = RobotBehaviorState::LINE_FOLLOWING;
            } else { // 无线，则根据设置决定下一步
                if (post_avoidance_forward_duration_sec_ > 015) {
                    next_behavior_state = RobotBehaviorState::POST_AVOIDANCE_SEARCH_FORWARD;
                    RCLCPP_INFO(this->get_logger(), "-> POST_AVOIDANCE_SEARCH_FORWARD.");
                } else {
                    next_behavior_state = RobotBehaviorState::POST_AVOIDANCE_RECOVERY_TURN;
                    RCLCPP_INFO(this->get_logger(), "-> POST_AVOIDANCE_RECOVERY_TURN (Skipping/short forward search).");
                }
                state_transition_timestamp_ = this->now();
            }
        }
        break;

    case RobotBehaviorState::POST_AVOIDANCE_SEARCH_FORWARD:
        if (!line_rois.empty()) {
            RCLCPP_INFO(this->get_logger(), "Found line during FORWARD search -> LINE_FOLLOWING.");
            next_behavior_state = RobotBehaviorState::LINE_FOLLOWING;
        } else {
            rclcpp::Duration time_since_search_start = this->now() - state_transition_timestamp_;
            if (time_since_search_start.seconds() < post_avoidance_forward_duration_sec_) {
                target_linear_x = line_following_speed_ * recovery_turn_linear_speed_ratio_;
                target_angular_z = 0.0; // 直行
                RCLCPP_INFO(this->get_logger(), "State: POST_AVOIDANCE_SEARCH_FORWARD (%.1fs / %.1fs). LX:%.2f", time_since_search_start.seconds(), post_avoidance_forward_duration_sec_, target_linear_x);
            } else { // 超时
                RCLCPP_INFO(this->get_logger(), "FORWARD search timeout -> POST_AVOIDANCE_RECOVERY_TURN.");
                next_behavior_state = RobotBehaviorState::POST_AVOIDANCE_RECOVERY_TURN;
                state_transition_timestamp_ = this->now();
            }
        }
        break;

    case RobotBehaviorState::POST_AVOIDANCE_RECOVERY_TURN:
        if (!line_rois.empty()) {
            RCLCPP_INFO(this->get_logger(), "Found line during RECOVERY_TURN -> LINE_FOLLOWING.");
            next_behavior_state = RobotBehaviorState::LINE_FOLLOWING;
        } else {
            rclcpp::Duration time_since_recovery_start = this->now() - state_transition_timestamp_;
            if (post_avoidance_recovery_turn_duration_sec_ > 0.0 && time_since_recovery_start.seconds() > post_avoidance_recovery_turn_duration_sec_) {
                RCLCPP_WARN(this->get_logger(), "RECOVERY_TURN timeout (%.1fs). LastTurn: %d. -> POST_AVOIDANCE_SEARCH_FORWARD (retry).", post_avoidance_recovery_turn_duration_sec_, static_cast<int>(last_avoidance_turn_direction_));
                next_behavior_state = RobotBehaviorState::POST_AVOIDANCE_SEARCH_FORWARD;
                state_transition_timestamp_ = this->now();
                last_avoidance_turn_direction_ = LastAvoidanceTurnDirection::NONE;
            } else {
                target_linear_x = 0.6;
                if (last_avoidance_turn_direction_ == LastAvoidanceTurnDirection::LEFT) { // 上次车向左转避障 (意味着向右恢复)
                    target_angular_z = -1.25; // 车向右转
                    RCLCPP_INFO(this->get_logger(), "State: RECOVERY_TURN (Last avoid LEFT, now turning RIGHT). AZ: %.2f", target_angular_z);
                } else if (last_avoidance_turn_direction_ == LastAvoidanceTurnDirection::RIGHT) { // 上次车向右转避障 (意味着向左恢复)
                    target_angular_z = 1.25;  // 车向左转
                    RCLCPP_INFO(this->get_logger(), "State: RECOVERY_TURN (Last avoid RIGHT, now turning LEFT). AZ: %.2f", target_angular_z);
                } else { // 无明确上次转向，执行S型摆动
                    target_angular_z = 1.0;  // 车向左转
                    RCLCPP_INFO(this->get_logger(), "State: RECOVERY_TURN (Last avoid RIGHT, now turning LEFT). AZ: %.2f", target_angular_z);
                }
            }
        }
        break;
    }
    current_behavior_state_ = next_behavior_state;

    publishFollowerLineState(current_behavior_state_ == RobotBehaviorState::LINE_FOLLOWING);
    setSpeed(target_linear_x, target_angular_z);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompleteControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}