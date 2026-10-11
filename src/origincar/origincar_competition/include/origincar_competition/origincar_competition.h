#ifndef __ORIGINCAR_COMPETITION_H__
#define __ORIGINCAR_COMPETITION_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "origincar_msg/msg/sign.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <cmath> // For M_PI, sin

// 二维码相关定义
#define QR_NOTHING 0
#define QR_CLOCKWISE 3
#define QR_ANTICLOCKWISE 4

// 上位机信号相关定义
#define SIG_NORMAL 0
#define SIG_START_TELEOPERATION 5
#define SIG_ENDING_TELEOPERATION 6

// 图像属性
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const int IMAGE_CENTER_X = IMAGE_WIDTH / 2;

// 机器人行为状态枚举
enum class RobotBehaviorState {
    LINE_FOLLOWING,                 // 巡线状态
    LINE_RECOVERY_TURN,             // 巡线丢线恢复状态
    CONE_AVOIDANCE_ACTIVE,          // 正在主动避障状态
    POST_AVOIDANCE_SEARCH_FORWARD,  // 避障后，向前直线搜索线
    POST_AVOIDANCE_RECOVERY_TURN,   // 直线搜索失败后，根据上次避障方向进行反向恢复转向搜索
    QR_CODE_WAITING                 // 接收到二维码指令，停车等待遥操作
};

// 记录避障时的主要转向方向
enum class LastAvoidanceTurnDirection {
    NONE,  // 未记录或不明确
    LEFT,  // 避障时车向左转了 (意味着锥桶在右侧或正前方偏右)
    RIGHT  // 避障时车向右转了 (意味着锥桶在左侧或正前方偏左)
};

class CompleteControl : public rclcpp::Node
{
public:
    CompleteControl();
    ~CompleteControl() = default;

private:
    // 回调函数
    void handleAiMsg(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
    void qrCodeRawCallback(const std_msgs::msg::String::SharedPtr msg);
    void teleopSignalCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // 控制与辅助函数
    void setSpeed(double linear_x, double angular_z);
    void publishFollowerLineState(bool is_following);
    void publishHardStop();

    // ROS 参数存储变量
    double line_following_speed_;
    double line_kp_;
    double cone_avoidance_speed_;
    double cone_detection_y_threshold_;
    double cone_critical_y_threshold_;
    double cone_avoidance_steering_gain_; // 也用作恢复转向和S型摆动的角速度幅度
    double cone_lateral_offset_threshold_; // 判断锥桶是否在“正前方”区域的阈值
    double centered_cone_avoid_turn_bias_; // 正前方锥桶避让偏向：1.0向左, -1.0向右
    double post_avoidance_forward_duration_sec_;
    double post_avoidance_recovery_turn_duration_sec_;
    double recovery_turn_linear_speed_ratio_;
    double search_swing_frequency_; // 用于当last_avoidance_turn_direction_为NONE时的S型摆动

    // 丢线恢复变量
    rclcpp::Time last_line_detected_time_;
    float last_valid_line_error_ = 0.0f;
    bool is_first_line_lost_ = true;

    // 状态变量
    bool is_teleoperation_active_;
    int last_qr_code_command_;
    RobotBehaviorState current_behavior_state_;
    rclcpp::Time state_transition_timestamp_;
    LastAvoidanceTurnDirection last_avoidance_turn_direction_;

    // ROS 通信接口
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr perception_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr follower_line_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stop_publisher_;
    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_to_upper_computer_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_code_raw_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr teleop_signal_sub_;
};

#endif // __ORIGINCAR_COMPETITION_H__