#ifndef STM32_NAV_COMM__STM32_COMM_NODE_H_
#define STM32_NAV_COMM__STM32_COMM_NODE_H_

#include <cstdint>                    // 固定宽度整数类型
#include <memory>                     // 智能指针
#include <string>                     // 字符串
#include <vector>                     // 动态数组

#include "geometry_msgs/msg/transform_stamped.hpp"  // 坐标变换消息
#include "geometry_msgs/msg/twist.hpp"              // 速度指令消息
#include "nav_msgs/msg/odometry.hpp"                // 里程计消息
#include "rclcpp/rclcpp.hpp"                        // ROS2核心头文件
#include "serial/serial.h"                          // 串口通信库
#include "std_msgs/msg/u_int8.hpp"                   // 单字节状态消息
#include "tf2_ros/transform_broadcaster.h"           // TF坐标广播

#pragma pack(push, 1)               // 1字节对齐，紧凑结构体
struct OdomFeedback                  // 底盘里程计反馈结构体
{
    uint32_t tick_ms;                // 系统时间戳 ms
    uint8_t control_mode;            // 控制模式
    uint8_t nav_state;               // 导航状态
    float omega_a;                  // A轮转速
    float omega_b;                  // B轮转速
    float omega_d;                  // D轮转速
};
#pragma pack(pop)                    // 恢复默认对齐

class Stm32CommNode : public rclcpp::Node  // ROS2节点类
{
public:
    Stm32CommNode();                 // 构造函数
    ~Stm32CommNode() override = default;  // 默认析构

private:
    static constexpr uint8_t FRAME_HEAD = 0xAA;  // 帧头
    static constexpr uint8_t FRAME_TAIL = 0x55;  // 帧尾

    static constexpr uint8_t CMD_VEL  = 0x01;    // 速度指令
    static constexpr uint8_t CMD_NAV  = 0x02;    // 导航指令
    static constexpr uint8_t CMD_ODOM = 0x04;    // 里程计指令
    static constexpr uint8_t CMD_MODE = 0x05;    // 模式切换指令

    static constexpr uint8_t MODE_MANUAL = 0;    // 手动模式
    static constexpr uint8_t MODE_ROS2   = 1;    // ROS2控制模式

    // 速度指令订阅回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pollSerial();                  // 轮询串口数据

    // 发送速度指令
    void sendCmdVel(float vx, float vy, float wz);
    void sendMode(uint8_t mode);        // 发送控制模式

    // 尝试解析一帧数据
    bool tryParseOneFrame(std::vector<uint8_t>& frame);
    void handleFrame(const std::vector<uint8_t>& frame);  // 处理完整帧
    void handleOdomFrame(const std::vector<uint8_t>& payload);  // 处理里程计帧

    // 根据轮速更新里程计
    void updateOdometryFromWheelSpeeds(
        double omega_a, double omega_b, double omega_d,
        const rclcpp::Time& now);

    void publishOdometry(const rclcpp::Time& stamp);  // 发布里程计
    void publishTransform(const rclcpp::Time& stamp); // 发布TF坐标变换

    // 计算校验和
    uint8_t calcChecksum(const std::vector<uint8_t>& data) const;

    // 模板：将数据按字节追加到帧
    template<typename T>
    void appendBytes(std::vector<uint8_t>& frame, const T& value)
    {
        const auto* ptr = reinterpret_cast<const uint8_t*>(&value);
        frame.insert(frame.end(), ptr, ptr + sizeof(T));
    }

private:
    std::string serial_port_;           // 串口设备名
    int baud_rate_;                     // 波特率
    std::string odom_frame_;            // 里程计坐标系
    std::string base_frame_;            // 底盘基坐标系
    bool publish_tf_;                   // 是否发布TF
    double wheel_radius_;                // 车轮半径
    double wheel_base_;                 // 轮距/轴距
    double cmd_timeout_sec_;            // 指令超时时间
    double odom_timeout_sec_;           // 里程计超时时间

    serial::Serial serial_;              // 串口对象

    // 速度指令订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;  // 里程计发布
    // 模式发布
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr stm32_mode_pub_;
    // 状态发布
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr stm32_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF广播器
    rclcpp::TimerBase::SharedPtr poll_timer_;       // 串口轮询定时器

    double x_;                          // X坐标
    double y_;                          // Y坐标
    double yaw_;                        // 航向角

    double vx_;                         // X方向线速度
    double vy_;                         // Y方向线速度
    double wz_;                         // 角速度

    rclcpp::Time last_odom_ros_time_;   // 上次里程计时间
    rclcpp::Time last_cmd_time_;        // 上次指令时间

    uint8_t last_control_mode_;         // 上次控制模式
    uint8_t last_nav_state_;            // 上次导航状态
    uint32_t last_tick_ms_;             // 上次硬件时间戳

    std::vector<uint8_t> rx_buffer_;    // 接收缓冲区
};

#endif  // STM32_NAV_COMM__STM32_COMM_NODE_H_

