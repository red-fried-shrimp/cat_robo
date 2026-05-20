#include "stm32_nav_comm/stm32_comm_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{
constexpr double kSqrt3 = 1.7320508075688772;    // √3 常数（麦克纳姆轮/三轮底盘计算用）
constexpr double kPi = 3.14159265358979323846;     // π 常数
}

// ==============================
// 构造函数：初始化参数、串口、订阅发布器
// ==============================
Stm32CommNode::Stm32CommNode()
: Node("stm32_comm_node"),
  x_(0.0),       // 里程计X坐标初始值
  y_(0.0),       // 里程计Y坐标初始值
  yaw_(0.0),     // 航向角初始值
  vx_(0.0),      // X方向线速度
  vy_(0.0),      // Y方向线速度
  wz_(0.0),      // 角速度
  last_control_mode_(MODE_MANUAL),  // 初始控制模式：手动
  last_nav_state_(0),               // 初始导航状态
  last_tick_ms_(0)                  // 初始时间戳
{
    // 声明并获取ROS参数（带默认值）
    serial_port_      = this->declare_parameter<std::string>("stm32_serial_port", "/dev/ttyACM1");
    baud_rate_        = this->declare_parameter<int>("stm32_baud_rate", 115200);
    odom_frame_       = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_       = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_       = this->declare_parameter<bool>("publish_tf", true);
    wheel_radius_     = this->declare_parameter<double>("wheel_radius", 0.042);
    wheel_base_       = this->declare_parameter<double>("wheel_base", 0.1977);
    cmd_timeout_sec_  = this->declare_parameter<double>("cmd_timeout_sec", 0.5);
    odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 0.5);

    // 打印参数信息
    RCLCPP_INFO(this->get_logger(), "stm32_serial_port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "stm32_baud_rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "wheel_radius: %.6f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "wheel_base: %.6f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "cmd_timeout_sec: %.3f", cmd_timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "odom_timeout_sec: %.3f", odom_timeout_sec_);

    // 订阅速度指令 /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 20,
        std::bind(&Stm32CommNode::cmdVelCallback, this, _1));

    // 发布里程计 /odom
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    // 发布控制模式
    stm32_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/stm32/control_mode", 10);
    // 发布导航状态
    stm32_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/stm32/nav_state", 10);

    // TF坐标广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 初始化时间戳
    last_odom_ros_time_ = this->now();
    last_cmd_time_ = this->now();

    try
    {
        // 配置并打开串口
        serial_.setPort(serial_port_);
        serial_.setBaudrate(static_cast<uint32_t>(baud_rate_));
        serial::Timeout to = serial::Timeout::simpleTimeout(20);
        serial_.setTimeout(to);
        serial_.open();

        if (serial_.isOpen())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "STM32 serial opened: %s @ %d",
                serial_port_.c_str(),
                baud_rate_);
        }
    }
    catch (const serial::IOException &e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to open STM32 serial %s: %s",
            serial_port_.c_str(),
            e.what());
    }

    // 5ms 定时器轮询串口数据
    poll_timer_ = this->create_wall_timer(
        5ms, std::bind(&Stm32CommNode::pollSerial, this));
}

// ==============================
// 计算异或校验和
// ==============================
uint8_t Stm32CommNode::calcChecksum(const std::vector<uint8_t>& data) const
{
    uint8_t checksum = 0;
    for (const auto byte : data)
    {
        checksum ^= byte;
    }
    return checksum;
}

// ==============================
// 发送控制模式（手动/ROS2）
// ==============================
void Stm32CommNode::sendMode(uint8_t mode)
{
    if (!serial_.isOpen())
    {
        return;
    }

    std::vector<uint8_t> frame;
    frame.reserve(6);

    frame.push_back(FRAME_HEAD);       // 帧头
    frame.push_back(CMD_MODE);         // 命令字
    frame.push_back(1);                // 数据长度
    frame.push_back(mode);             // 模式数据

    const uint8_t checksum = calcChecksum(frame);
    frame.push_back(checksum);         // 校验
    frame.push_back(FRAME_TAIL);       // 帧尾

    serial_.write(frame);
}

// ==============================
// 发送速度指令到STM32
// ==============================
void Stm32CommNode::sendCmdVel(float vx, float vy, float wz)
{
    if (!serial_.isOpen())
    {
        return;
    }

    std::vector<uint8_t> frame;
    frame.reserve(3 + 12 + 2);

    frame.push_back(FRAME_HEAD);
    frame.push_back(CMD_VEL);
    frame.push_back(12);  // 3个float = 12字节

    appendBytes(frame, vx);
    appendBytes(frame, vy);
    appendBytes(frame, wz);

    const uint8_t checksum = calcChecksum(frame);
    frame.push_back(checksum);
    frame.push_back(FRAME_TAIL);

    serial_.write(frame);
}

// ==============================
// /cmd_vel 回调：接收ROS速度指令并转发给STM32
// ==============================
void Stm32CommNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_time_ = this->now();

    // ROS坐标系 → STM32坐标系 转换
    // ROS:   x前+, y左+, z逆时针+
    // STM32: x右+, y前+, z逆时针+
    const float stm32_vx = static_cast<float>(-msg->linear.y);
    const float stm32_vy = static_cast<float>( msg->linear.x);
    const float stm32_wz = static_cast<float>( msg->angular.z);

    sendCmdVel(stm32_vx, stm32_vy, stm32_wz);
}

// ==============================
// 轮询串口：读数据、解析帧、超时处理
// ==============================
void Stm32CommNode::pollSerial()
{
    if (!serial_.isOpen())
    {
        return;
    }

    // 指令超时：自动发送0速度
    if ((this->now() - last_cmd_time_).seconds() > cmd_timeout_sec_)
    {
        sendCmdVel(0.0f, 0.0f, 0.0f);
        last_cmd_time_ = this->now();
    }

    try
    {
        const size_t n = serial_.available();
        if (n > 0)
        {
            const std::string data = serial_.read(n);
            rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.end());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Serial read exception: %s", e.what());
        return;
    }

    // 循环解析完整帧
    std::vector<uint8_t> frame;
    while (tryParseOneFrame(frame))
    {
        handleFrame(frame);
        frame.clear();
    }

    // 里程计接收超时警告
    if ((this->now() - last_odom_ros_time_).seconds() > odom_timeout_sec_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "No STM32 odom frame received for %.2f s",
            odom_timeout_sec_);
    }
}

// ==============================
// 从接收缓冲区解析一帧完整数据
// ==============================
bool Stm32CommNode::tryParseOneFrame(std::vector<uint8_t>& frame)
{
    // 找到帧头
    while (!rx_buffer_.empty() && rx_buffer_.front() != FRAME_HEAD)
    {
        rx_buffer_.erase(rx_buffer_.begin());
    }

    if (rx_buffer_.size() < 5)
    {
        return false;
    }

    const uint8_t len = rx_buffer_[2];
    const size_t full_len = static_cast<size_t>(len) + 5;

    if (rx_buffer_.size() < full_len)
    {
        return false;
    }

    // 检查帧尾
    if (rx_buffer_[full_len - 1] != FRAME_TAIL)
    {
        rx_buffer_.erase(rx_buffer_.begin());
        return false;
    }

    // 校验和检查
    std::vector<uint8_t> checksum_area(
        rx_buffer_.begin(), rx_buffer_.begin() + 3 + len);

    const uint8_t checksum = calcChecksum(checksum_area);
    const uint8_t recv_checksum = rx_buffer_[3 + len];

    if (checksum != recv_checksum)
    {
        rx_buffer_.erase(rx_buffer_.begin());
        return false;
    }

    // 提取一帧
    frame.assign(rx_buffer_.begin(), rx_buffer_.begin() + full_len);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + full_len);
    return true;
}

// ==============================
// 处理一帧数据（根据命令字分发）
// ==============================
void Stm32CommNode::handleFrame(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 5)
    {
        return;
    }

    const uint8_t cmd = frame[1];
    const uint8_t len = frame[2];

    std::vector<uint8_t> payload(frame.begin() + 3, frame.begin() + 3 + len);

    switch (cmd)
    {
        case CMD_ODOM:
            handleOdomFrame(payload);  // 处理里程计
            break;

        default:
            RCLCPP_DEBUG(this->get_logger(), "Unknown STM32 cmd: 0x%02X", cmd);
            break;
    }
}

// ==============================
// 处理里程计数据帧：解包、发布状态、更新里程计
// ==============================
void Stm32CommNode::handleOdomFrame(const std::vector<uint8_t>& payload)
{
    if (payload.size() != sizeof(OdomFeedback))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Invalid ODOM payload size: %zu, expected: %zu",
            payload.size(), sizeof(OdomFeedback));
        return;
    }

    OdomFeedback fb{};
    std::memcpy(&fb, payload.data(), sizeof(OdomFeedback));

    // 更新状态
    last_control_mode_ = fb.control_mode;
    last_nav_state_ = fb.nav_state;
    last_tick_ms_ = fb.tick_ms;

    // 发布模式与状态
    std_msgs::msg::UInt8 mode_msg;
    mode_msg.data = last_control_mode_;
    stm32_mode_pub_->publish(mode_msg);

    std_msgs::msg::UInt8 state_msg;
    state_msg.data = last_nav_state_;
    stm32_state_pub_->publish(state_msg);

    const rclcpp::Time now = this->now();

    // 根据轮速解算底盘里程计
    updateOdometryFromWheelSpeeds(
        static_cast<double>(fb.omega_a),
        static_cast<double>(fb.omega_b),
        static_cast<double>(fb.omega_d),
        now);

    last_odom_ros_time_ = now;

    // 打印日志（1秒1次）
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "STM32 odom: tick=%u mode=%u state=%u omega=[%.3f %.3f %.3f]",
        fb.tick_ms, fb.control_mode, fb.nav_state,
        fb.omega_a, fb.omega_b, fb.omega_d);
}

// ==============================
// 核心：三轮/麦克纳姆轮 逆运动学 → 计算底盘速度与里程计
// ==============================
void Stm32CommNode::updateOdometryFromWheelSpeeds(
    double omega_a, double omega_b, double omega_d,
    const rclcpp::Time& now)
{
    /*
      底盘运动学逆解
      transverse_right : 右为正
      ward_forward     : 前为正
      rotate_ccw       : 逆时针为正
    */

    const double r = wheel_radius_;
    const double L = wheel_base_;

    const double va = r * omega_a;
    const double vb = r * omega_b;
    const double vd = r * omega_d;

    // 底盘局部速度
    const double stm32_x = (2.0 * vd - va - vb) / 3.0;
    const double stm32_y = (vb - va) / kSqrt3;
    const double stm32_w = (va + vb + vd) / (3.0 * L);

    // 转换为ROS坐标系
    vx_ = stm32_y;         // ROS x 前
    vy_ = -stm32_x;        // ROS y 左
    wz_ = stm32_w;         // ROS z 逆时针

    // 时间差
    double dt = (now - last_odom_ros_time_).seconds();
    if (dt <= 0.0 || dt > 0.2)
    {
        dt = 0.02;  // 防止异常dt
    }

    // 坐标积分（世界坐标系）
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);

    const double vx_world = vx_ * cos_yaw - vy_ * sin_yaw;
    const double vy_world = vx_ * sin_yaw + vy_ * cos_yaw;

    x_ += vx_world * dt;
    y_ += vy_world * dt;
    yaw_ += wz_ * dt;

    // 角度归一化到 [-π, π]
    while (yaw_ > kPi)  yaw_ -= 2.0 * kPi;
    while (yaw_ < -kPi) yaw_ += 2.0 * kPi;

    // 发布里程计与TF
    publishOdometry(now);
    if (publish_tf_)
    {
        publishTransform(now);
    }
}

// ==============================
// 发布 /odom 里程计消息
// ==============================
void Stm32CommNode::publishOdometry(const rclcpp::Time& stamp)
{
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odom_msg.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = wz_;

    // 协方差（固定值）
    odom_msg.pose.covariance[0] = 0.02;
    odom_msg.pose.covariance[7] = 0.02;
    odom_msg.pose.covariance[35] = 0.05;

    odom_msg.twist.covariance[0] = 0.02;
    odom_msg.twist.covariance[7] = 0.02;
    odom_msg.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom_msg);
}

// ==============================
// 发布 TF：odom → base_link
// ==============================
void Stm32CommNode::publishTransform(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = -x_;
    tf_msg.transform.translation.y = -y_;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = std::sin(yaw_ * 0.5);
    tf_msg.transform.rotation.w = std::cos(yaw_ * 0.5);

    tf_broadcaster_->sendTransform(tf_msg);
}

// ==============================
// main 函数：节点入口
// ==============================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Stm32CommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

