#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "socketcan.h"
#include "dbc_encoder_decoder.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <cstring>
// 用于发布标准里程计和TF变换
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <iomanip>
#include <clocale>
#include <chrono>

using namespace std::chrono_literals;

// 协方差矩阵参数（基于 1 RPM 底噪评估）
// 1 RPM -> 单轮线速度误差 ≈ 0.0056 m/s
// pose.covariance[0]  = var(x) 估算约 0.001（约 0.03 m/s 传播）
// twist.covariance[0] = var(vx) 直接来自速度误差 ≈ 0.0056^2/2 ≈ 0.00003，保守取 0.001
// 角速度方差：两轮误差叠加后 ω 误差 ≈ 0.037 rad/s，方差 ≈ 0.0014，保守取 0.005
// yaw 方差（积分累积）：保守取 0.01
static constexpr double ODOM_POSE_VAR_X   = 0.001;   // m^2
static constexpr double ODOM_POSE_VAR_Y   = 0.001;   // m^2
static constexpr double ODOM_POSE_VAR_YAW = 0.01;    // rad^2
static constexpr double ODOM_TWIST_VAR_VX = 0.001;   // (m/s)^2
static constexpr double ODOM_TWIST_VAR_WZ = 0.005;   // (rad/s)^2

static constexpr size_t PATH_MAX_POSES = 5000;        // 路径轨迹最大点数

class ChassisDriverNode : public rclcpp::Node {
public:
    ChassisDriverNode() : Node("chassis_driver_node"),
        odom_broadcaster_(this),
        static_broadcaster_(this)
    {
        RCLCPP_INFO(this->get_logger(), "can_node_start");

        // 声明并读取参数（支持 launch 文件传入）
        this->declare_parameter<double>("track_width",  0.3);
        this->declare_parameter<double>("wheel_radius", 0.0535);
        this->declare_parameter<double>("odom_rate",    100.0);    // Hz
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("path_max_poses", (int)PATH_MAX_POSES);

        track_width_  = this->get_parameter("track_width").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        double odom_rate = this->get_parameter("odom_rate").as_double();
        std::string can_iface = this->get_parameter("can_interface").as_string();
        path_max_poses_ = (size_t)this->get_parameter("path_max_poses").as_int();

        can_socket_ = initializeSocketCAN(can_iface.c_str());
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket on %s", can_iface.c_str());
            throw std::runtime_error("SocketCAN initialization failed");
        }

        // 订阅与发布
        control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ChassisDriverNode::controlCallback, this, std::placeholders::_1));

        can_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("can_status", 10);
        odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 50);
        path_pub_       = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        fault_pub_      = this->create_publisher<std_msgs::msg::UInt8MultiArray>("bms/fault_info", 10);
        soc_pub_        = this->create_publisher<std_msgs::msg::Float32>("bms/soc", 10);

        path_msg_.header.frame_id = "odom";

        // 定时器
        auto odom_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / odom_rate));
        rec_timer_ = this->create_wall_timer(odom_period,
            std::bind(&ChassisDriverNode::recCallback, this));

        status_timer_ = this->create_wall_timer(5s,
            std::bind(&ChassisDriverNode::timerCallback, this));

        bms_wakeup_timer_ = this->create_wall_timer(2s,
            std::bind(&ChassisDriverNode::bmsWakeupCallback, this));

        last_time_ = this->now();
        last_can_rec_time_ = this->now();

        initMotor();
    }

    ~ChassisDriverNode() {
        // 发送零速停止指令
        my_dbc_.composeSDO_Write4Byte(0x60FF, 0x03, 0x0000, can_id_, data_, dlc_);
        sendCANFrame(can_socket_, can_id_, data_, dlc_);
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
        RCLCPP_INFO(this->get_logger(), "ChassisDriverNode destroyed, CAN socket closed.");
    }

private:
    // ------ ROS 2 接口 ------
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr          can_status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           path_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr fault_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr         soc_pub_;
    rclcpp::TimerBase::SharedPtr rec_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr bms_wakeup_timer_;

    tf2_ros::TransformBroadcaster       odom_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    // ------ CAN 相关 ------
    dbc_encoder my_dbc_;
    uint8_t  data_[8];
    uint32_t can_id_;
    uint8_t  dlc_;
    int      can_socket_;
    BMS_Data bms_data_;

    // ------ 车体参数（可通过 launch 参数覆盖）------
    double track_width_  = 0.3;
    double wheel_radius_ = 0.0535;
    size_t path_max_poses_ = PATH_MAX_POSES;

    // ------ 里程计状态 ------
    double left_speed_  = 0.0;  // RPM
    double right_speed_ = 0.0;  // RPM
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

    rclcpp::Time last_time_;
    rclcpp::Time last_can_rec_time_;
    bool is_can_recv_valid_ = false;  // 只有成功读到有效帧才置 true
    nav_msgs::msg::Path path_msg_;
    bool is_command_received_ = false;

    // ------ 状态定时器回调 ------
    void timerCallback() {
        std_msgs::msg::Int32 status_msg;
        if (can_socket_ < 0) {
            status_msg.data = 1; // 初始化失败
        } else if ((this->now() - last_can_rec_time_).seconds() > 0.5) {
            status_msg.data = 2; // 丢包或断线超过 0.5s
        } else {
            status_msg.data = 0; // 正常
        }
        can_status_pub_->publish(status_msg);
    }

    // ------ 等待 SDO ACK ------
    bool sendCANFrame_WaitAck(uint32_t id, uint8_t* data, uint8_t len, int timeout_ms = 200) {
        if (!sendCANFrame(can_socket_, id, data, len)) return false;
        int elapsed = 0;
        while (elapsed < timeout_ms) {
            uint32_t rx_id;
            uint8_t  rx_data[8];
            uint8_t  rx_dlc;
            while (recvCANFrame(can_socket_, rx_id, rx_data, rx_dlc) > 0) {
                if (rx_id == (0x580 + ZLAC_NODE_ID)) {
                    if (rx_data[0] == 0x60 && rx_data[1] == 0x60) {
                        RCLCPP_INFO(this->get_logger(), "speed mode set success");
                        return true;
                    } else if (rx_data[0] == 0x80) {
                        RCLCPP_ERROR(this->get_logger(), "SDO Abort: 0x%02X%02X%02X%02X",
                            rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
                        return false;
                    } else if (rx_data[0] == 0x60 && rx_data[1] == 0x40) {
                        RCLCPP_INFO(this->get_logger(), "enable success");
                        return true;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            elapsed += 10;
        }
        RCLCPP_ERROR(this->get_logger(), "SDO Timeout (No Ack for ID 0x%X)", id);
        return false;
    }

    // ------ 速度指令回调 ------
    void controlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_x  = msg->linear.x;
        double angular_z = msg->angular.z;
        is_command_received_ = true;

        double v_l = linear_x - angular_z * track_width_ / 2.0;
        double v_r = linear_x + angular_z * track_width_ / 2.0;

        double rpm_l = (v_l * 60.0) / (2.0 * M_PI * wheel_radius_);
        double rpm_r = (v_r * 60.0) / (2.0 * M_PI * wheel_radius_);

        int16_t left_cmd  = (int16_t)rpm_l;
        int16_t right_cmd = (int16_t)(-rpm_r);  // 右轮取反（背夹安装）
        int32_t combined_cmd = ((uint16_t)right_cmd << 16) | (uint16_t)left_cmd;

        my_dbc_.composeSDO_Write4Byte(0x60FF, 0x03, combined_cmd, can_id_, data_, dlc_);
        sendCANFrame(can_socket_, can_id_, data_, dlc_);
    }

    double normalizeAngle(double angle) {
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ------ 主循环回调（里程计 + CAN 读取）------
    void recCallback() {
        // 1. 轮询电机速度 SDO Read 0x606C:03
        my_dbc_.composeSDO_Read(0x606C, 0x03, can_id_, data_, dlc_);
        sendCANFrame(can_socket_, can_id_, data_, dlc_);

        // 2. 非阻塞读取所有 CAN 帧
        uint32_t rx_id;
        uint8_t  rx_data[8];
        uint8_t  rx_dlc;
        bool got_valid_frame = false;

        while (recvCANFrame(can_socket_, rx_id, rx_data, rx_dlc) > 0) {
            got_valid_frame = true;
            if ((rx_id & CAN_SFF_MASK) == (uint32_t)(0x580 + ZLAC_NODE_ID)) {
                // SDO 响应：电机速度
                if (my_dbc_.parseSDO_Response(rx_data, rx_dlc, left_speed_, right_speed_)) {
                    std::cout << "motor_rpm(L,R): " << left_speed_ << ", " << right_speed_ << std::endl;
                }
            } else if (rx_id & CAN_EFF_FLAG) {
                // 扩展帧：BMS 数据
                struct can_frame bms_frame;
                bms_frame.can_id  = rx_id;
                bms_frame.can_dlc = rx_dlc;
                std::memcpy(bms_frame.data, rx_data, 8);
                my_dbc_.parseFrame(bms_frame, bms_data_);
            }
        }

        // 只有真正读到帧才更新心跳时间（修复原版 Bug：无论是否读到都更新导致断线检测失效）
        if (got_valid_frame) {
            last_can_rec_time_ = this->now();
        }

        // 3. 发布 SOC
        if (bms_data_.info0.soc > 0) {
            std_msgs::msg::Float32 soc_msg;
            soc_msg.data = bms_data_.info0.soc;
            soc_pub_->publish(soc_msg);
        }

        // 4. 发布故障信息
        std_msgs::msg::UInt8MultiArray fault_msg;
        fault_msg.data.insert(fault_msg.data.end(),
            bms_data_.fault_data.page1, bms_data_.fault_data.page1 + 8);
        fault_msg.data.insert(fault_msg.data.end(),
            bms_data_.fault_data.page2, bms_data_.fault_data.page2 + 8);
        fault_pub_->publish(fault_msg);

        uint64_t fault_code = bms_data_.fault_data.getFaultCode();
        std::string fault_str = bms_data_.fault_data.getFaultDescription();
        if (fault_code != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "BMS error: %s (Code: 0x%llX)", fault_str.c_str(), (unsigned long long)fault_code);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "BMS staus: normal (Code: 0x0)");
        }

        // 5. 计算里程计（差速运动学）
        double left_speed_ms  = (left_speed_  / 60.0 * 2.0 * M_PI * wheel_radius_);
        double right_speed_ms = (right_speed_ / 60.0 * 2.0 * M_PI * wheel_radius_);

        linear_velocity_  = (left_speed_ms + right_speed_ms) / 2.0;
        angular_velocity_ = (right_speed_ms - left_speed_ms) / track_width_;

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt < 0.0) return;

        // 中点积分法（2阶 Runge-Kutta）
        double delta_theta = angular_velocity_ * dt;
        double mid_theta   = theta_ + delta_theta / 2.0;
        double delta_x     = linear_velocity_ * std::cos(mid_theta) * dt;
        double delta_y     = linear_velocity_ * std::sin(mid_theta) * dt;

        x_     += delta_x;
        y_     += delta_y;
        theta_ += delta_theta;
        theta_  = normalizeAngle(theta_);
        last_time_ = current_time;

        // 打印 odom 状态
        std::cout << "odom(x,y,theta,v,omega): "
                  << x_ << ", " << y_ << ", " << theta_ << ", "
                  << linear_velocity_ << ", " << angular_velocity_ << std::endl;

        // 6. 构造四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

        // 7. 发布 odom -> base_link TF（真实动态 TF，让 Cartographer 接管）
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp    = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster_.sendTransform(odom_trans);  // ✅ 已启用（原版被注释）

        // 8. 发布 Odometry 消息（含协方差矩阵）
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x  = linear_velocity_;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = angular_velocity_;

        // ✅ 协方差矩阵（6x6，按行主序展开）
        // 索引: [0]=x, [7]=y, [14]=z, [21]=roll, [28]=pitch, [35]=yaw
        odom.pose.covariance[0]  = ODOM_POSE_VAR_X;    // var(x)
        odom.pose.covariance[7]  = ODOM_POSE_VAR_Y;    // var(y)
        odom.pose.covariance[14] = 1e-6;               // var(z) 2D场景极小
        odom.pose.covariance[21] = 1e-6;               // var(roll) 2D场景极小
        odom.pose.covariance[28] = 1e-6;               // var(pitch) 2D场景极小
        odom.pose.covariance[35] = ODOM_POSE_VAR_YAW;  // var(yaw)

        odom.twist.covariance[0]  = ODOM_TWIST_VAR_VX;  // var(vx)
        odom.twist.covariance[7]  = 1e-6;               // var(vy)
        odom.twist.covariance[14] = 1e-6;               // var(vz)
        odom.twist.covariance[21] = 1e-6;               // var(ωx)
        odom.twist.covariance[28] = 1e-6;               // var(ωy)
        odom.twist.covariance[35] = ODOM_TWIST_VAR_WZ;  // var(ωz)

        odom_pub_->publish(odom);

        // 9. 发布轨迹路径（限制最大点数，防止内存无限增长）
        geometry_msgs::msg::PoseStamped this_pose;
        this_pose.header.stamp    = current_time;
        this_pose.header.frame_id = "odom";
        this_pose.pose.position.x = x_;
        this_pose.pose.position.y = y_;
        this_pose.pose.position.z = 0.0;
        this_pose.pose.orientation = odom_quat;

        path_msg_.header.stamp    = current_time;
        path_msg_.header.frame_id = "odom";
        path_msg_.poses.push_back(this_pose);

        // ✅ 限制路径数组长度（原版无限增长 Bug 修复）
        if (path_msg_.poses.size() > path_max_poses_) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }

        path_pub_->publish(path_msg_);
    }

    // ------ BMS 唤醒心跳 ------
    void bmsWakeupCallback() {
        struct can_frame wakeup_frame;
        my_dbc_.encodeBMSControl(wakeup_frame);
        sendCANFrame(can_socket_, wakeup_frame);
    }

    // ------ 电机初始化（ZLAC8015D）------
    void initMotor() {
        RCLCPP_INFO(this->get_logger(), "Initializing ZLAC8015D...");

        my_dbc_.composeReserve(can_id_, data_, dlc_);
        sendCANFrame(can_socket_, can_id_, data_, dlc_);
        RCLCPP_INFO(this->get_logger(), "Sent 0x000 config frame. Waiting for drive internal mapping/reboot...");
        std::this_thread::sleep_for(2000ms);

        my_dbc_.composeSDO_Write1Byte(0x6060, 0x00, 3, can_id_, data_, dlc_);
        if (!sendCANFrame_WaitAck(can_id_, data_, dlc_)) {
            RCLCPP_WARN(this->get_logger(), "Failed to set Velocity Mode.");
        }
        std::this_thread::sleep_for(50ms);

        // CiA 402 状态机: Shutdown -> Switch On -> Enable Operation
        my_dbc_.composeSDO_Write2Byte(0x6040, 0x00, 0x06, can_id_, data_, dlc_);
        sendCANFrame_WaitAck(can_id_, data_, dlc_);
        std::this_thread::sleep_for(50ms);

        my_dbc_.composeSDO_Write2Byte(0x6040, 0x00, 0x07, can_id_, data_, dlc_);
        sendCANFrame_WaitAck(can_id_, data_, dlc_);
        std::this_thread::sleep_for(50ms);

        my_dbc_.composeSDO_Write2Byte(0x6040, 0x00, 0x0F, can_id_, data_, dlc_);
        sendCANFrame_WaitAck(can_id_, data_, dlc_);
        std::this_thread::sleep_for(50ms);

        RCLCPP_INFO(this->get_logger(), "ZLAC8015D Initialized success.");
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
