/*
 * chassis_node.cpp
 * 户外重载型四轮舵轮底盘 - ROS2 串口驱动节点
 *
 * 通讯协议 (USART1, 115200bps, 8N1):
 *   上行 STM32→PC: 0xAA 0xAA 0xF1 [len=120] [30×float BE] [checksum] = 125字节
 *   下行 PC→STM32: 0xAA 0xAA [50字节payload] [checksum] = 53字节
 *
 * 话题:
 *   订阅: cmd_vel (Twist), motor_enable (Bool), reset_odom (Empty)
 *   发布: odom (Odometry), imu (Imu), /tf (TFMessage)
 *
 * 参数: 见 config/params.yaml
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>
#include <cstring>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/* ---- 帧常量 ---- */
#define HEADER_UP1   0xAA
#define HEADER_UP2   0xAA
#define HEADER_UP3   0xF1
#define FRAME_UP_LEN 125
#define DATA_COUNT   30
#define DATA_OFFSET  4
#define FRAME_DN_LEN 53

#define TO_RAD  0.01745329252f
#define TO_DEG  57.295779513f

/*
 * 4轮舵轮逆运动学
 *
 * 这里的公式必须和 STM32 固件 control.c 中 XYZ 模式的分解方式一致，
 * 否则 Mode 0 (上位机解算) 和 Mode 1 (下位机解算) 同样的输入会产生不同运动。
 *
 * STM32 固件里的轮子位置和速度分解:
 *   A(前右): pos=(+L1_half, -W_half)  Vx=Vx+ω*W_half   Vy=Vy+ω*L1_half
 *   B(前左): pos=(+L1_half, +W_half)  Vx=Vx-ω*W_half   Vy=Vy+ω*L1_half
 *   C(后左): pos=(-L1_half, +W_half)  Vx=Vx-ω*W_half   Vy=Vy-ω*L1_half
 *   D(后右): pos=(-L1_half, -W_half)  Vx=Vx+ω*W_half   Vy=Vy-ω*L1_half
 *
 * 其中 L1_half=wheelbase/2, W_half=track_width/2
 * ω 即 angular.z (rad/s)
 *
 * 注意: 和教科书上的 vix = vx - ω*pos_y 公式不同，
 * 这是因为 STM32 固件里 Y 轴方向和自己定义的轮子位置序号有关，
 * 这里直接按固件的公式写，保证两种模式行为一致。
 */
static void inverse_4w_steer(double vx, double vy, double vth,
                             double wheelbase_m, double track_width_m,
                             double speeds[4], double angles_deg[4])
{
    double hL = wheelbase_m / 2.0;   /* L1_half, 前后半轴距 */
    double hW = track_width_m / 2.0; /* W_half, 左右半轮距 */

    /* A(前右), B(前左), C(后左), D(后右) */
    double vix[4], viy[4];

    /* A轮 */
    vix[0] = vx + vth * hW;
    viy[0] = vy + vth * hL;
    /* B轮 */
    vix[1] = vx - vth * hW;
    viy[1] = vy + vth * hL;
    /* C轮 */
    vix[2] = vx - vth * hW;
    viy[2] = vy - vth * hL;
    /* D轮 */
    vix[3] = vx + vth * hW;
    viy[3] = vy - vth * hL;

    for (int i = 0; i < 4; i++) {
        speeds[i]    = std::hypot(vix[i], viy[i]);
        angles_deg[i] = std::atan2(viy[i], vix[i]) * TO_DEG;
    }
}

/* ---- 串口 ---- */
static int  serial_fd = -1;
static bool keep_running = true;

static uint8_t sum_checksum(const uint8_t *data, int len) {
    uint16_t s = 0;
    for (int i = 0; i < len; i++) s += data[i];
    return (uint8_t)(s & 0xFF);
}

/* 大端序 float ↔ 4字节 */
static float b2f(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t raw = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16)
                 | ((uint32_t)b2 << 8)  |  (uint32_t)b3;
    float f; std::memcpy(&f, &raw, sizeof(f)); return f;
}

static void f2b(float val, uint8_t *out) {
    uint32_t raw; std::memcpy(&raw, &val, sizeof(raw));
    out[0] = (raw >> 24) & 0xFF; out[1] = (raw >> 16) & 0xFF;
    out[2] = (raw >> 8)  & 0xFF; out[3] =  raw        & 0xFF;
}

static bool serial_open(const char *port, int baud) {
    serial_fd = open(port, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) return false;
    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_lflag = 0;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 10; /* 1秒超时单位, 10=1秒 */

    int speed = B115200;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tcflush(serial_fd, TCIFLUSH);
    tcsetattr(serial_fd, TCSANOW, &tio);
    return true;
}

static int  serial_available()       { int n=0; ioctl(serial_fd, FIONREAD, &n); return n; }
static int  serial_read(uint8_t *b, int len)  { return read(serial_fd, b, len); }
static int  serial_write(const uint8_t *b, int len) { return write(serial_fd, b, len); }
static void serial_close()           { if (serial_fd >= 0) close(serial_fd); }

/* ---- ROS2 节点 ---- */
class ChassisNode : public rclcpp::Node
{
public:
    ChassisNode() : Node("chassis_node")
    {
        /* 声明参数 */
        this->declare_parameter("serial_port",   "/dev/ttyUSB0");
        this->declare_parameter("serial_baud",   115200);
        this->declare_parameter("odom_frame",    "odom");
        this->declare_parameter("base_frame",    "base_link");
        this->declare_parameter("publish_rate",  20.0);
        this->declare_parameter("control_mode",  1);
        this->declare_parameter("wheelbase",     0.39);
        this->declare_parameter("track_width",   0.44);
        this->declare_parameter("start_flag",    1);
        this->declare_parameter("cmd_timeout_ms", 500);

        motor_enabled_ = this->get_parameter("start_flag").as_int() != 0;

        /* 发布者 */
        odom_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        imu_pub_    = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        /* 订阅者 */
        using std::placeholders::_1;
        cmd_sub_   = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ChassisNode::cmd_vel_cb, this, _1));
        motor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "motor_enable", 10, std::bind(&ChassisNode::motor_enable_cb, this, _1));
        reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "reset_odom", 10, std::bind(&ChassisNode::reset_odom_cb, this, _1));

        /* 定时发布 odom/tf/imu + 下发控制帧 */
        double rate = this->get_parameter("publish_rate").as_double();
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&ChassisNode::publish_odom_tf, this));

        /* 打开串口 */
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("serial_baud").as_int();
        ser_connected_ = serial_open(port.c_str(), baud);
        if (ser_connected_) {
            RCLCPP_INFO(this->get_logger(), "[OK] 串口已连接: %s @ %d", port.c_str(), baud);
        } else {
            RCLCPP_WARN(this->get_logger(), "[WARN] 串口打开失败: %s", port.c_str());
        }

        /* 启动时发一帧模式1零速度，让STM32进入串口控制 */
        if (ser_connected_) {
            send_mode1(0, 0, 0, true);
        }
    }

    ~ChassisNode() {
        keep_running = false;
        if (ser_connected_) {
            send_mode1(0, 0, 0, false);  /* 停车关电机 */
            serial_close();
        }
    }

    void run() {
        rclcpp::Rate loop_rate(150);
        while (rclcpp::ok() && keep_running) {
            if (ser_connected_) receive_and_process();
            rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
        }
    }

private:
    /* 解析后的上行数据 */
    float    data_ur_[30];
    double   angles_[4]  = {0}, speeds_[4] = {0};
    double   gyro_[3]    = {0}, accel_[3]   = {0}, rpy_[3] = {0};
    double   voltage_    = 0, odom_x_ = 0, odom_y_ = 0, odom_th_ = 0;
    int      start_flag_ = 0;
    bool     ser_connected_ = false, frame_valid_ = false;
    uint16_t rx_cnt_ = 0, ok_cnt_ = 0;

    /* cmd_vel 缓存 + 超时 */
    double cmd_vx_ = 0, cmd_vy_ = 0, cmd_vth_ = 0;
    rclcpp::Time last_cmd_time_;
    bool motor_enabled_    = true;
    bool reset_odom_pending_ = false;

    /* ROS2 接口 */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* ---- 回调 ---- */
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vx_  = msg->linear.x;
        cmd_vy_  = msg->linear.y;
        cmd_vth_ = msg->angular.z;
        last_cmd_time_ = this->now();
    }

    void motor_enable_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        motor_enabled_ = msg->data;
        if (!motor_enabled_) { cmd_vx_ = cmd_vy_ = cmd_vth_ = 0; }
    }

    void reset_odom_cb(const std_msgs::msg::Empty::SharedPtr) {
        reset_odom_pending_ = true;
    }

    /*
     * 下行帧: Mode 1 — XYZ 速度模式
     * STM32 收到 Flag_Mode=1 后自己做运动学解算
     */
    void send_mode1(float vx, float vy, float vth, bool reset_odom = false) {
        uint8_t buf[FRAME_DN_LEN];
        std::memset(buf, 0, FRAME_DN_LEN);
        buf[0] = 0xAA;
        buf[1] = 0xAA;
        /* buf[2] 和 buf[3] 填0, STM32的ISR会存入但不参与数据解析 */

        int off = 4;
        float start = motor_enabled_ ? 1.0f : 0.0f;
        f2b(start,  buf + off); off += 4;  /* byte 4-7:  启停标志 */
        f2b(1.0f,   buf + off); off += 4;  /* byte 8-11: Flag_Mode=1 */
        f2b(vx,     buf + off); off += 4;  /* byte 12-15: Velocity_X (m/s) */
        f2b(vy,     buf + off); off += 4;  /* byte 16-19: Velocity_Y (m/s) */
        f2b(vth,    buf + off); off += 4;  /* byte 20-23: Velocity_Z (rad/s) */
        /* byte 24-43: 填0 (Mode 1 不需要独立轮参数) */

        f2b(reset_odom ? 0.0f : 1.0f, buf + 44);  /* byte 44-47: 0=复位里程计, 1=保持 */
        /* byte 48-51: 填0 */

        buf[52] = sum_checksum(buf, 52);
        serial_write(buf, FRAME_DN_LEN);
    }

    /*
     * 下行帧: Mode 0 — 独立轮控模式
     * 上位机计算每个轮的舵角(deg)和线速度(m/s), STM32直接执行
     * 注意: 逆运动学用的是和STM32固件一致的公式
     */
    void send_mode0(double angles[4], double speeds[4], bool reset_odom = false) {
        uint8_t buf[FRAME_DN_LEN];
        std::memset(buf, 0, FRAME_DN_LEN);
        buf[0] = 0xAA;
        buf[1] = 0xAA;

        int off = 4;
        float start = motor_enabled_ ? 1.0f : 0.0f;
        f2b(start,  buf + off); off += 4;  /* byte 4-7:  启停标志 */
        f2b(0.0f,   buf + off); off += 4;  /* byte 8-11: Flag_Mode=0 */
        f2b((float)angles[0], buf + off); off += 4;  /* byte 12-15: Angle_A (deg) */
        f2b((float)angles[1], buf + off); off += 4;  /* byte 16-19: Angle_B */
        f2b((float)angles[2], buf + off); off += 4;  /* byte 20-23: Angle_C */
        f2b((float)angles[3], buf + off); off += 4;  /* byte 24-27: Angle_D */
        f2b((float)speeds[0], buf + off); off += 4;  /* byte 28-31: Speed_A (m/s) */
        f2b((float)speeds[1], buf + off); off += 4;  /* byte 32-35: Speed_B */
        f2b((float)speeds[2], buf + off); off += 4;  /* byte 36-39: Speed_C */
        f2b((float)speeds[3], buf + off); off += 4;  /* byte 40-43: Speed_D */

        f2b(reset_odom ? 0.0f : 1.0f, buf + 44);

        buf[52] = sum_checksum(buf, 52);
        serial_write(buf, FRAME_DN_LEN);
    }

    /* ---- 接收上行帧 ---- */
    void receive_and_process() {
        static uint8_t buf[256];
        int n = serial_available();
        if (n <= 0) return;
        rx_cnt_++;

        /* 缓冲区积压太多就清掉, 防止读太老的帧 */
        if (n >= 250) {
            uint8_t dump[128];
            while (serial_available() >= FRAME_UP_LEN) serial_read(dump, 128);
            return;
        }

        if (n >= FRAME_UP_LEN) {
            /* 先找帧头 0xAA */
            while (n > 0) {
                uint8_t b;
                serial_read(&b, 1);
                n--;
                if (b == HEADER_UP1) { buf[0] = b; break; }
            }
            if (n <= 0 || buf[0] != HEADER_UP1) return;

            /* 读剩下的124字节 */
            int rd = serial_read(buf + 1, FRAME_UP_LEN - 1);
            if (rd < FRAME_UP_LEN - 1) return;

            /* 验证帧头和校验和 */
            if (buf[1] == HEADER_UP2 && buf[2] == HEADER_UP3) {
                if (sum_checksum(buf, FRAME_UP_LEN - 1) == buf[FRAME_UP_LEN - 1]) {
                    ok_cnt_++;
                    frame_valid_ = true;
                    parse_frame(buf);
                }
            }
        }
    }

    /* 解析30个float, 按STM32上行帧定义 */
    void parse_frame(const uint8_t *frame) {
        for (int i = 0; i < DATA_COUNT; i++) {
            int off = DATA_OFFSET + i * 4;
            data_ur_[i] = b2f(frame[off], frame[off+1], frame[off+2], frame[off+3]);
        }
        start_flag_ = (int)data_ur_[0];

        /* 四轮舵角和线速度 */
        for (int i = 0; i < 4; i++) {
            angles_[i] = data_ur_[1 + i];   /* deg */
            speeds_[i] = data_ur_[5 + i];   /* m/s */
        }
        /* 陀螺仪、加速度、姿态角 */
        for (int i = 0; i < 3; i++) {
            gyro_[i]  = data_ur_[9  + i];  /* deg/s */
            accel_[i] = data_ur_[12 + i];  /* raw */
            rpy_[i]   = data_ur_[15 + i];  /* deg */
        }
        /* 电压 (单位10mV, 除以100得到V) */
        voltage_ = data_ur_[18] / 100.0f;
        /* 里程计 (STM32融合输出) */
        odom_x_  = data_ur_[19];  /* m */
        odom_y_  = data_ur_[20];  /* m */
        odom_th_ = data_ur_[21];  /* rad */
    }

    /* ---- 定时器回调: 发布odom/tf/imu + 下发控制帧 ---- */
    void publish_odom_tf() {
        if (!frame_valid_) return;

        /* cmd_vel 超时保护: 超过指定时间没收到新指令就自动停车 */
        auto now = this->now();
        int timeout_ms = this->get_parameter("cmd_timeout_ms").as_int();
        if (last_cmd_time_.nanoseconds() > 0) {
            auto dt = now - last_cmd_time_;
            if (dt.seconds() * 1000.0 > timeout_ms) {
                cmd_vx_ = cmd_vy_ = cmd_vth_ = 0;
            }
        }

        /* 里程计复位信号 (只在收到reset_odom话题消息时触发一次) */
        bool do_reset = reset_odom_pending_;
        if (do_reset) reset_odom_pending_ = false;

        /* 根据控制模式下发下行帧 */
        if (ser_connected_) {
            int mode = this->get_parameter("control_mode").as_int();
            if (mode == 0) {
                /* Mode 0: 上位机做逆运动学, 把cmd_vel分解成4轮角度+速度 */
                double wb = this->get_parameter("wheelbase").as_double();
                double tw = this->get_parameter("track_width").as_double();
                double ang[4], spd[4];
                inverse_4w_steer(cmd_vx_, cmd_vy_, cmd_vth_, wb, tw, spd, ang);
                send_mode0(ang, spd, do_reset);
            } else {
                /* Mode 1: 直接发vx/vy/vth, STM32自己做解算 (默认模式) */
                send_mode1(cmd_vx_, cmd_vy_, cmd_vth_, do_reset);
            }
        }

        /* ---- 发布 Odometry ---- */
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp    = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";
        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;

        double yaw = odom_th_;  /* rad, 来自STM32融合 */
        odom_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

        /* 用4轮速度和舵角做正运动学, 估算底盘速度 */
        double vx_sum = 0, vy_sum = 0;
        for (int i = 0; i < 4; i++) {
            double a_rad = angles_[i] * TO_RAD;
            vx_sum += speeds_[i] * std::cos(a_rad);
            vy_sum += speeds_[i] * std::sin(a_rad);
        }
        odom_msg.twist.twist.linear.x  = vx_sum / 4.0;
        odom_msg.twist.twist.linear.y  = vy_sum / 4.0;
        odom_msg.twist.twist.angular.z = gyro_[2] * TO_RAD;
        odom_pub_->publish(odom_msg);

        /* ---- 发布 TF: odom → base_link ---- */
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = odom_x_;
        tf.transform.translation.y = odom_y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        /* ---- 发布 IMU ---- */
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp    = now;
        imu_msg.header.frame_id = "imu_link";

        double r = rpy_[0] * TO_RAD;
        double p = rpy_[1] * TO_RAD;
        double y = rpy_[2] * TO_RAD;

        double cr = std::cos(r/2), sr = std::sin(r/2);
        double cp = std::cos(p/2), sp = std::sin(p/2);
        double cy = std::cos(y/2), sy = std::sin(y/2);

        imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
        imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
        imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
        imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;

        imu_msg.angular_velocity.x = gyro_[0] * TO_RAD;
        imu_msg.angular_velocity.y = gyro_[1] * TO_RAD;
        imu_msg.angular_velocity.z = gyro_[2] * TO_RAD;

        /* 加速度: 原始值/16384*9.81 转成 m/s² */
        imu_msg.linear_acceleration.x = accel_[0] / 16384.0 * 9.81;
        imu_msg.linear_acceleration.y = accel_[1] / 16384.0 * 9.81;
        imu_msg.linear_acceleration.z = accel_[2] / 16384.0 * 9.81;
        imu_pub_->publish(imu_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}