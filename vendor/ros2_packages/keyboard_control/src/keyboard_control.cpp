/*
 * keyboard_control.cpp
 * 底盘键盘遥控节点
 *
 * 读取终端按键 → 发布 cmd_vel / motor_enable / reset_odom
 * 方向定义遵循 ROS2 标准: X前, Y左, Z逆时针
 *
 * 键位:
 *   u  i  o     左前 / 前进 / 右前
 *   j  k  l     左移 / 全停 / 右移
 *   m  ,  .     左后 / 后退 / 右后
 *   t     b     逆时针自转 / 顺时针自转
 *
 *   1  2  3  4  速度档位 (0.25 / 0.35 / 0.50 / 0.70 m/s)
 *   h     g     电机启动 / 电机停止
 *   f           里程计复位
 *
 *   q/z X步进 +/- 0.05   w/x Y步进 +/- 0.05
 *   e/c Z步进 +/- 0.10   r  重置步进为默认值
 *
 *   Ctrl+C 退出
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <iomanip>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* 速度档位预设, 按数字键1-4切换 */
struct SpeedGear { double x, y, z; const char *label; };
static const SpeedGear GEARS[] = {
    {0.25, 0.25, 0.5,  "1档-低速"},
    {0.35, 0.35, 0.8,  "2档-中速"},
    {0.50, 0.50, 1.2,  "3档-快速"},
    {0.70, 0.70, 2.0,  "4档-高速"},
};

/* 终端设置: 进raw模式, 退出时恢复 */
static struct termios old_tio;
static void setup_terminal() {
    tcgetattr(STDIN_FILENO, &old_tio);
    struct termios tio = old_tio;
    tio.c_lflag &= ~(ICANON | ECHO);  /* 关闭行缓冲和回显 */
    tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}
static void restore_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

static int kbhit() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

static char read_key() {
    char c = 0;
    if (kbhit()) read(STDIN_FILENO, &c, 1);
    return c;
}

/* ---- ROS2 节点 ---- */
class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        this->declare_parameter("step_x", 0.30);
        this->declare_parameter("step_y", 0.30);
        this->declare_parameter("step_z", 0.50);
        this->declare_parameter("publish_rate", 30.0);

        /* 这三个话题直接对接 chassis_driver 的订阅 */
        pub_cmd_   = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pub_motor_ = this->create_publisher<std_msgs::msg::Bool>("motor_enable", 10);
        pub_reset_ = this->create_publisher<std_msgs::msg::Empty>("reset_odom", 10);

        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&KeyboardControl::tick, this));

        setup_terminal();
        print_help();
    }

    ~KeyboardControl() { restore_terminal(); }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motor_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_reset_;
    rclcpp::TimerBase::SharedPtr timer_;

    double step_x_, step_y_, step_z_;
    double vx_ = 0, vy_ = 0, vth_ = 0;
    int    hold_ct_ = 0;       /* 无按键计数, 超过阈值就停车 */
    char   last_key_ = 0;
    double x_def_, y_def_, z_def_;  /* 默认步进值, 按 r 可重置 */
    bool   motor_on_ = false;
    int    gear_ = 0;          /* 0=手动微调, 1-4=预设档位 */

    /* 第一次tick时从参数加载默认值 */
    void init_defaults() {
        x_def_ = step_x_ = this->get_parameter("step_x").as_double();
        y_def_ = step_y_ = this->get_parameter("step_y").as_double();
        z_def_ = step_z_ = this->get_parameter("step_z").as_double();
    }

    /* 定时器回调: 读键 + 发布速度 */
    void tick() {
        if (step_x_ == 0) init_defaults();

        char c = read_key();
        if (c) { last_key_ = c; process_key(c); }

        /* 松键约270ms后自动停车 (8个周期, 每周期约33ms) */
        if (last_key_) {
            hold_ct_++;
            if (hold_ct_ > 8) {
                vx_ = vy_ = vth_ = 0;
                last_key_ = 0;
                hold_ct_ = 0;
            }
        }

        /* 发布速度指令 */
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x  = vx_;
        msg.linear.y  = vy_;
        msg.angular.z = vth_;
        pub_cmd_->publish(msg);
    }

    void process_key(char c) {
        hold_ct_ = 0;  /* 有按键就重置松键计数 */

        /* 启停 + 复位 */
        if (c == 'h') {
            motor_on_ = true;
            auto m = std_msgs::msg::Bool(); m.data = true;
            pub_motor_->publish(m);
            std::cout << "\n[电机 启动]" << std::endl;
            return;
        }
        if (c == 'g') {
            motor_on_ = false;
            vx_ = vy_ = vth_ = 0;
            auto m = std_msgs::msg::Bool(); m.data = false;
            pub_motor_->publish(m);
            std::cout << "\n[电机 停止]" << std::endl;
            return;
        }
        if (c == 'f') {
            pub_reset_->publish(std_msgs::msg::Empty());
            std::cout << "\n[里程计 复位]" << std::endl;
            return;
        }

        /* 档位切换 */
        if (c >= '1' && c <= '4') {
            int g = c - '0';
            gear_ = g;
            step_x_ = GEARS[g-1].x;
            step_y_ = GEARS[g-1].y;
            step_z_ = GEARS[g-1].z;
            std::cout << "\n[" << GEARS[g-1].label
                      << " x:" << step_x_ << " y:" << step_y_
                      << " z:" << step_z_ << "]" << std::endl;
            return;
        }

        /* 手动微调后退出档位模式 */
        gear_ = 0;

        /* 方向键映射 */
        switch (c) {
            case 'i': vx_ = +step_x_; vy_ = 0;        vth_ = 0;        break;  /* 前进 */
            case ',': vx_ = -step_x_; vy_ = 0;        vth_ = 0;        break;  /* 后退 */
            case 'j': vx_ = 0;        vy_ = +step_y_; vth_ = 0;        break;  /* 左移 */
            case 'l': vx_ = 0;        vy_ = -step_y_; vth_ = 0;        break;  /* 右移 */
            case 'u': vx_ = +step_x_; vy_ = +step_y_; vth_ = 0;        break;  /* 左前 */
            case 'o': vx_ = +step_x_; vy_ = -step_y_; vth_ = 0;        break;  /* 右前 */
            case 'm': vx_ = -step_x_; vy_ = +step_y_; vth_ = 0;        break;  /* 左后 */
            case '.': vx_ = -step_x_; vy_ = -step_y_; vth_ = 0;        break;  /* 右后 */
            case 't': vx_ = 0;        vy_ = 0;        vth_ = +step_z_; break;  /* 逆时针 */
            case 'b': vx_ = 0;        vy_ = 0;        vth_ = -step_z_; break;  /* 顺时针 */
            case 'k': vx_ = 0;        vy_ = 0;        vth_ = 0;        break;  /* 全停 */

            /* 步进微调 */
            case 'q': step_x_ += 0.05; break;
            case 'z': step_x_ -= 0.05; break;
            case 'w': step_y_ += 0.05; break;
            case 'x': step_y_ -= 0.05; break;
            case 'e': step_z_ += 0.10; break;
            case 'c': step_z_ -= 0.10; break;
            case 'r': step_x_ = x_def_; step_y_ = y_def_; step_z_ = z_def_; break;
        }

        /* 步进值上下限 */
        if (step_x_ < 0.05) step_x_ = 0.05;
        if (step_x_ > 2.0)  step_x_ = 2.0;
        if (step_y_ < 0.05) step_y_ = 0.05;
        if (step_y_ > 2.0)  step_y_ = 2.0;
        if (step_z_ < 0.10) step_z_ = 0.10;
        if (step_z_ > 5.0)  step_z_ = 5.0;

        print_status();
    }

    void print_help() {
        std::cout << "\n"
            "  +========================================+\n"
            "  |       底盘键盘遥控 - 操作说明          |\n"
            "  +========================================+\n"
            "  |  方向控制                             |\n"
            "  |  u    i    o   左前 / 前进 / 右前     |\n"
            "  |  j    k    l   左移 / 全停 / 右移     |\n"
            "  |  m    ,    .   左后 / 后退 / 右后     |\n"
            "  |  t         b   逆时针 / 顺时针自转    |\n"
            "  +----------------------------------------+\n"
            "  |  系统控制                             |\n"
            "  |  h   启动电机    g   停止电机         |\n"
            "  |  f   里程计复位                       |\n"
            "  +----------------------------------------+\n"
            "  |  速度档位 (预设)                      |\n"
            "  |  1   低速 0.25m/s   2   中速 0.35m/s  |\n"
            "  |  3   快速 0.50m/s   4   高速 0.70m/s  |\n"
            "  +----------------------------------------+\n"
            "  |  微调                                 |\n"
            "  |  q/z X+/-  w/x Y+/-  e/c Z+/-  r 重置|\n"
            "  |            Ctrl+C 退出                |\n"
            "  +----------------------------------------+\n"
            << std::endl;
    }

    void print_status() {
        const char *gear_str  = gear_ ? GEARS[gear_-1].label : "手动";
        const char *motor_str = motor_on_ ? "ON" : "OFF";
        std::cout << "\r[" << gear_str << " 电机:" << motor_str
                  << "] x:" << std::fixed << std::setprecision(2) << step_x_
                  << " y:" << step_y_ << " z:" << step_z_
                  << " | vx:" << vx_ << " vy:" << vy_ << " vth:" << vth_
                  << "    " << std::flush;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    rclcpp::spin(node);
    restore_terminal();
    rclcpp::shutdown();
    return 0;
}