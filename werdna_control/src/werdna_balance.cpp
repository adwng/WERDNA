#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

float KP_pitch = -50.5464;
float KD_pitch = 0;
float KI_pitch = 0;

class BalanceControl: public rclcpp::Node
{
public:
    BalanceControl()
    : Node("balance_control_node")
    {
        // Subscribers
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&BalanceControl::imu_callback, this, std::placeholders::_1));

        // Publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&BalanceControl::pid_compute, this));

    }
    
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract pitch and pitch rate from IMU data
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        pitch_error = pitch_ref_  - pitch;
        pitch_rate = msg->angular_velocity.y;
        pitch_error_sum += pitch_error;
    }

    void pid_compute()
    {

        RCLCPP_INFO(this->get_logger(), "pitch: %lf - pitch_rate: %lf - pitch_error_sum: %lf", pitch_error, pitch_rate, pitch_error_sum);
        
        double control_pitch = (KP_pitch * pitch_error) + (KD_pitch * pitch_rate) + (KI_pitch * pitch_error_sum);

        control_pitch = std::min(std::max(control_pitch, -10.0), 10.0);

        RCLCPP_INFO(this->get_logger(), "control_linear: %lf ", control_pitch);

        auto cmd_msg = geometry_msgs::msg::Twist();

        cmd_msg.linear.x = control_pitch;
        velocity_publisher_->publish(cmd_msg);
    }

    float pitch_ref_ = 0.0;
    float pitch_error;
    float pitch_rate;
    float pitch_error_sum;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceControl>());
    rclcpp::shutdown();
    return 0;
}