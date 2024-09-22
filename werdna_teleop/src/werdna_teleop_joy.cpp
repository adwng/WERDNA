#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <chrono>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "werdna_msgs/msg/joy_ctrl_cmds.hpp"

using std::placeholders::_1;
using namespace std::chrono;

// Declare some variables for button toggle delay and time tracking
auto cmd = werdna_msgs::msg::JoyCtrlCmds();
steady_clock::time_point t1 = steady_clock::now();
int btn_tgl_delay = 3000;  // 3000 milliseconds

class PublishingSubscriber : public rclcpp::Node
{
public:
    PublishingSubscriber()
        : Node("werdna_teleop_gamepad_node")
    {
        // Subscribe to the joystick topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 12, std::bind(&PublishingSubscriber::topic_callback, this, _1));

        // Publisher to publish JoyCtrlCmds messages
        publisher_ = this->create_publisher<werdna_msgs::msg::JoyCtrlCmds>("werdna_control", 40);
    }

    // Function to map joystick state to JoyCtrlCmds
    void joy_state_to_joy_cmd(sensor_msgs::msg::Joy::SharedPtr msg_joy)
    {
        auto now = steady_clock::now();

        // Toggle state with button 3 (start balancing)
        if (!cmd.state && msg_joy->buttons[3] && duration_cast<milliseconds>(now - t1).count() > btn_tgl_delay)
        {
            cmd.state = true;
            t1 = steady_clock::now();
        }
        else if (cmd.state && msg_joy->buttons[3] && duration_cast<milliseconds>(now - t1).count() > btn_tgl_delay)
        {
            cmd.state = false;
            t1 = steady_clock::now();
        }

        // If in the active state, map joystick input to robot control commands
        if (cmd.state)
        {

            cmd.height = (std::abs(1 - msg_joy->axes[4]))/2 * 0.1;

            // Map axes displacement (normalized between -0.1 and 0.1)
            cmd.displacement = msg_joy->axes[3] * 0.1;

            // Map axes to linear and angular motion
            cmd.linear.x = msg_joy->axes[1];  // Forward/backward
            cmd.angular.z = msg_joy->axes[0];  // Left/right

            if (msg_joy->buttons[6])
            {
              if (cmd.linear.x > 0.1)
              {
                cmd.linear.x +=2.0;
              }
              else if (cmd.linear.x < -0.1)
              {
                cmd.linear.x -= 2.0;
              }
              if (cmd.angular.z > 0.1)
              {
                cmd.angular.z+=2.0;
              }
              else if (cmd.angular.z < -0.1)
              {
                cmd.angular.z-=2.0;
              }
            }
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg_rx)
    {
        // Process joystick input and map it to robot control commands
        joy_state_to_joy_cmd(msg_rx);

        // Publish the control command
        publisher_->publish(cmd);
    }

    // ROS 2 subscription and publisher
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<werdna_msgs::msg::JoyCtrlCmds>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishingSubscriber>());
    rclcpp::shutdown();
    return 0;
}
