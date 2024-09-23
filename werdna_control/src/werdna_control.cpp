#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <chrono>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "werdna_msgs/msg/joy_ctrl_cmds.hpp"

class MotionPlanner : public rclcpp::Node {
public:
    MotionPlanner() : 
        Node("motion_planner_node"),
        joint_names{},
        joint_positions{},
        state(0),
        height(0),
        previous_height(0),
        displacement(0),
        linear_x(0),
        angular_z(0),
        current_positions{0.0, 0.0, 0.0, 0.0},
        target_positions{0.0, 0.0, 0.0, 0.0},
        smooth_speed(0.1)
    {
        teleop_subscriber = this->create_subscription<werdna_msgs::msg::JoyCtrlCmds>(
            "werdna_control", 10,
            std::bind(&MotionPlanner::teleop_callback, this, std::placeholders::_1));
        
        joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_broad/joint_states", 10,
            std::bind(&MotionPlanner::joints_callback, this, std::placeholders::_1));
        
        leg_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_cont/joint_trajectory", 130);
        
        diff_drive_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            "diff_cont/cmd_vel_unstamped", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&MotionPlanner::timer_callback, this)
        );
    }

private:
    void teleop_callback(const werdna_msgs::msg::JoyCtrlCmds::SharedPtr msg) {
        state = msg->state;
        height = msg->height;
        displacement = msg->displacement;
        linear_x = static_cast<float>(msg->linear.x);
        angular_z = static_cast<float>(msg->angular.z);
    }

    void joints_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_names = msg->name;
        joint_positions.clear();
        for (size_t i = 0; i < joint_names.size(); ++i) {
            joint_positions[joint_names[i]] = msg->position[i];
        }
        current_positions = {
            joint_positions.count("left_hip_joint") ? joint_positions["left_hip_joint"] : 0.0,
            joint_positions.count("left_knee_joint") ? joint_positions["left_knee_joint"] : 0.0,
            joint_positions.count("right_hip_joint") ? joint_positions["right_hip_joint"] : 0.0,
            joint_positions.count("right_knee_joint") ? joint_positions["right_knee_joint"] : 0.0
        };
    }

    std::pair<double, double> inverse_kinematics(double f, double x) {
        double L1 = 0.1;
        double L2 = 0.1;

        if (f != 0) {
            double knee_theta = std::acos((L1 * L1 + L2 * L2 - f * f) / (2 * L1 * L2)) + (M_PI / 3);
            double hip_theta = std::asin(x / f) - std::acos((L1 * L1 + f * f - L2 * L2) / (2 * L1 * f));
            return {knee_theta, hip_theta};
        } else {
            return {0, 0};
        }
    }

    void pub_positions()
    {

        points.clear();

        // First point (current position)
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // Start ASAP
        point.positions.resize(current_positions.size());
        for (size_t i = 0; i < current_positions.size(); ++i) {
            point.positions[i] = current_positions[i];  // Set current positions
        }
        points.push_back(point);

        // Second point (target position)
        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.time_from_start = rclcpp::Duration::from_seconds(1.0);
        point2.positions.resize(target_positions.size());
        for (size_t i = 0; i < target_positions.size(); ++i) {
            point2.positions[i] = target_positions[i];  // Set target positions
        }
        points.push_back(point2);

         // Create and populate JointTrajectory message
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.joint_names = {"left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};  // Set joint names
        trajectory_msg.points = points;              // Add points

        // Publish the trajectory message
        leg_publisher->publish(trajectory_msg);
    }

    void timer_callback() {
        if (height != previous_height)
        {
            auto [knee_theta, hip_theta] = inverse_kinematics(height, displacement);
            target_positions = {static_cast<float>(hip_theta), static_cast<float>(knee_theta), static_cast<float>(hip_theta), static_cast<float>(knee_theta)};
            previous_height = height;
            pub_positions();
        }

        auto drive_msg = geometry_msgs::msg::Twist();
        drive_msg.linear.x = linear_x;
        drive_msg.angular.z = angular_z;
        diff_drive_publisher->publish(drive_msg);

        // Print the information from subscribers
        RCLCPP_INFO(this->get_logger(),
                     "State: %d, Height: %.2f, Displacement: %.2f, Linear X: %.2f, Angular Z: %.2f",
                     state, height, displacement, linear_x, angular_z);

        RCLCPP_INFO(this->get_logger(),
                     "Current Positions: [Hip L: %.2f, Knee L: %.2f, Hip R: %.2f, Knee R: %.2f]",
                     current_positions[0], current_positions[1], current_positions[2], current_positions[3]);
    }

    std::vector<std::string> joint_names;
    std::unordered_map<std::string, double> joint_positions;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

    int state;
    float height;
    float previous_height;
    float displacement;
    double linear_x;
    double angular_z;

    std::vector<double> current_positions;
    std::vector<double> target_positions;
    double smooth_speed;

    rclcpp::Subscription<werdna_msgs::msg::JoyCtrlCmds>::SharedPtr teleop_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr leg_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_drive_publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPlanner>());
    rclcpp::shutdown();
    return 0;
}
