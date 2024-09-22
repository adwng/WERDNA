import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from werdna_msgs.msg import JoyCtrlCmds  # Import the custom message type
import numpy as np

class motion_planner(Node):
    def __init__(self):
        super().__init__('Motion_Plannner_Node')

        # Subscribers
        self.teleop_subscriber = self.create_subscription(JoyCtrlCmds, 'werdna_control', self.teleop_callback, 10)
        self.joint_states_subscriber = self.create_subscription(JointState, 'joint_states', self.joints_callback, 10)

        # Publishers
        self.leg_publisher = self.create_publisher(Float64MultiArray, 'position_cont/commands', 10)
        self.diff_drive_publisher = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)

        # Variables
        self.joint_names = []
        self.joint_positions = {}

        self.state = None
        self.height = 0
        self.previous_height = 0  # Track the previous height
        self.displacement = 0
        self.linear_x = 0
        self.angular_z = 0

        self.current_positions = [0.0, 0.0, 0.0, 0.0]  # Store current joint positions
        self.target_positions = [0.0, 0.0, 0.0, 0.0]  # Store target joint positions

        self.smooth_speed = 0.1  # Speed of transition between positions

        # Timer for callbacks
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def teleop_callback(self, msg):
        self.state = msg.state
        self.height = msg.height
        self.displacement = msg.displacement
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def joints_callback(self, msg):
        self.joint_names = msg.name
        self.joint_positions = dict(zip(self.joint_names, msg.position))
        # Update the current positions with the relevant joints' positions
        self.current_positions = [self.joint_positions.get('left_knee_joint', 0.0),
                                  self.joint_positions.get('left_hip_joint', 0.0),
                                  self.joint_positions.get('right_knee_joint', 0.0),
                                  self.joint_positions.get('right_hip_joint', 0.0)]

    def inverse_kinematics(self, f, x):
        L1 = L2 = 0.1

        if f != 0:
            knee_theta = np.arccos((L1**2 + L2**2 - f**2) / (2*L1*L2)) + (np.pi/3)
            hip_theta = np.arcsin(x/f) - np.arccos( ( L1**2 + f**2 - L2**2 ) / ( 2 * L1 * f ) )
            return knee_theta, hip_theta
        else:
            return 0, 0

    def interpolate_positions(self):
        # Interpolate the current positions towards the target positions
        interpolated_positions = []
        for current, target in zip(self.current_positions, self.target_positions):
            new_position = current + np.sign(target - current) * min(abs(target - current), self.smooth_speed)
            interpolated_positions.append(new_position)

        # Publish the interpolated joint positions
        msg = Float64MultiArray()
        msg.data = interpolated_positions
        self.leg_publisher.publish(msg)

    def timer_callback(self):
        # Log joystick state and movement values
        self.get_logger().info(f'{self.state}, {self.height}, {self.displacement}, {self.linear_x}, {self.angular_z}')

        # Check if the height has changed
        if self.height != self.previous_height:
            # Compute new joint positions if height changed
            knee_theta, hip_theta = self.inverse_kinematics(self.height, self.displacement)

            # Set the target joint positions (left knee, left hip, right knee, right hip)
            self.target_positions = [hip_theta, knee_theta, -hip_theta, -knee_theta]

            # Update the previous height to the current value
            self.previous_height = self.height

        # Smoothly interpolate between the current and target positions
        self.interpolate_positions()

        # Prepare and publish diff drive commands regardless of height change
        drive_msg = Twist()
        drive_msg.linear.x = self.linear_x
        drive_msg.angular.z = self.angular_z
        self.diff_drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = motion_planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
