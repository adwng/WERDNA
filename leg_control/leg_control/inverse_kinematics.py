import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class Inverse_Kinematics(Node):
    def __init__(self):
        super().__init__('Inverse_Kinematics_Node')

        # Subscribers: teleop, joint states
        self.teleop_subscriber = self.create_subscription(Twist, 'cmd_vel_keyboard', self.teleop_callback, 10)
        self.joints_subscriber = self.create_subscription(JointState, 'joint_states', self.joints_callback, 10)
        
        # Publishers: joints 
        self.inv_publisher = self.create_publisher(Float64MultiArray, 'position_cont/commands', 10)
        self.drive_publisher = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)

        # Timer for periodic updates
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize storage for joint information
        self.joint_names = []
        self.joint_positions = {}
        self.joint_velocities = {}

        self.max_height = 0.1
        self.max_displacement = 0.1
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.height = 0
        self.displacement = 0

        # Initialize target positions and smoothing parameters
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        self.smooth_speed = 0.1  # The speed at which to approach the target position

        self.prev_height

    def teleop_callback(self, msg):
        self.height = msg.linear.z
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        if self.height > self.max_height:
            self.height = self.max_height

    def joints_callback(self, msg):
    # Assuming 'msg' contains joint names, positions, velocities, and efforts
        self.joint_names = msg.name  # Joint names
        self.joint_positions = dict(zip(self.joint_names, msg.position))  # Convert to dictionary
        self.joint_velocities = dict(zip(self.joint_names, msg.velocity))  # Convert to dictionary


    def inverse_kinematics(self, f, x):
        L1 = L2 = 0.1

        if f != 0:
            knee_theta = np.arccos((L1**2 + L2**2 - f**2) / (2*L1*L2)) + (np.pi/3)
            hip_theta = np.arcsin(x/f) - np.arccos( ( L1**2 + f**2 - L2**2 ) / ( 2 * L1 * f ) )
            return knee_theta, hip_theta
        else:
            return 0, 0
        
    def compute_points(self, height, displacement):
        if height != self.prev_height:
            # Compute target positions
            knee_theta, hip_theta = self.inverse_kinematics(height, displacement)
            new_target_positions = [hip_theta, knee_theta, -hip_theta, -knee_theta]

            # Smoothly interpolate towards the new target positions
            current_positions = [self.joint_positions.get(name, 0.0) for name in ['left_hip_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']]
            smoothed_positions = [current + self.smooth_speed * (target - current) for current, target in zip(current_positions, new_target_positions)]

            self.prev_height = height
            return smoothed_positions
        else:
            return self.prev_height

        

    def timer_callback(self):
        
        smoothed_positions = self.compute_points(self.height, self.displacement)
        # Create and publish the message
        msg = Float64MultiArray()
        msg.data = smoothed_positions

        drive_msg = Twist()
        drive_msg.linear.x = self.linear_x
        drive_msg.angular.z = self.angular_z

        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.inv_publisher.publish(msg)
        self.drive_publisher.publish(drive_msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = Inverse_Kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
