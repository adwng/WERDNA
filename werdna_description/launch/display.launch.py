import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import xacro

def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    show_gui = LaunchConfiguration('gui')

    # Process the URDF filewerdna_description
    package_path = get_package_share_directory('werdna_description')

    xacro_file = os.path.join(package_path, 'urdf', 'werdna_core.xacro')
    robot_description_config=xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    # Create the joint_state_publisher node
    node_joint_state_publisher = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Optionally include the joint_state_publisher_gui (for GUI control of the joints)
    node_joint_state_publisher_gui = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        gui_arg,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_joint_state_publisher_gui,
    ])
