import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    bringup_prefix = get_package_share_directory('bringup')
    description_prefix = get_package_share_directory('werdna_description')
    teleop_prefix = get_package_share_directory('werdna_teleop')
    control_prefix = get_package_share_directory('werdna_control')

    gazebo_launch_file = os.path.join(description_prefix, 'launch', 'gazebo.launch.py')

    teleop_launch_file = os.path.join(teleop_prefix, 'launch', 'werdna_teleop.launch.py')
    control_launch_file = os.path.join(control_prefix, 'launch', 'werdna_control.launch.py')

    rviz_config_file = os.path.join(bringup_prefix, 'configs', 'config.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file)
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )

    joint_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_cont']
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    return LaunchDescription(
        [gazebo, teleop, control, rviz_node, diff_drive_spawner, joint_cont_spawner, joint_broad_spawner]
    )
