import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    # Declare the argument 'use_sim_time'
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Flag to determine if use sim time"
    )

    # Get the launch configuration for 'use_sim_time'
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get package name
    prefix = get_package_share_directory("bringup")
    gazebo_prefix = get_package_share_directory("gazebo_ros")
    teleop_prefix = get_package_share_directory('werdna_teleop')

    # Path for robot state publisher
    rsp_file = os.path.join(prefix, "launch", "rsp.launch.py")

    # Path for teleop
    teleop_file = os.path.join(teleop_prefix, 'launch', 'werdna_teleop.launch.py')

    # Path for gazebo_ros_pkgs 
    gazebo_file = os.path.join(gazebo_prefix, "launch", "gzserver.launch.py")
    gz_file = os.path.join(gazebo_prefix, 'launch', 'gzclient.launch.py')

    # Include the robot description launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_file),
        launch_arguments={'gui': 'false'}.items()
    )

    # Include launch description for werdna-teleop
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_file)
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_file),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_file)
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'Model_URDF'
        ],
        output='screen'
    )   


    # Launch the controller manager for diff_drive robot from ros2_control
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont']
    )

    position_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['position_cont']
    )

    # Launch the controller manager for managing joints from ros2_control
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broad']
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rsp,
        gazebo,
        gazebo_client,
        spawn_entity,
        diff_drive_spawner,
        position_cont_spawner,
        joint_broad_spawner,
        teleop,
    ])