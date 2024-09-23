import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    urdf_prefix = get_package_share_directory('werdna_description')
    gazebo_prefix = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(urdf_prefix, 'launch', 'display.launch.py')
    gazebo_file = os.path.join(gazebo_prefix, 'launch', 'gzserver.launch.py')
    gz_file = os.path.join(gazebo_prefix, 'launch', 'gzclient.launch.py')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_file)
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_file)
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
            '-entity', 'werdna',
            '-z', '0.09'
        ],
        output='screen'
    )   

    return LaunchDescription(
        [rsp, gazebo, gazebo_client, spawn_entity]
    )