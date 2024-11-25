import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # get directory paths to necessary packages:
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_project = get_package_share_directory('conveyor_belt')
    
    # identify paths to requried world
    world = os.path.join(pkg_project, 'worlds', 'project_world_2.world')
    # rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    
    # set use_sim_time as well as global coordinates for the turtlebot upon launch
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true'),

    # Start Launch Description
    gazebo_LD = LaunchDescription([
        # Gazebo Launch directives
        # Include in launch description the gazebo server and client files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        # Launch RVIZ Node with rviz configuration file as argument (not the default file, but .rviz we have in this directory)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
    return gazebo_LD