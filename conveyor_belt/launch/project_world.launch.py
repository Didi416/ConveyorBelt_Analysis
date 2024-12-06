import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():
    # get directory paths to necessary packages:
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_project = get_package_share_directory('conveyor_belt')
    
    # identify paths to requried world
    world = os.path.join(pkg_project, 'worlds', 'project_world_2.world')
    rviz_config_dir = os.path.join(pkg_project, 'rviz', 'rviz2.rviz')
    
    # set use_sim_time as well as global coordinates for the turtlebot upon launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='true'),
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    urdf_path = os.path.join(
        get_package_share_directory('conveyor_belt'), 'urdf', 'camera.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    model_path = os.path.join(
        get_package_share_directory('conveyor_belt'), 'models','realsense_camera', 'model.sdf')

    # Start Launch Description
    gazebo_LD = LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='Specify namespace of the robot'),

        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Specify namespace of the robot'),
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

        # Start conveyor belt service
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/conveyor_belt_1/CONVEYORPOWER ",
                "conveyorbelt_msgs/srv/ConveyorBeltControl ",
                '"{power: 5}"',
            ]],
            shell=True
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', "camera_robot",
                '-file', model_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01'
            ],
            output='screen',
        ),

        # Launch rviz, spawn, despawn and load profile nodes
        Node( 
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # Node(
        #     package='conveyor_belt',
        #     executable='spawnObjects',
        #     name='spawnObjects',
        #     output='screen'),
        
        Node(
            package='conveyor_belt',
            executable='despawnObjects',
            name='despawnObjects',
            output='screen'),

        # Node(
        #     package='conveyor_belt',
        #     executable='load_profile',
        #     name='load_profile',
        #     output='screen'),
    ])
    return gazebo_LD