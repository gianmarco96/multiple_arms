from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Launch arguments
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Y position')
    declare_z = DeclareLaunchArgument('z', default_value='0.1', description='Z position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Paths
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    # your_pkg = get_package_share_directory('your_package')
    # urdf_file = os.path.join(your_pkg, 'urdf', 'my_robot.urdf.xacro')

    
    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    TURTLEBOT3_MODEL = 'waffle'


    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    world = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'worlds', 'multi_robot_world.world')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
                            'publish_frequency': 10.0}]
        ,
          arguments=[urdf],
    )

    # Gazebo launch (empty.world assumed)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn robot using gazebo_ros spawn_entity.py
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_x, declare_y, declare_z, declare_yaw,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot
    ])
