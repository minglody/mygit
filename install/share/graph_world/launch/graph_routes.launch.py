from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('graph_world')
    world = os.path.join(pkg_share, 'worlds', 'graph_routes.world')

    # Start pose args
    start_x = LaunchConfiguration('x')
    start_y = LaunchConfiguration('y')
    start_yaw = LaunchConfiguration('yaw')

    # Gazebo classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # TurtleBot3 URDF (waffle_pi)
    tb3_desc_share = get_package_share_directory('turtlebot3_description')
    urdf = os.path.join(tb3_desc_share, 'urdf', 'turtlebot3_waffle_pi.urdf')

    # Robot State Publisher (TF용)
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen'
    )

    # Spawn into Gazebo from URDF file
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle_pi',
            '-file', urdf,
            '-x', start_x, '-y', start_y, '-z', '0.01',
            '-Y', start_yaw
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-8.0'),  # A 근처
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo, rsp, spawn
    ])
