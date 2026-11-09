from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('cr5_gazebo')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='first.world',
        description='Gazebo world file name located in cr5_gazebo/worlds (e.g. first.world, first_ac.world)'
    )

    full_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo, '/launch/sim_zedgazebo_full.launch.py']),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    oracle_node = Node(
        package='cr5_gazebo',
        executable='oracle_frustum_culling_node',
        name='oracle_frustum_culling',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bbox_image_overlay_node = Node(
        package='cr5_gazebo',
        executable='bbox_image_overlay_node',
        name='bbox_image_overlay_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        full_sim_launch,

        # Start perception and 2D image overlay visualization after robot homing
        TimerAction(
            period=16.0,
            actions=[oracle_node, bbox_image_overlay_node]
        )
    ])
