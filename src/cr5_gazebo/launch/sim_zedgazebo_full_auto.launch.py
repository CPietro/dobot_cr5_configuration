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

    reworked_map_node = Node(
        package='cr5_moveit_cpp_demo',
        executable='reworked_map_node',
        name='reworked_map_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    move_cr5_node = Node(
        package='cr5_moveit_cpp_demo',
        executable='move_cr5_node_modular',
        name='move_cr5_node_modular',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        full_sim_launch,

        # Start perception, planning map reprocessing, and trajectory execution after robot homing
        TimerAction(
            period=16.0,
            actions=[oracle_node, reworked_map_node, move_cr5_node]
        )
    ])
