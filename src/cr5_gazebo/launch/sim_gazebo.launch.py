import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_cr5_gazebo = get_package_share_directory('cr5_gazebo')
    
    pkg_dobot_rviz = get_package_share_directory('dobot_rviz')
    dobot_rviz_model_path = os.path.dirname(pkg_dobot_rviz)

    models_path = os.path.join(pkg_cr5_gazebo, 'models')
    gazebo_model_paths = os.pathsep.join([models_path, dobot_rviz_model_path])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='first.world',
        description='Gazebo world file name located in cr5_gazebo/worlds (e.g. first.world, first_ac.world)'
    )

    world_file_path = PathJoinSubstitution([
        pkg_cr5_gazebo, 'worlds', LaunchConfiguration('world')
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'extra_gazebo_args': '--verbose'
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_dobot',
        arguments=[
            '-entity', 'dobot_cr5',
            '-topic', 'robot_description',
            '-x', '0.2',
            '-y', '-0.6',
            '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_paths),
        gazebo_launch,
        spawn_robot
    ])
