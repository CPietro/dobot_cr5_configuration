import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gazebo = FindPackageShare('cr5_gazebo')
    pkg_moveit = FindPackageShare('dobot_moveit')
    rviz_config_file = os.path.join(pkg_gazebo.find('cr5_gazebo'), 'rviz', 'zed_perception.rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo, '/launch/sim_gazebo.launch.py'])
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_moveit, '/launch/dobot_moveit.launch.py']),
        launch_arguments={'use_rviz': 'false', 'use_sim_time': 'true'}.items()
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_gazebo',
        output='log', arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    homing_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/cr5_group_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '"{joint_names: [\'joint1\', \'joint2\', \'joint3\', \'joint4\', \'joint5\', \'joint6\'], points: [{positions: [-1.8326, 0.0, 0.0, 0.7854, -1.5708, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}"'
        ],
        shell=True
    )

    oracle_node = Node(
        package='cr5_gazebo',
        executable='oracle_frustum_culling_node',
        name='oracle_frustum_culling',
        output='screen'
    )

    bbox_image_overlay_node = Node(
        package='cr5_gazebo',
        executable='bbox_image_overlay_node',
        name='bbox_image_overlay_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,

        TimerAction(
            period=8.0,
            actions=[moveit_launch, rviz_node]
        ),

        TimerAction(
            period=12.0,
            actions=[homing_cmd]
        ),

        # After 4 seconds from homing cmd
        TimerAction(
            period=16.0,
            actions=[oracle_node, bbox_image_overlay_node]
        )
    ])
