"""Launch Gazebo with cr5_gazebo world and spawn the Dobot CR5 robot.

Note:
    This launch file starts Gazebo with the model path configured and executes
    spawn_entity.py to spawn 'dobot_cr5'. The spawn entity node waits for
    the '/robot_description' topic, which is published by robot_state_publisher
    in dobot_moveit.launch.py.
    This launch file is intended to be included by sim_zedgazebo_full.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _is_invalid_model_path(path_str):
    if not path_str:
        return True
    norm = os.path.normpath(path_str)
    # turtlebot3 hook appends /opt/ros/humble/share/turtlebot3_manipulation_gazebo/.. which resolves to /opt/ros/humble/share
    if 'turtlebot3' in norm:
        return True
    if norm == '/opt/ros/humble/share' or norm.endswith('/opt/ros/humble/share'):
        return True
    if norm.startswith('/opt/ros/') and norm.endswith('/share'):
        return True
    return False


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_cr5_gazebo = get_package_share_directory('cr5_gazebo')
    
    pkg_dobot_rviz = get_package_share_directory('dobot_rviz')
    dobot_rviz_model_path = os.path.dirname(pkg_dobot_rviz)

    models_path = os.path.join(pkg_cr5_gazebo, 'models')

    # Setup clean models directory for dobot_rviz so it can be resolved as model://dobot_rviz
    # without exposing share/ parent (which contains ament_index and colcon-core without model.config)
    clean_models_dir = os.path.join(pkg_cr5_gazebo, 'gazebo_models')
    try:
        os.makedirs(clean_models_dir, exist_ok=True)
        dobot_symlink = os.path.join(clean_models_dir, 'dobot_rviz')
        if not os.path.exists(dobot_symlink) and not os.path.islink(dobot_symlink):
            os.symlink(pkg_dobot_rviz, dobot_symlink)
    except OSError:
        clean_models_dir = None

    # Filter out invalid /opt/ros/humble/share paths added by hooks (e.g. turtlebot3)
    raw_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    clean_model_paths = [
        p for p in raw_model_path.split(os.pathsep)
        if p and not _is_invalid_model_path(p)
    ]

    # Ensure standard Gazebo model paths if they exist
    for default_model in ['/usr/share/gazebo-11/models', '/usr/share/gazebo/models', os.path.expanduser('~/.gazebo/models')]:
        if os.path.isdir(default_model) and default_model not in clean_model_paths:
            clean_model_paths.append(default_model)

    # Prepend cr5_gazebo models and clean dobot_rviz model directory
    model_paths_to_add = [models_path]
    if clean_models_dir:
        model_paths_to_add.append(clean_models_dir)
    else:
        # Fallback to dobot_rviz_model_path if symlink creation was not possible
        if dobot_rviz_model_path not in clean_model_paths:
            clean_model_paths.append(dobot_rviz_model_path)

    for p in reversed(model_paths_to_add):
        if p in clean_model_paths:
            clean_model_paths.remove(p)
        clean_model_paths.insert(0, p)

    gazebo_model_paths = os.pathsep.join(clean_model_paths)

    # Setup GAZEBO_RESOURCE_PATH to ensure package://dobot_rviz/meshes/... resolves
    raw_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    clean_resource_paths = [
        p for p in raw_resource_path.split(os.pathsep)
        if p and not _is_invalid_model_path(p)
    ]
    for default_res in ['/usr/share/gazebo-11', '/usr/share/gazebo']:
        if os.path.isdir(default_res) and default_res not in clean_resource_paths:
            clean_resource_paths.append(default_res)
    for res_p in [dobot_rviz_model_path, models_path]:
        if res_p not in clean_resource_paths:
            clean_resource_paths.append(res_p)
    gazebo_resource_paths = os.pathsep.join(clean_resource_paths)

    # Monkey-patch GazeboRosPaths.get_paths to intercept gzserver and gzclient launch scripts
    # which otherwise prepend turtlebot3_manipulation_gazebo/.. to additional_env['GAZEBO_MODEL_PATH']
    try:
        from scripts import GazeboRosPaths
        _orig_get_paths = GazeboRosPaths.get_paths

        def _patched_get_paths():
            _, plugin, _ = _orig_get_paths()
            return gazebo_model_paths, plugin, gazebo_resource_paths

        GazeboRosPaths.get_paths = _patched_get_paths
    except Exception as e:
        print(f'[sim_gazebo.launch.py] Warning: could not patch GazeboRosPaths: {e}')

    # Prevent gzserver.launch.py and gzclient.launch.py from doing:
    # model += pathsep + environ['GAZEBO_MODEL_PATH'] (which causes duplicates or leading colon)
    os.environ.pop('GAZEBO_MODEL_PATH', None)
    os.environ.pop('GAZEBO_RESOURCE_PATH', None)

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
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_paths),
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=gazebo_resource_paths),
        gazebo_launch,
        spawn_robot
    ])
