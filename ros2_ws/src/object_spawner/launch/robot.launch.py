from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import re
import tempfile
from time import sleep
from launch.actions import TimerAction


# The URDF and world files refer to other files through package placeholders
# instead of absolute paths, so the workspace can live anywhere:
#   $(find <pkg>)    -> the package's share directory (meshes, config, ...)
#   $(prefix <pkg>)  -> the package's install prefix (for plugin libraries)
_PACKAGE_TOKEN = re.compile(r'\$\((find|prefix) ([A-Za-z0-9_]+)\)')


def resolve_package_tokens(text):
    """Replace $(find pkg) and $(prefix pkg) placeholders with real paths."""
    def lookup(match):
        kind, package = match.groups()
        if kind == 'find':
            return get_package_share_directory(package)
        return get_package_prefix(package)
    return _PACKAGE_TOKEN.sub(lookup, text)


def resolved_copy(path, out_dir):
    """Write a copy of `path` with placeholders resolved and return its path.

    Gazebo and spawn_entity.py read model files from disk, so the resolved
    text is written to a temporary directory and that copy is used instead.
    """
    with open(path, 'r') as infp:
        text = resolve_package_tokens(infp.read())
    out_path = os.path.join(out_dir, os.path.basename(path))
    with open(out_path, 'w') as outfp:
        outfp.write(text)
    return out_path


def generate_launch_description():
    pkg_share = get_package_share_directory('object_spawner')
    resolved_dir = tempfile.mkdtemp(prefix='object_spawner_')
    urdf_path = resolved_copy(os.path.join(pkg_share, 'urdf', 'robot.urdf'), resolved_dir)
    world_path = resolved_copy(os.path.join(pkg_share, 'worlds', 'empty_world.sdf'), resolved_dir)
    ramp_pol_urdf = resolved_copy(os.path.join(pkg_share, 'urdf', 'ramp_polished.urdf'), resolved_dir)
    ramp_conc_urdf = resolved_copy(os.path.join(pkg_share, 'urdf', 'ramp_concrete.urdf'), resolved_dir)
    ramp_rubber_urdf = resolved_copy(os.path.join(pkg_share, 'urdf', 'ramp_rubber.urdf'), resolved_dir)
    cube_urdf = os.path.join(pkg_share,'urdf','cube.urdf')
    controller_config = PathJoinSubstitution([
        FindPackageShare("object_spawner"),
        "config",
        "robot_controllers.yaml"
    ])

    # Load URDF content
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # Pass robot_description only to robot_state_publisher
    robot_description_param = {
        'robot_description': robot_description_content,
        'use_sim_time': True
    }

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-file', urdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    spawn_ramp_pol = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ramp',
        output='screen',
        arguments=[
            '-entity', 'ramp_pol',         
            '-file', ramp_pol_urdf,
            '-y', '5.0', '-z', '0.856',               
        ]
    )

    spawn_ramp_conc = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ramp',
        output='screen',
        arguments=[
            '-entity', 'ramp_conc',         
            '-file', ramp_conc_urdf,
            '-y', '5.0', '-z', '0.856', '-Y', '-1.57'             
        ]
    )

    spawn_ramp_rubber = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ramp',
        output='screen',
        arguments=[
            '-entity', 'ramp_rubber',         
            '-file', ramp_rubber_urdf,
            '-y', '5.0', '-z', '0.856', '-Y', '1.57'             
        ]
    )

    
    spawn_cube1 = TimerAction(period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_cube',
                output='screen',
                arguments=[
                    '-entity', 'cube1',         
                    '-file', cube_urdf,
                    '-x', '-0.05', '-y', '4.1', '-z', '0.9'              
                ]
            )
        ]
    )

    spawn_cube2 = TimerAction(period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_cube',
                output='screen',
                arguments=[
                    '-entity', 'cube2',         
                    '-file', cube_urdf,
                    '-x', '-0.95', '-y', '4.95', '-z', '0.9',           
                ]
            )
        ]
    )

    spawn_cube3 = TimerAction(period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_cube',
                output='screen',
                arguments=[
                    '-entity', 'cube3',         
                    '-file', cube_urdf,
                    '-x', '0.8', '-y', '4.97', '-z', '0.9',      
                ]
            )
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )


    # Start ros2_control_node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    controller_names = [
    'joint_state_broadcaster',
    'shoulder_joint_position_controller',
    'elbow_joint_position_controller',
    'left_gripper_position_controller',
    'right_gripper_position_controller',
    'diff_drive_controller',
    ]
    # Event handler to spawn controllers only when the controller manager is ready
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[controller],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ) for controller in controller_names
            ]
        )
    )


    # Launch description: make sure all nodes are included
    return LaunchDescription([
        gazebo,
        spawn_entity,
        spawn_ramp_pol,
        spawn_ramp_conc,
        spawn_ramp_rubber,
        spawn_cube1,
        spawn_cube2,
        spawn_cube3,
        robot_state_publisher,
        controller_manager,
        delayed_controllers,
    ])
