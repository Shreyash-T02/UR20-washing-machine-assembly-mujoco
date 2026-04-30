import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ur20_share    = get_package_share_directory('wm_ur20_description')
    bringup_share = get_package_share_directory('wm_bringup')
    cell_share    = get_package_share_directory('wm_cell_description')

    # ── Generate scene MJCF from xacro (synchronous — runs before any node starts) ──
    scene_xacro    = os.path.join(cell_share, 'mjcf', 'scene.xacro')
    mesh_dir       = os.path.join(ur20_share, 'meshes')
    generated_mjcf = '/tmp/wm_assembly_scene.xml'

    subprocess.run(
        ['xacro', scene_xacro, f'meshdir:={mesh_dir}', '-o', generated_mjcf],
        check=True,
    )
    # Strip the xacro namespace declaration that MuJoCo's parser doesn't expect.
    with open(generated_mjcf) as fh:
        txt = fh.read()
    txt = txt.replace(' xmlns:xacro="http://www.ros.org/wiki/xacro"', '')
    with open(generated_mjcf, 'w') as fh:
        fh.write(txt)

    xacro_file = os.path.join(ur20_share, 'urdf', 'ur20_mujoco.urdf.xacro')
    ctrl_yaml  = os.path.join(ur20_share, 'config', 'ur20_controllers.yaml')
    rviz_cfg   = os.path.join(bringup_share, 'config', 'ur20.rviz')

    robot_description = {
        'robot_description': Command([FindExecutable(name='xacro'), ' ', xacro_file])
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        parameters=[
            robot_description,
            ctrl_yaml,
            {'robot_model_path':     generated_mjcf},
            {'simulation_frequency': 500.0},
            {'realtime_factor':      1.0},
            {'show_gui': LaunchConfiguration('gui')},
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen',
    )

    # 5-second delay lets the controller_manager finish initialising before spawners run.
    spawn_controllers = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Spawning controllers...'),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['ur20_joint_trajectory_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['conveyor_velocity_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_position_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            ),
        ],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='log',
    )

    # Kinematics nodes start after controllers are up (7 s gives controllers time to activate).
    kinematics_nodes = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='Starting kinematics nodes...'),
            Node(package='wm_kinematics', executable='fk_node',       output='screen'),
            Node(package='wm_kinematics', executable='jacobian_node',  output='screen'),
            Node(package='wm_kinematics', executable='ik_node',        output='screen'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui',  default_value='true',  description='Open MuJoCo viewer'),
        DeclareLaunchArgument('rviz', default_value='true',  description='Open RViz2'),
        robot_state_publisher,
        mujoco_node,
        spawn_controllers,
        kinematics_nodes,
        rviz_node,
    ])
