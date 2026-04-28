from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur20_share = FindPackageShare("wm_ur20_description")
    bringup_share = FindPackageShare("wm_bringup")

    xacro_file = PathJoinSubstitution([ur20_share, "urdf", "ur20_mujoco.urdf.xacro"])
    mjcf_file  = PathJoinSubstitution([ur20_share, "mjcf", "ur20.xml"])
    ctrl_yaml  = PathJoinSubstitution([ur20_share, "config", "ur20_controllers.yaml"])
    rviz_cfg   = PathJoinSubstitution([bringup_share, "config", "ur20.rviz"])

    # xacro CLI avoids Python environment conflicts with conda
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            ctrl_yaml,
            {"robot_model_path": mjcf_file},
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"show_gui": LaunchConfiguration("gui")},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    # 5-second delay: OnProcessStart fires before the controller_manager finishes
    # initialising, causing spawners to find ghost controllers from the previous run.
    spawn_controllers = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Spawning controllers..."),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster",
                           "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["ur20_joint_trajectory_controller",
                           "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui",  default_value="true",  description="Open MuJoCo viewer"),
        DeclareLaunchArgument("rviz", default_value="true",  description="Open RViz2"),
        robot_state_publisher,
        mujoco_node,
        spawn_controllers,
        rviz_node,
    ])
