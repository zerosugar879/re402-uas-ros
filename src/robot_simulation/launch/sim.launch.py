import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc_name = "robot_description"
    pkg_bringup_name = "robot_simulation"
    robot_name = "robot"

    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")

    pkg_desc_path = get_package_share_directory(pkg_desc_name)
    pkg_bringup_path = get_package_share_directory(pkg_bringup_name)

    urdf_file = os.path.join(pkg_desc_path, "urdf", "robot.urdf")
    apriltag_file = os.path.join(pkg_desc_path, "urdf", "aprilTag.urdf")
    ball_file = os.path.join(pkg_desc_path, "urdf", "ball.urdf")
    obstacle_file = os.path.join(pkg_desc_path, "urdf", "obstacle.urdf")
    field_file = os.path.join(pkg_desc_path, "urdf", "field.urdf")
    bridge_yaml = os.path.join(pkg_bringup_path, "config", "bridge.yaml")
    world_file = os.path.join(pkg_bringup_path, "worlds", "empty.sdf")
    rviz_config = os.path.join(pkg_desc_path, "rviz", "robot_view.rviz")

    with open(urdf_file) as f:
        robot_description_xml = f.read()
    with open(apriltag_file) as f:
        apriltag_xml = f.read()
    robot_description_xml = robot_description_xml.replace(
        "</robot>", apriltag_xml + "\n</robot>"
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_desc_path, ".."),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ]),
        launch_arguments={"gz_args": "-r -v 4 " + world_file}.items(),
    )

    node_spawn_field = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_field",
        arguments=[
            "-name", "field",
            "-file", field_file,
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
            "-R", "0.0", "-P", "0.0", "-Y", "1.5708",
        ],
        output="screen",
    )

    node_spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_robot",
            arguments=[
                "-name", robot_name,
                "-string", robot_description_xml,
                "-x", robot_x, "-y", robot_y, "-z", "0.0",
            ],
            output="screen",
        )],
    )

    node_spawn_ball = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_ball",
            arguments=[
                "-name", "ball",
                "-file", ball_file,
                "-x", "0.0", "-y", "0.0", "-z", "0.0335",
            ],
            output="screen",
        )],
    )

    node_spawn_obstacles = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_obstacle_{i}",
                arguments=[
                    "-name", f"obstacle_{i}",
                    "-file", obstacle_file,
                    "-x", "0.0", "-y", "0.0", "-z", "0.05",
                ],
                output="screen",
            )
            for i in range(5)
        ],
    )

    node_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="twin_view_bridge",
        arguments=[
            "--ros-args", "-p",
            f"config_file:={bridge_yaml}",
        ],
        output="screen",
    )

    node_robot_state = TimerAction(
        period=6.0,
        actions=[Node(
            package="robot_simulation",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
        )],
    )

    node_cmd_vel_state = Node(
        package="robot_simulation",
        executable="cmd_vel_state",
        name="cmd_vel_state",
        output="screen",
    )

    node_ball_state = TimerAction(
        period=6.0,
        actions=[Node(
            package="robot_simulation",
            executable="ball_state_publisher",
            name="ball_state_publisher",
            output="screen",
        )],
    )

    node_obstacle_state = TimerAction(
        period=6.0,
        actions=[Node(
            package="robot_simulation",
            executable="obstacle_state_publisher",
            name="obstacle_state_publisher",
            output="screen",
        )],
    )

    node_rviz_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_xml,
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen",
    )

    vision_overlay = Node(
        package='robot_hardware',
        executable='vision_overlay',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_x", default_value="0.0", description="Robot spawn X position"),
        DeclareLaunchArgument("robot_y", default_value="0.0", description="Robot spawn Y position"),
        set_gz_resource_path,
        gazebo_launch,
        node_spawn_field,
        node_spawn_robot,
        node_spawn_ball,
        node_spawn_obstacles,
        node_bridge,
        node_robot_state,
        node_ball_state,
        node_obstacle_state,
        node_cmd_vel_state,
        node_rviz_state_publisher,
        rviz,
        vision_overlay,
    ])
