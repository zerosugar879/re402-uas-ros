from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Communication node
    comm = Node(
        package='robot_hardware',
        executable='communication_node',
        output='screen',
    )

    # Path planner node
    planner = Node(
        package='robot_hardware',
        executable='path_planner_node',
        output='screen',
    )

    # Controller node
    controller = Node(
        package='robot_hardware',
        executable='controller_node',
        output='screen',
    )

    return LaunchDescription([
        comm,
        planner,
        controller,
    ])
