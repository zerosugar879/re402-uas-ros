from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # USB camera
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'framerate': 30.0,
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ]
    )

    # Vision node
    vision = Node(
        package='robot_hardware',
        executable='vision_node',
        output='screen',
    )

    return LaunchDescription([
        usb_cam,
        vision,
    ])
