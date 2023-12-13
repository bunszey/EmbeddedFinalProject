import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    os.system("sudo chmod a+rw /dev/ttyACM0")

    cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_node',
            parameters=[
                {'video_device': '/dev/video2'},
                {'image_width': 320},
                {'image_height': 240},
                {'pixel_format': 'yuyv'},
            ]
        )
    
    motor_node = Node(
        package="dynamixel_read_write",
        executable="read_write_node",
        name="motor_node",
    )

    return LaunchDescription([
        cam_node,
        motor_node,
    ])
