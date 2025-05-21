from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_color': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile':   '640x480x30',
                'pointcloud.enable':   True,
                'align_depth.enable':  True,
            }]
        ),

        Node(
            package='camera_manager',
            executable='camera_manager',
            name='camera_manager',
            output='screen'
        )
    ])
