from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os

def generate_launch_description():
    model_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share', 'turtlebot3_description', 'urdf', 'turtlebot3_burger.urdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value="/home/sayantani/Documents/Spring/ComputerVision/Visual-Pose-Graph-Estimation/turtlebot3_description/urdf",
            description='Absolute path to robot urdf file'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),
        # static transform publisher for camera and base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link_transform',
            output='screen',
            arguments=['0', '0', '0.5', '0', '0', '0', 'odom', 'base_link']
        ),
        # Include the pose_estimation launch file
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    '/home/sayantani/Documents/Spring/ComputerVision/Visual-Pose-Graph-Estimation',
                    'pose_estimation.launch.xml'
                )
            )
        )
    ])