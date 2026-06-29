from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Dynamischer Gimbal-Broadcaster inkl. base_link -> gimbal_joint3_yaw (Yaw).
    """
    return LaunchDescription([
        Node(
            package='thermal_mapper',
            executable='gimbal_tf_broadcaster',
            name='gimbal_tf_broadcaster',
            output='screen',
        ),
    ])
