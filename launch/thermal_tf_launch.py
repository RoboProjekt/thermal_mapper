from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Statischer Basis-TF (base_link -> gimbal_joint3_yaw) plus dynamischer
    Gimbal-Broadcaster fuer die drei Gelenk-Transforms.
    """
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_joint3_yaw',
            arguments=[
                '-0.0346', '0.0', '0.1554', '0.0', '0.0', '0.0',
                'base_link', 'gimbal_joint3_yaw'
            ],
        ),
        Node(
            package='thermal_mapper',
            executable='gimbal_tf_broadcaster',
            name='gimbal_tf_broadcaster',
            output='screen',
        ),
    ])
