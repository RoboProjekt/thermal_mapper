from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Startet alle Nodes fuer Thermal Mapping."""
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
        Node(
            package='thermal_mapper',
            executable='thermal_camera_publisher',
            name='thermal_camera_publisher',
            output='screen',
        ),
        Node(
            package='thermal_mapper',
            executable='thermal_projection_node',
            name='thermal_projection_node',
            output='screen',
        ),
    ])
