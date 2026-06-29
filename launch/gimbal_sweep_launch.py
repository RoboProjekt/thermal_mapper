from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Isolierter Gimbal-Sweep-Test: TF + automatischer Yaw-Sweep.
    Fuer visuelle Pruefung in RViz (Fixed Frame: base_link, TF anzeigen).
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_mode',
            default_value='delta',
            description='delta = Schritte relativ zum Yaw zu Schrittbeginn',
        ),
        DeclareLaunchArgument(
            'yaw_targets_deg',
            default_value='180,-180,0',
        ),
        DeclareLaunchArgument(
            'yaw_speed',
            default_value='20',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_joint3_yaw',
            arguments=[
                '-0.0346', '0.0', '0.1554', '0.0', '0.0', '0.0',
                'base_link', 'gimbal_joint3_yaw',
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
            executable='gimbal_sweep_node',
            name='gimbal_sweep_node',
            output='screen',
            parameters=[{
                'target_mode': LaunchConfiguration('target_mode'),
                'yaw_targets_deg': LaunchConfiguration('yaw_targets_deg'),
                'yaw_speed': LaunchConfiguration('yaw_speed'),
                'auto_start': LaunchConfiguration('auto_start'),
            }],
        ),
    ])
