from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Baut den kompletten TF-Tree des Unitree Go2 inkl. SIYI ZT6 Gimbal auf.
    Parameter-Reihenfolge: x y z yaw pitch roll frame_id child_frame_id
    """
    return LaunchDescription([
        # --- Gimbal Struktur (in Metern!) ---
        # Basis direkt zu joint 3 (Gimbal Aufhängung am Roboter)
        # Berechnet aus Hesai(0.1384, 0, 0.1284) + Offset(-0.173, 0, 0.027)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_joint3_yaw',
            arguments=['-0.0346', '0.0', '0.1554', '0.0', '0.0', '0.0', 'base_link', 'gimbal_joint3_yaw']
        ),
        # joint 3 zu joint 2 (Drehung um X / Roll)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='joint3_to_joint2_roll',
            arguments=['-0.040', '0.0', '0.055', '0.0', '0.0', '0.0', 'gimbal_joint3_yaw', 'gimbal_joint2_roll']
        ),
        # joint 2 zu joint 1 (Drehung um Y / Pitch)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='joint2_to_joint1_pitch',
            arguments=['0.045', '0.0', '0.0', '0.0', '0.0', '0.0', 'gimbal_joint2_roll', 'gimbal_joint1_pitch']
        ),
        # joint 1 zu Linse (Kamera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='joint1_to_lens',
            arguments=[
                '0.010', '0.0', '0.010', '0.0', '3.141592653589793', '0.0',
                'gimbal_joint1_pitch', 'siyi_lens'
            ]
        )
    ])