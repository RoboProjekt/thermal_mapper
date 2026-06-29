"""Gemeinsame TF-Hilfen fuer Gimbal-Broadcaster und manuellen Test."""

import math

from geometry_msgs.msg import TransformStamped

# Statischer Offset base_link -> gimbal_joint3_yaw
BASE_TO_JOINT3_TRANSLATION = (-0.0346, 0.0, 0.1554)

GIMBAL_CHAIN = [
    (
        'gimbal_joint3_yaw', 'gimbal_joint2_roll',
        (-0.040, 0.0, 0.055), 'z',
    ),
    (
        'gimbal_joint2_roll', 'gimbal_joint1_pitch',
        (0.045, 0.0, 0.0), 'x',
    ),
    (
        'gimbal_joint1_pitch', 'siyi_lens',
        (0.010, 0.0, 0.010), 'y',
    ),
]


def euler_axis_quaternion(axis, angle_deg):
    """Quaternion fuer Rotation um eine Achse (x/y/z) in Grad."""
    angle_rad = math.radians(angle_deg)
    half = angle_rad * 0.5
    s = math.sin(half)
    c = math.cos(half)
    if axis == 'x':
        return (s, 0.0, 0.0, c)
    if axis == 'y':
        return (0.0, s, 0.0, c)
    return (0.0, 0.0, s, c)


def quat_multiply(q1, q2):
    """Hamilton-Produkt q1 * q2 (je (x, y, z, w))."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def make_static_base_transform(stamp):
    """TransformStamped base_link -> gimbal_joint3_yaw."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'gimbal_joint3_yaw'
    t.transform.translation.x = BASE_TO_JOINT3_TRANSLATION[0]
    t.transform.translation.y = BASE_TO_JOINT3_TRANSLATION[1]
    t.transform.translation.z = BASE_TO_JOINT3_TRANSLATION[2]
    t.transform.rotation.w = 1.0
    return t


def send_gimbal_transforms(
    broadcaster,
    stamp,
    yaw,
    pitch,
    roll,
    lens_mount_pitch_deg=180.0,
):
    """Publiziert statischen Basis-TF und dynamische Gimbal-Kette."""
    broadcaster.sendTransform(make_static_base_transform(stamp))

    joint_angles = [yaw, roll, pitch]
    for i, (parent, child, translation, axis) in enumerate(GIMBAL_CHAIN):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        qx, qy, qz, qw = euler_axis_quaternion(axis, joint_angles[i])
        if child == 'siyi_lens' and lens_mount_pitch_deg != 0.0:
            q_mount = euler_axis_quaternion('y', lens_mount_pitch_deg)
            qx, qy, qz, qw = quat_multiply((qx, qy, qz, qw), q_mount)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        broadcaster.sendTransform(t)
