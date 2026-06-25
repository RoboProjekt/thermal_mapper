"""Voxel-zu-Pixel Projektion mit Z-Buffer, Splatting und Temperatur-Mapping."""

import pickle

import cv2  # Muss vor cv_bridge importiert werden
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from thermal_mapper.temperature_utils import gray_to_celsius, temperature_to_color


def transform_to_matrix(translation, rotation):
    """4x4 Homogene Transformationsmatrix aus Translation + Quaternion."""
    rot_obj = Rotation.from_quat([
        rotation.x, rotation.y, rotation.z, rotation.w
    ])
    # scipy < 1.4: as_dcm(); scipy >= 1.4: as_matrix()
    rot = rot_obj.as_matrix() if hasattr(rot_obj, 'as_matrix') else rot_obj.as_dcm()
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = [translation.x, translation.y, translation.z]
    return mat


def load_voxel_map(path):
    """Laedt R3D .pkl und berechnet Voxel-Zentren in Weltkoordinaten."""
    with open(path, 'rb') as handle:
        data = pickle.load(handle)

    if not isinstance(data, dict) or 'graph' not in data:
        raise ValueError("Erwarte Dictionary mit 'graph', 'origin', 'voxel_size'.")

    graph = data['graph']
    origin = np.asarray(data['origin'], dtype=np.float64)
    voxel_size = float(data['voxel_size'])

    nodes = list(graph.nodes())
    centers = np.zeros((len(nodes), 3), dtype=np.float64)
    for idx, (vx, vy, vz) in enumerate(nodes):
        centers[idx, 0] = origin[0] + vx * voxel_size + voxel_size / 2.0
        centers[idx, 1] = origin[1] + vy * voxel_size + voxel_size / 2.0
        centers[idx, 2] = origin[2] + vz * voxel_size + voxel_size / 2.0

    return nodes, centers, voxel_size


class ThermalProjectionNode(Node):
    def __init__(self):
        super().__init__('thermal_projection_node')

        self.declare_parameter(
            'map_path',
            '/home/unitree/Desktop/Transfer_Bachelor/min_hits_12.pkl'
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'siyi_lens')
        self.declare_parameter('image_topic', '/siyi/thermal/image_raw')
        self.declare_parameter('marker_topic', '/thermal/voxel_markers')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 512)
        self.declare_parameter('fx', 320.0)
        self.declare_parameter('fy', 320.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 256.0)
        self.declare_parameter('temp_min', 0.0)
        self.declare_parameter('temp_max', 100.0)
        self.declare_parameter('use_optical_frame', True)

        map_path = self.get_parameter('map_path').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.image_topic = self.get_parameter('image_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.temp_min = float(self.get_parameter('temp_min').value)
        self.temp_max = float(self.get_parameter('temp_max').value)
        self.use_optical_frame = self.get_parameter('use_optical_frame').value

        self.voxel_nodes, self.voxel_centers, self.voxel_size = load_voxel_map(map_path)
        self.get_logger().info(
            f'Karte geladen: {len(self.voxel_nodes)} Voxel, '
            f'voxel_size={self.voxel_size:.3f} m'
        )

        # Laufender Temperatur-Mittelwert pro Voxel-Index
        self.temp_accum = {}
        self.temp_counts = {}

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

    def image_callback(self, msg):
        self.latest_image = msg
        if not self.image_received:
            self.get_logger().info(
                f'Bild empfangen: {msg.width}x{msg.height}, encoding={msg.encoding}'
            )
            self.image_received = True

        try:
            self.process_frame(msg)
        except Exception as exc:
            self.get_logger().warn(f'Projektion fehlgeschlagen: {exc}')

    def _lookup_transform(self):
        return self.tf_buffer.lookup_transform(
            self.camera_frame,
            self.map_frame,
            rclpy.time.Time()
        )

    def _to_optical(self, points_cam):
        """Body-Frame (x vorne) -> optischer Frame (z vorne, x rechts, y unten)."""
        if not self.use_optical_frame:
            return points_cam
        x = points_cam[:, 0]
        y = points_cam[:, 1]
        z = points_cam[:, 2]
        optical = np.zeros_like(points_cam)
        optical[:, 0] = -y
        optical[:, 1] = -z
        optical[:, 2] = x
        return optical

    def process_frame(self, msg):
        try:
            tf_msg = self._lookup_transform()
        except Exception:
            self.get_logger().warn(
                f'TF {self.map_frame} -> {self.camera_frame} nicht verfuegbar',
                throttle_duration_sec=5.0
            )
            return

        mat = transform_to_matrix(
            tf_msg.transform.translation,
            tf_msg.transform.rotation
        )

        # Homogene Koordinaten: map -> camera
        ones = np.ones((len(self.voxel_centers), 1))
        pts_h = np.hstack([self.voxel_centers, ones])
        pts_cam = (mat @ pts_h.T).T[:, :3]
        pts_cam = self._to_optical(pts_cam)

        z = pts_cam[:, 2]
        valid = z > 0.01
        if not np.any(valid):
            return

        indices = np.where(valid)[0]
        pts = pts_cam[valid]
        z = pts[:, 2]

        u = self.fx * pts[:, 0] / z + self.cx
        v = self.fy * pts[:, 1] / z + self.cy

        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if gray.ndim == 3:
            gray = gray[:, :, 0]

        z_buffer = np.full((self.img_h, self.img_w), np.inf, dtype=np.float32)
        owner = np.full((self.img_h, self.img_w), -1, dtype=np.int32)

        splat_radius = self.voxel_size * self.fx / z
        half = np.maximum(1, np.round(splat_radius * 0.5).astype(np.int32))

        for local_i, global_i in enumerate(indices):
            ui = int(round(u[local_i]))
            vi = int(round(v[local_i]))
            if ui < 0 or ui >= self.img_w or vi < 0 or vi >= self.img_h:
                continue

            r = int(half[local_i])
            u0 = max(0, ui - r)
            u1 = min(self.img_w, ui + r + 1)
            v0 = max(0, vi - r)
            v1 = min(self.img_h, vi + r + 1)

            depth = z[local_i]
            region_z = z_buffer[v0:v1, u0:u1]
            update_mask = depth < region_z
            if not np.any(update_mask):
                continue

            z_buffer[v0:v1, u0:u1][update_mask] = depth
            owner[v0:v1, u0:u1][update_mask] = global_i

        # Temperatur aus Grauwert am Splat-Zentrum fuer sichtbare Voxel
        visible = set(owner[owner >= 0])
        for global_i in visible:
            local_matches = np.where(indices == global_i)[0]
            if len(local_matches) == 0:
                continue
            local_i = local_matches[0]
            ui = int(np.clip(round(u[local_i]), 0, self.img_w - 1))
            vi = int(np.clip(round(v[local_i]), 0, self.img_h - 1))
            gray_val = float(gray[vi, ui])
            temp = gray_to_celsius(gray_val, self.temp_min, self.temp_max)

            if global_i not in self.temp_accum:
                self.temp_accum[global_i] = temp
                self.temp_counts[global_i] = 1
            else:
                self.temp_accum[global_i] += temp
                self.temp_counts[global_i] += 1

        self.publish_markers()

    def publish_markers(self):
        if not self.temp_accum:
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'thermal_voxels'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size

        for global_i, total in self.temp_accum.items():
            count = self.temp_counts[global_i]
            temp = total / count
            pt = Point()
            pt.x = float(self.voxel_centers[global_i, 0])
            pt.y = float(self.voxel_centers[global_i, 1])
            pt.z = float(self.voxel_centers[global_i, 2])
            marker.points.append(pt)
            r, g, b = temperature_to_color(temp, self.temp_min, self.temp_max)
            color = ColorRGBA()
            color.r = float(r)
            color.g = float(g)
            color.b = float(b)
            color.a = 1.0
            marker.colors.append(color)

        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
