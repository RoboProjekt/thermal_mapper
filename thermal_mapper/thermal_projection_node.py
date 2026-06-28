"""Voxel-zu-Pixel Projektion mit vektorisierter Pinhole-Mathematik und PointCloud2-Output."""

import pickle

import cv2  # Muss vor cv_bridge importiert werden
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:
    pc2 = None

from thermal_mapper.temperature_utils import grays_to_celsius, temperatures_to_rgb


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


def z_buffer_nearest_per_pixel(ui, vi, z, global_indices, img_w, img_h):
    """
    Vektorisierter Z-Buffer: pro Pixel nur der naechste Voxel (kleinstes Z).
    Gibt (global_indices, ui, vi) der Gewinner zurueck.
    """
    in_bounds = (
        (ui >= 0) & (ui < img_w) &
        (vi >= 0) & (vi < img_h)
    )
    if not np.any(in_bounds):
        return (
            np.array([], dtype=np.int64),
            np.array([], dtype=np.int32),
            np.array([], dtype=np.int32),
        )

    ui = ui[in_bounds].astype(np.int32)
    vi = vi[in_bounds].astype(np.int32)
    z = z[in_bounds]
    g = global_indices[in_bounds]

    order = np.argsort(z)
    pixel_id = vi[order].astype(np.int64) * img_w + ui[order].astype(np.int64)
    ui = ui[order]
    vi = vi[order]
    g = g[order]

    _, winner_idx = np.unique(pixel_id, return_index=True)
    return g[winner_idx], ui[winner_idx], vi[winner_idx]


def build_xyzrgb_cloud(header, points, colors_uint8):
    """Erzeugt sensor_msgs/PointCloud2 mit XYZ + RGB (vektorisiert)."""
    n = points.shape[0]
    structured = np.empty(n, dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)
    ])
    structured['x'] = points[:, 0].astype(np.float32)
    structured['y'] = points[:, 1].astype(np.float32)
    structured['z'] = points[:, 2].astype(np.float32)
    rgba = np.empty((n, 4), dtype=np.uint8)
    rgba[:, 0] = colors_uint8[:, 0]
    rgba[:, 1] = colors_uint8[:, 1]
    rgba[:, 2] = colors_uint8[:, 2]
    rgba[:, 3] = 255
    structured['rgb'] = rgba.view(np.uint32).reshape(-1)

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]
    if pc2 is not None:
        cloud = pc2.create_cloud(header, fields, structured)
    else:
        cloud = PointCloud2(
            header=header,
            height=1,
            width=n,
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * n,
            data=structured.tobytes(),
        )

    return cloud


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
        self.declare_parameter('cloud_topic', '/thermal/voxel_cloud')
        self.declare_parameter('map_cloud_topic', '/thermal/map_cloud')
        self.declare_parameter('map_cloud_color', 160)
        self.declare_parameter('frame_skip', 5)
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
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.map_cloud_topic = self.get_parameter('map_cloud_topic').value
        map_cloud_color = int(self.get_parameter('map_cloud_color').value)
        self.map_cloud_color = np.clip(map_cloud_color, 0, 255)
        self.frame_skip = max(1, int(self.get_parameter('frame_skip').value))
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
            f'voxel_size={self.voxel_size:.3f} m, frame_skip={self.frame_skip}'
        )
        if pc2 is None:
            self.get_logger().warn(
                'sensor_msgs_py nicht gefunden – PointCloud2-Fallback aktiv. '
                'Installieren: sudo apt install ros-foxy-sensor-msgs-py'
            )

        self.temp_accum = np.zeros(len(self.voxel_centers), dtype=np.float64)
        self.temp_counts = np.zeros(len(self.voxel_centers), dtype=np.int32)
        self._map_cloud_logged = False

        self.bridge = CvBridge()
        self._frame_counter = 0
        self.image_received = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)
        # VOLATILE QoS: RViz-Default; periodisches Republish statt TRANSIENT_LOCAL
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, self.map_cloud_topic, 10
        )
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.publish_map_cloud()
        self.create_timer(5.0, self.publish_map_cloud)

    def image_callback(self, msg):
        if not self.image_received:
            self.get_logger().info(
                f'Bild empfangen: {msg.width}x{msg.height}, encoding={msg.encoding}'
            )
            self.image_received = True

        self._frame_counter += 1
        if self._frame_counter % self.frame_skip != 0:
            return

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
        optical = np.empty_like(points_cam)
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

        # Vektorisiert: alle Voxel map -> Kamera
        ones = np.ones((len(self.voxel_centers), 1), dtype=np.float64)
        pts_cam = (mat @ np.hstack([self.voxel_centers, ones]).T).T[:, :3]
        pts_cam = self._to_optical(pts_cam)

        z = pts_cam[:, 2]
        valid = z > 0.01
        if not np.any(valid):
            return

        global_indices = np.nonzero(valid)[0]
        pts = pts_cam[valid]
        z = pts[:, 2]

        # Vektorisierte Pinhole-Projektion
        u = self.fx * pts[:, 0] / z + self.cx
        v = self.fy * pts[:, 1] / z + self.cy
        ui = np.round(u).astype(np.int32)
        vi = np.round(v).astype(np.int32)

        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if gray.ndim == 3:
            gray = gray[:, :, 0]

        winners, win_ui, win_vi = z_buffer_nearest_per_pixel(
            ui, vi, z, global_indices, self.img_w, self.img_h
        )
        if winners.size == 0:
            return

        gray_vals = gray[win_vi, win_ui].astype(np.float64)
        temps = grays_to_celsius(gray_vals, self.temp_min, self.temp_max)

        # Laufender Temperatur-Mittelwert (vektorisiert pro Frame)
        self.temp_accum[winners] += temps
        self.temp_counts[winners] += 1

        self.publish_point_cloud(msg.header.stamp, winners)

    def publish_map_cloud(self):
        """Publiziert die geladene Voxelkarte als neutrale PointCloud2 (alle 5s)."""
        n = len(self.voxel_centers)
        gray = int(self.map_cloud_color)
        colors = np.full((n, 3), gray, dtype=np.uint8)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        cloud = build_xyzrgb_cloud(header, self.voxel_centers, colors)
        self.map_cloud_pub.publish(cloud)

        if not self._map_cloud_logged:
            self.get_logger().info(
                f'Karten-PointCloud: {n} Punkte auf {self.map_cloud_topic} (Republish alle 5s)'
            )
            self._map_cloud_logged = True

    def publish_point_cloud(self, stamp, visible_indices):
        counts = self.temp_counts[visible_indices]
        if np.all(counts == 0):
            return

        avg_temps = self.temp_accum[visible_indices] / counts
        points = self.voxel_centers[visible_indices]
        colors = temperatures_to_rgb(avg_temps, self.temp_min, self.temp_max)

        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame

        cloud = build_xyzrgb_cloud(header, points, colors)
        self.cloud_pub.publish(cloud)


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
