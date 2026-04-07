"""Camera-LiDAR Fusion Node (v 0.06)

Subscribes to YOLO Detection2DArray and a depth Image, fuses them to produce
SemanticObstacleArray messages for the Nav2 perception costmap layer.

Pipeline:
  1. ApproximateTime-sync Detection2DArray + depth Image
  2. Back-project depth pixels to 3-D points in camera optical frame
  3. Voxel downsample + RANSAC floor-plane removal
  4. Project surviving 3-D points to 2-D pixels, match YOLO bounding boxes
  5. 1-D depth-gap clustering — select nearest compact cluster per box
  6. Temporal tracking with EMA position smoothing
  7. Publish SemanticObstacleArray for confirmed tracks + debug MarkerArray

Quick Test:
    # Terminal 1  tb4 yolo viz 
    # Terminal 2: tb4 fusion viz
    # Terminal 3: ros2 topic echo /semantic_obstacles      
    # Terminal 4 (optional): rviz2 with "odom" fixed frame, 
        Add -> By topic -> /fusion_debug_markers -> MarkerArray 
        Place a person, chair, or stop-sign in front of the camera

Perf Test:
    # Terminal 1  tb4 yolo 
    # Terminal 2: tb4 fusion
    # Terminal 3: tb4 bbox_rate         (i.e. ros2 topic hz /detections )
    # Terminal 4: tb4 depth_cam_rate    (i.e. ros2 topic hz /oakd/stereo/image_raw)
    # Terminal 5: tb4 pointcloud_topic  (i.e. ros2 topic hz /oakd/stereo/points)
    # Terminal 6: tb4 obstacles         (i.e. ros2 topic hz /semantic_obstacles)    
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time, Duration

import message_filters
import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point

from tb4_perception_layer.msg import SemanticObstacle, SemanticObstacleArray

# ── Helpers ───────────────────────────────────────────────────────────────────
def quaternion_matrix(quaternion):
    """Return 4x4 homogeneous rotation matrix from quaternion [x, y, z, w]."""
    x, y, z, w = quaternion
    tx, ty, tz = 2.0 * x, 2.0 * y, 2.0 * z
    wx, wy, wz = w * tx, w * ty, w * tz
    xx, xy, xz = x * tx, x * ty, x * tz
    yy, yz, zz = y * ty, y * tz, z * tz
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy, 0.0],
        [xy + wz, 1.0 - (xx + zz), yz - wx, 0.0],
        [xz - wy, yz + wx, 1.0 - (xx + yy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

def transform_to_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation

    # Convert quaternion -> rotation matrix (4x4)
    T = quaternion_matrix([q.x, q.y, q.z, q.w])

    # Set translation
    T[0:3, 3] = [t.x, t.y, t.z]

    return T

# If voxel_downsample(pts, 0.05) is called after NaN filtering and before RANSAC then 
# this typically reduces point count by ~10x on a 480×300 depth image
# making RANSAC and all downstream steps much faster
def voxel_downsample(points, voxel_size):
    """Downsample point cloud by averaging points within each voxel."""

    if voxel_size <= 0 or points.shape[0] == 0:
        return points

    # Quantize to voxel grid
    keys = np.floor(points / voxel_size).astype(np.int32)

    # Unique voxels — use structured view for fast unique
    _, idx, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    
    # Accumulate and average
    n_voxels = counts.shape[0]
    sums = np.zeros((n_voxels, 3), dtype=np.float64)
    np.add.at(sums, idx, points)
    
    return sums / counts[:, None]

def ransac_floor_removal(points, iterations, dist_thresh, normal_tol):
    """Remove floor plane via RANSAC. Returns non-floor points (nx3)."""
    n = points.shape[0]
    if n < 3:
        return points

    best_inliers = np.zeros(n, dtype=bool)
    best_count = 0
    rng = np.random.default_rng()

    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx]
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-8:
            continue
        normal /= norm_len

        # Check if normal is roughly vertical (z-component dominant)
        angle_to_vertical = np.arccos(np.clip(abs(normal[2]), 0.0, 1.0))
        if angle_to_vertical > normal_tol:
            continue

        d = -np.dot(normal, p0)
        dists = np.abs(points @ normal + d)
        inliers = dists < dist_thresh
        count = int(np.sum(inliers))

        if count > best_count:
            best_count = count
            best_inliers = inliers

    return points[~best_inliers]

def depth_gap_cluster(depths, gap_threshold):
    """
    1-D depth-gap clustering.

    Sort depths, split where consecutive gap > threshold.
    Return list of index arrays (into the sorted order).
    """
    order = np.argsort(depths)
    sorted_d = depths[order]
    gaps = np.diff(sorted_d)
    split_idx = np.where(gaps > gap_threshold)[0] + 1
    clusters = np.split(order, split_idx)
    return clusters

# ── Color palette for debug markers ──────────────────────────────────────────

CLASS_COLORS = {
    'person':    (1.0, 0.2, 0.2),
    'chair':     (0.2, 0.6, 1.0),
    'dog':       (0.9, 0.6, 0.1),
    'cat':       (0.8, 0.4, 0.8),
    'bicycle':   (0.2, 0.9, 0.3),
    'stop sign': (1.0, 0.0, 0.0),
}

# ── Node ──────────────────────────────────────────────────────────────────────

class CameraLidarFusionNode(Node):

    def __init__(self):
        super().__init__('camera_lidar_fusion_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('depth_image_topic',
                               '/oakd/stereo/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/oakd/stereo/camera_info')
        self.declare_parameter('output_topic', '/semantic_obstacles')
        self.declare_parameter('marker_topic', '/fusion_debug_markers')
        self.declare_parameter('publish_debug_markers', False)

        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('sync_queue_size', 10)

        # Voxel downsampling
        self.declare_parameter('voxel_size', 0.05)

        # RANSAC
        self.declare_parameter('ransac_iterations', 100)
        self.declare_parameter('ransac_distance_threshold', 0.02)
        self.declare_parameter('floor_normal_tolerance', 0.3)

        # Matching / clustering
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('depth_gap_threshold', 0.3)
        self.declare_parameter('max_obstacle_distance', 4.0)

        # Tracking
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('association_distance', 0.5)
        self.declare_parameter('min_hits_to_confirm', 3)
        self.declare_parameter('max_misses_to_delete', 5)
        self.declare_parameter('track_timeout', 1.0)

        # Class-specific config (parallel arrays)
        self.declare_parameter('class_names', [
            'person', 'chair', 'dog', 'cat', 'bicycle', 'stop sign',
        ])
        self.declare_parameter('class_radii', [
            0.3, 0.25, 0.2, 0.15, 0.4, 0.15,
        ])
        self.declare_parameter('class_costs', [
            254.0, 254.0, 200.0, 180.0, 254.0, 200.0,
        ])

        # Read all parameters
        self.det_topic = self.get_parameter('detection_topic').value
        self.depth_topic = self.get_parameter('depth_image_topic').value
        self.ci_topic = self.get_parameter('camera_info_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.publish_markers = self.get_parameter('publish_debug_markers').value

        self.sync_slop = self.get_parameter('sync_slop').value
        self.sync_queue = self.get_parameter('sync_queue_size').value

        self.voxel_size = self.get_parameter('voxel_size').value

        self.ransac_iter = self.get_parameter('ransac_iterations').value
        self.ransac_dist = self.get_parameter('ransac_distance_threshold').value
        self.floor_tol = self.get_parameter('floor_normal_tolerance').value

        self.min_pts = self.get_parameter('min_cluster_points').value
        self.gap_thresh = self.get_parameter('depth_gap_threshold').value
        self.max_dist = self.get_parameter('max_obstacle_distance').value

        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.assoc_dist = self.get_parameter('association_distance').value
        self.min_hits = self.get_parameter('min_hits_to_confirm').value
        self.max_misses = self.get_parameter('max_misses_to_delete').value
        self.track_timeout = self.get_parameter('track_timeout').value

        cnames = self.get_parameter('class_names').value
        cradii = self.get_parameter('class_radii').value
        ccosts = self.get_parameter('class_costs').value
        self.class_radius = dict(zip(cnames, cradii))
        self.class_cost = dict(zip(cnames, ccosts))

        # ── TF2 ──────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── CameraInfo (cached, not synced) ───────────────────────────
        self.K = None  # 3×3 intrinsics
        self.cam_frame = None
        self.img_w = 0
        self.img_h = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.ci_sub = self.create_subscription(
            CameraInfo, self.ci_topic,
            self.camera_info_cb, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────
        self.obs_pub = self.create_publisher(
            SemanticObstacleArray, self.out_topic, 10)
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, self.marker_topic, 10)
        else:
            self.marker_pub = None

        # ── Synchronized subscribers ──────────────────────────────────
        self.det_sub = message_filters.Subscriber(
            self, Detection2DArray, self.det_topic,
            qos_profile=10)
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic,
            qos_profile=sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.det_sub, self.depth_sub],
            queue_size=self.sync_queue,
            slop=self.sync_slop)
        self.sync.registerCallback(self.synced_callback)

        # ── Tracker state ─────────────────────────────────────────────
        self.tracks: list[dict] = []
        self.next_track_id = 0

        self.get_logger().info(
            f'CameraLidarFusionNode ready — '
            f'det={self.det_topic}, depth={self.depth_topic}, '
            f'out={self.out_topic}')

    # ── CameraInfo cache ──────────────────────────────────────────────

    def camera_info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if np.all(K == 0):
            return
        self.K = K
        self.cam_frame = msg.header.frame_id
        self.img_w = msg.width
        self.img_h = msg.height

    # ── Main synced callback ──────────────────────────────────────────

    def synced_callback(self, det_msg: Detection2DArray, depth_msg: Image):
        if self.K is None:
            self.get_logger().warn(
                'No CameraInfo received yet — skipping', throttle_duration_sec=5.0)
            return

        if len(det_msg.detections) == 0:
            self.update_tracker([], det_msg.header.stamp)
            return

        cam_frame = self.cam_frame
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        # (1) Convert depth image -> Nx3 points in camera optical frame
        if depth_msg.encoding == '16UC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                depth_msg.height, depth_msg.width).astype(np.float64) / 1000.0
        elif depth_msg.encoding == '32FC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                depth_msg.height, depth_msg.width).astype(np.float64)
        else:
            self.get_logger().warn(
                f'Unsupported depth encoding: {depth_msg.encoding}',
                throttle_duration_sec=5.0)
            return

        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(
            np.arange(w, dtype=np.float64),
            np.arange(h, dtype=np.float64))

        valid = (depth > 0) & np.isfinite(depth)
        Z = depth[valid]
        u_px = u_grid[valid]
        v_px = v_grid[valid]

        X = (u_px - cx) * Z / fx
        Y = (v_px - cy) * Z / fy
        pts_cam = np.stack([X, Y, Z], axis=-1)       # Nx3

        if pts_cam.shape[0] < 10:
            return

        # (2) Voxel downsample
        if self.voxel_size > 0:
            pts_cam = voxel_downsample(pts_cam, self.voxel_size)

        # (3) RANSAC floor-plane removal
        pts_cam = ransac_floor_removal(
            pts_cam, self.ransac_iter, self.ransac_dist, self.floor_tol)
        if pts_cam.shape[0] < self.min_pts:
            return

        # (4) Re-project to 2-D pixels for YOLO box matching
        Z = pts_cam[:, 2]
        valid_z = Z > 0.0
        pts_cam = pts_cam[valid_z]
        Z = Z[valid_z]

        u = (fx * pts_cam[:, 0] / Z + cx).astype(np.float64)
        v = (fy * pts_cam[:, 1] / Z + cy).astype(np.float64)

        in_bounds = (
            (u >= 0) & (u < w) &
            (v >= 0) & (v < h)
        )
        u = u[in_bounds]
        v = v[in_bounds]
        pts_cam = pts_cam[in_bounds]
        Z = Z[in_bounds]

        if pts_cam.shape[0] == 0:
            return

        # (5) Match YOLO boxes -> projected points
        new_detections: list[dict] = []

        for det in det_msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            class_id = hyp.class_id
            confidence = hyp.score

            bx = det.bbox.center.position.x
            by = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            x1 = bx - bw / 2.0
            y1 = by - bh / 2.0
            x2 = bx + bw / 2.0
            y2 = by + bh / 2.0

            inside = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
            if int(np.sum(inside)) < self.min_pts:
                continue

            matched_pts = pts_cam[inside]
            matched_Z = Z[inside]

            # (6) 1-D depth-gap clustering
            clusters = depth_gap_cluster(matched_Z, self.gap_thresh)
            if len(clusters) == 0:
                continue

            # Select the nearest cluster
            best_cluster = None
            best_median = float('inf')
            for cl in clusters:
                if len(cl) < self.min_pts:
                    continue
                med = float(np.median(matched_Z[cl]))
                if med < best_median:
                    best_median = med
                    best_cluster = cl

            if best_cluster is None or best_median > self.max_dist:
                continue

            centroid_cam = matched_pts[best_cluster].mean(axis=0)  # 3-D

            new_detections.append({
                'class_id': class_id,
                'confidence': confidence,
                'centroid_cam': centroid_cam,
            })

        # Transform centroids from camera_frame -> odom for tracking
        odom_frame = 'odom'
        try:
            tf_cam_odom = self.tf_buffer.lookup_transform(
                odom_frame, cam_frame,
                Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(
                f'TF {cam_frame}->{odom_frame}: {e}',
                throttle_duration_sec=5.0)
            return

        mat_odom = transform_to_matrix(tf_cam_odom)

        for d in new_detections:
            c = d['centroid_cam']
            pt_h = np.array([c[0], c[1], c[2], 1.0])
            pt_odom = mat_odom @ pt_h
            d['x'] = float(pt_odom[0])
            d['y'] = float(pt_odom[1])

        # (7) Temporal tracking with EMA
        self.update_tracker(new_detections, det_msg.header.stamp)

    # ── Tracker ───────────────────────────────────────────────────────

    def update_tracker(self, new_dets: list[dict], stamp):
        now = Time.from_msg(stamp) if not isinstance(stamp, Time) else stamp
        now_sec = now.nanoseconds / 1e9

        matched_track_ids: set[int] = set()

        for d in new_dets:
            best_track = None
            best_dist = float('inf')

            for t in self.tracks:
                if t['class_id'] != d['class_id']:
                    continue
                dist = np.hypot(t['x'] - d['x'], t['y'] - d['y'])
                if dist < best_dist and dist < self.assoc_dist:
                    best_dist = dist
                    best_track = t

            if best_track is not None:
                # Update existing track (EMA)
                a = self.ema_alpha
                best_track['x'] = a * d['x'] + (1 - a) * best_track['x']
                best_track['y'] = a * d['y'] + (1 - a) * best_track['y']
                best_track['confidence'] = d['confidence']
                best_track['hit_count'] += 1
                best_track['miss_count'] = 0
                best_track['last_seen'] = now_sec
                matched_track_ids.add(best_track['track_id'])
            else:
                # Create new track
                tid = self.next_track_id
                self.next_track_id += 1
                self.tracks.append({
                    'track_id': tid,
                    'class_id': d['class_id'],
                    'confidence': d['confidence'],
                    'x': d['x'],
                    'y': d['y'],
                    'hit_count': 1,
                    'miss_count': 0,
                    'last_seen': now_sec,
                })
                matched_track_ids.add(tid)

        # Increment misses for unmatched tracks
        for t in self.tracks:
            if t['track_id'] not in matched_track_ids:
                t['miss_count'] += 1

        # Delete stale tracks
        deleted_ids = []
        keep = []
        for t in self.tracks:
            age_since_seen = now_sec - t['last_seen']
            if (t['miss_count'] >= self.max_misses or
                    age_since_seen > self.track_timeout):
                deleted_ids.append(t['track_id'])
            else:
                keep.append(t)
        self.tracks = keep

        # ── Publish SemanticObstacleArray (confirmed tracks only) ─────
        obs_msg = SemanticObstacleArray()
        obs_msg.header = Header()
        obs_msg.header.stamp = self.get_clock().now().to_msg()
        obs_msg.header.frame_id = 'odom'

        for t in self.tracks:
            if t['hit_count'] < self.min_hits:
                continue
            obs = SemanticObstacle()
            obs.header = obs_msg.header
            obs.class_id = t['class_id']
            obs.confidence = t['confidence']
            obs.x = t['x']
            obs.y = t['y']
            obs.radius = self.class_radius.get(
                t['class_id'], 0.3)
            obs.cost = self.class_cost.get(
                t['class_id'], 254.0)
            obs.track_id = t['track_id']
            obs_msg.obstacles.append(obs)

        self.obs_pub.publish(obs_msg)

        # ── Publish debug MarkerArray ─────────────────────────────────
        if self.marker_pub is not None:
            marker_msg = MarkerArray()

            for t in self.tracks:
                if t['hit_count'] < self.min_hits:
                    continue

                r_val = self.class_radius.get(t['class_id'], 0.3)
                rgb = CLASS_COLORS.get(t['class_id'], (0.5, 0.5, 0.5))

                # Cylinder marker
                m = Marker()
                m.header.stamp = obs_msg.header.stamp
                m.header.frame_id = 'odom'
                m.ns = 'semantic_obstacles'
                m.id = t['track_id']
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = t['x']
                m.pose.position.y = t['y']
                m.pose.position.z = 0.3
                m.scale.x = r_val * 2.0
                m.scale.y = r_val * 2.0
                m.scale.z = 0.6
                m.color = ColorRGBA(
                    r=rgb[0], g=rgb[1], b=rgb[2], a=0.6)
                m.lifetime.sec = 1
                marker_msg.markers.append(m)

                # Text label
                txt = Marker()
                txt.header = m.header
                txt.ns = 'semantic_labels'
                txt.id = t['track_id']
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = t['x']
                txt.pose.position.y = t['y']
                txt.pose.position.z = 0.8
                txt.scale.z = 0.15
                txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                txt.text = (
                    f"{t['class_id']} #{t['track_id']} "
                    f"({t['confidence']:.2f})")
                txt.lifetime.sec = 1
                marker_msg.markers.append(txt)

            # Delete markers for removed tracks
            for tid in deleted_ids:
                for ns in ('semantic_obstacles', 'semantic_labels'):
                    dm = Marker()
                    dm.header.stamp = obs_msg.header.stamp
                    dm.header.frame_id = 'odom'
                    dm.ns = ns
                    dm.id = tid
                    dm.action = Marker.DELETE
                    marker_msg.markers.append(dm)

            self.marker_pub.publish(marker_msg)

        if obs_msg.obstacles:
            summary = ', '.join(
                f'{o.class_id} #{o.track_id} ({o.confidence:.2f})'
                for o in obs_msg.obstacles)
            self.get_logger().debug(
                f'Published {len(obs_msg.obstacles)} obstacles: {summary}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
