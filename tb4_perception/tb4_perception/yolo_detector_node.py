"""
YOLO Object Detection Node for TurtleBot4 Standard.

Subscribes to the OAK-D camera RGB image, runs YOLOv8 inference,
and publishes Detection2DArray messages on /detections.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_rate_limit', 12.0)
        self.declare_parameter('target_classes', [
            'person', 'stop sign', 'chair', 'dog', 'cat', 'bicycle',
        ])

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        device = self.get_parameter('device').value
        self.rate_limit = self.get_parameter('publish_rate_limit').value
        self.target_classes = self.get_parameter('target_classes').value

        # --- YOLO model ---
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.class_names = self.model.names  # {0: 'person', 1: 'bicycle', ...}

        # Build a set of target class indices for fast lookup
        self.target_indices: set[int] = set()
        for idx, name in self.class_names.items():
            if name in self.target_classes:
                self.target_indices.add(idx)
        self.get_logger().info(
            f'Tracking classes: {[self.class_names[i] for i in sorted(self.target_indices)]}'
        )

        # --- ROS I/O ---
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray, detection_topic, 10
        )

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos
        )

        # Rate-limiting state
        self._last_publish_time = self.get_clock().now()
        self._min_period_ns = int(1e9 / self.rate_limit) if self.rate_limit > 0 else 0

        self.get_logger().info(
            f'YoloDetectorNode ready — subscribing to {image_topic}, '
            f'publishing to {detection_topic}'
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def image_callback(self, msg: Image):

        now = self.get_clock().now()
        if (now - self._last_publish_time).nanoseconds < self._min_period_ns:
            return
        self._last_publish_time = now

        # Convert ROS Image -> OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        if len(results) == 0 or results[0].boxes is None:
            self.detection_pub.publish(det_array)
            return

        boxes = results[0].boxes
        for box in boxes:
            cls_id = int(box.cls[0].item())

            # Skip categories we don't care about
            if cls_id not in self.target_indices:
                continue

            confidence = float(box.conf[0].item())

            # xyxy bounding box (pixels)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            det = Detection2D()
            det.header = msg.header

            # Center + size (Detection2D convention)
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.class_names[cls_id]
            hyp.hypothesis.score = confidence
            det.results.append(hyp)

            det_array.detections.append(det)

        self.detection_pub.publish(det_array)

        if det_array.detections:
            summary = ', '.join(
                f'{d.results[0].hypothesis.class_id} '
                f'({d.results[0].hypothesis.score:.2f})'
                for d in det_array.detections
            )
            self.get_logger().debug(f'Published {len(det_array.detections)} detections: {summary}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
