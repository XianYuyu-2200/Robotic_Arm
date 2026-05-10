from pathlib import Path
import os
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetectionNode(Node):
    """Loads a YOLO .pt model and publishes ROS vision_msgs detections."""

    def __init__(self):
        super().__init__("yolo_detection_node")

        self.declare_parameter("model_path", "weights/best.pt")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detector/detections")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("device", "")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("publish_numeric_class_id", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/detector/debug_image")

        self.model_path = self._resolve_model_path(self.get_parameter("model_path").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.device = self.get_parameter("device").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.publish_numeric_class_id = bool(self.get_parameter("publish_numeric_class_id").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.bridge = CvBridge()
        self.inference_lock = threading.Lock()
        self.model = self._load_model(self.model_path)

        self.publisher = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.debug_image_publisher = None
        if self.publish_debug_image:
            self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"YOLO detector ready: model='{self.model_path}'")
        self.get_logger().info(
            f"Subscribing image_topic='{self.image_topic}', publishing detections_topic='{self.detections_topic}'"
        )
        if self.publish_debug_image:
            self.get_logger().info(f"Publishing debug image_topic='{self.debug_image_topic}'")
        self.get_logger().info(f"Model classes: {self.model.names}")

    def _resolve_model_path(self, model_path):
        env_model = os.environ.get("GLUON_MODEL_PATH", "")
        candidates = []

        if env_model:
            candidates.append(Path(env_model))

        requested = Path(str(model_path)).expanduser()
        candidates.append(requested)
        if not requested.is_absolute():
            candidates.append(Path.cwd() / requested)

        for parent in [Path.cwd(), *Path.cwd().parents]:
            candidates.append(parent / "weights" / "best.pt")

        source_path = Path(__file__).resolve()
        for parent in source_path.parents:
            candidates.append(parent / "weights" / "best.pt")

        for candidate in candidates:
            if candidate.exists():
                return str(candidate.resolve())

        raise FileNotFoundError(
            "Could not find YOLO model. Set model_path or GLUON_MODEL_PATH, for example "
            "/home/nuc/gluon/Robotic_Arm-main/weights/best.pt"
        )

    def _load_model(self, model_path):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "Python package 'ultralytics' is required for gluon_vision. "
                "Install it on the robot with: pip3 install ultralytics"
            ) from exc

        return YOLO(model_path)

    def image_callback(self, msg):
        if not self.inference_lock.acquire(blocking=False):
            self.get_logger().debug("Skipping image because previous YOLO inference is still running")
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            predict_kwargs = {
                "conf": self.conf_threshold,
                "imgsz": self.imgsz,
                "verbose": False,
            }
            if self.device:
                predict_kwargs["device"] = self.device

            results = self.model(image, **predict_kwargs)
            if not results:
                return

            detections_msg = self._results_to_detections(msg, results[0])
            self.publisher.publish(detections_msg)
            self._publish_debug_image(msg, image, detections_msg)
        except Exception as exc:
            self.get_logger().error(f"YOLO detection failed: {exc}")
        finally:
            self.inference_lock.release()

    def _results_to_detections(self, image_msg, result):
        detections_msg = Detection2DArray()
        detections_msg.header = image_msg.header

        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return detections_msg

        for box in boxes:
            score = float(box.conf[0])
            if score < self.conf_threshold:
                continue

            class_index = int(box.cls[0])
            class_label = self._class_label(class_index)
            class_id = str(class_index) if self.publish_numeric_class_id else class_label
            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]

            detection = Detection2D()
            detection.header = image_msg.header
            detection.bbox.center.position.x = (x1 + x2) * 0.5
            detection.bbox.center.position.y = (y1 + y2) * 0.5
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = max(0.0, x2 - x1)
            detection.bbox.size_y = max(0.0, y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_id
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        if detections_msg.detections:
            self.get_logger().debug(f"Published {len(detections_msg.detections)} detections")

        return detections_msg

    def _publish_debug_image(self, image_msg, image, detections_msg):
        if not self.debug_image_publisher:
            return

        debug_image = image.copy()
        for detection in detections_msg.detections:
            cx = float(detection.bbox.center.position.x)
            cy = float(detection.bbox.center.position.y)
            width = float(detection.bbox.size_x)
            height = float(detection.bbox.size_y)
            x1 = int(round(cx - width * 0.5))
            y1 = int(round(cy - height * 0.5))
            x2 = int(round(cx + width * 0.5))
            y2 = int(round(cy + height * 0.5))

            label = "target"
            score = 0.0
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                score = float(detection.results[0].hypothesis.score)

            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{label} {score:.2f}"
            text_origin = (max(0, x1), max(20, y1 - 8))
            cv2.putText(
                debug_image,
                text,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_image_publisher.publish(debug_msg)

    def _class_label(self, class_index):
        names = getattr(self.model, "names", {})
        if isinstance(names, dict):
            return str(names.get(class_index, class_index))
        if 0 <= class_index < len(names):
            return str(names[class_index])
        return str(class_index)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
