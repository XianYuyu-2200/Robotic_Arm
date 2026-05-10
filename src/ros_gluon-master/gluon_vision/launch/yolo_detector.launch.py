from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    conf_threshold = LaunchConfiguration("conf_threshold")
    device = LaunchConfiguration("device")
    imgsz = LaunchConfiguration("imgsz")
    publish_numeric_class_id = LaunchConfiguration("publish_numeric_class_id")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    debug_image_topic = LaunchConfiguration("debug_image_topic")

    detector_node = Node(
        package="gluon_vision",
        executable="yolo_detection_node",
        name="yolo_detection_node",
        output="screen",
        parameters=[
            {
                "model_path": ParameterValue(model_path, value_type=str),
                "image_topic": ParameterValue(image_topic, value_type=str),
                "detections_topic": ParameterValue(detections_topic, value_type=str),
                "conf_threshold": ParameterValue(conf_threshold, value_type=float),
                "device": ParameterValue(device, value_type=str),
                "imgsz": ParameterValue(imgsz, value_type=int),
                "publish_numeric_class_id": ParameterValue(publish_numeric_class_id, value_type=bool),
                "publish_debug_image": ParameterValue(publish_debug_image, value_type=bool),
                "debug_image_topic": ParameterValue(debug_image_topic, value_type=str),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value="weights/best.pt"),
            DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("detections_topic", default_value="/detector/detections"),
            DeclareLaunchArgument("conf_threshold", default_value="0.5"),
            DeclareLaunchArgument("device", default_value="0"),
            DeclareLaunchArgument("imgsz", default_value="320"),
            DeclareLaunchArgument("publish_numeric_class_id", default_value="false"),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument("debug_image_topic", default_value="/detector/debug_image"),
            detector_node,
        ]
    )
