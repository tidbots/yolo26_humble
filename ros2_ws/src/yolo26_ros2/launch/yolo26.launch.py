from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # YOLO node arguments
            DeclareLaunchArgument("model", default_value="", description="Path to YOLO26 weights (.pt)"),
            DeclareLaunchArgument("image", default_value="/camera/image_raw", description="Input image topic"),
            DeclareLaunchArgument("detections", default_value="/yolo26/detections", description="Output detections topic"),
            DeclareLaunchArgument("debug_image", default_value="/yolo26/debug_image", description="Output debug image topic"),
            DeclareLaunchArgument("classes_yaml", default_value="", description="Class names YAML path"),
            DeclareLaunchArgument("device", default_value="0", description="Device: 0, cpu, etc"),
            DeclareLaunchArgument("conf", default_value="0.25", description="Confidence threshold"),
            DeclareLaunchArgument("iou", default_value="0.45", description="IoU threshold"),
            DeclareLaunchArgument("rate", default_value="15.0", description="Processing rate Hz"),
            DeclareLaunchArgument("transport", default_value="raw", description="raw or compressed"),
            DeclareLaunchArgument("publish_debug", default_value="true", description="Publish debug image"),
            DeclareLaunchArgument("cv_debug_window", default_value="true", description="Show OpenCV debug window"),
            # Tracking arguments
            DeclareLaunchArgument("tracking", default_value="false", description="Enable ByteTrack tracking"),
            DeclareLaunchArgument("tracker", default_value="bytetrack.yaml", description="Tracker config: bytetrack.yaml or botsort.yaml"),
            DeclareLaunchArgument("smoothing", default_value="15", description="Smoothing window frames for moving average"),
            DeclareLaunchArgument("appear_frames", default_value="3", description="Frames to confirm appearance"),
            DeclareLaunchArgument("disappear_frames", default_value="5", description="Frames to confirm disappearance"),
            # Camera node arguments
            DeclareLaunchArgument("use_camera", default_value="false", description="Launch v4l2_camera node"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0", description="Video device path"),
            DeclareLaunchArgument("pixel_format", default_value="YUYV", description="Camera pixel format"),
            # YOLO node
            Node(
                package="yolo26_ros2",
                executable="yolo26_node",
                name="yolo26_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model"),
                        "image_topic": LaunchConfiguration("image"),
                        "detections_topic": LaunchConfiguration("detections"),
                        "debug_image_topic": LaunchConfiguration("debug_image"),
                        "classes_yaml": LaunchConfiguration("classes_yaml"),
                        "device": LaunchConfiguration("device"),
                        "conf_thres": LaunchConfiguration("conf"),
                        "iou_thres": LaunchConfiguration("iou"),
                        "process_rate_hz": LaunchConfiguration("rate"),
                        "image_transport": LaunchConfiguration("transport"),
                        "publish_debug_image": LaunchConfiguration("publish_debug"),
                        "cv_debug_window": LaunchConfiguration("cv_debug_window"),
                        "enable_tracking": LaunchConfiguration("tracking"),
                        "tracker": LaunchConfiguration("tracker"),
                        "smoothing_window": LaunchConfiguration("smoothing"),
                        "appear_frames": LaunchConfiguration("appear_frames"),
                        "disappear_frames": LaunchConfiguration("disappear_frames"),
                    }
                ],
            ),
            # v4l2_camera node (optional)
            Node(
                condition=IfCondition(LaunchConfiguration("use_camera")),
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera_node",
                output="screen",
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "image_size": [640, 480],
                        "pixel_format": LaunchConfiguration("pixel_format"),
                    }
                ],
                remappings=[
                    ("image_raw", LaunchConfiguration("image")),
                ],
            ),
        ]
    )
