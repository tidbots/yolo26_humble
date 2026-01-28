
from setuptools import setup

package_name = "yolo26_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo26.launch.py"]),
        ("share/" + package_name + "/config", ["config/yolo26.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tidbots",
    maintainer_email="you@example.com",
    description="YOLO26 object detector node for ROS 2 Humble (rclpy).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo26_node = yolo26_ros2.yolo26_node:main",
        ],
    },
)
