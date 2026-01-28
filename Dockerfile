FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-lc"]

# --- OS deps ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-yaml \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-v4l2-camera \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# --- Python deps ---
# YOLO26 を Ultralytics 系で動かす想定（既存スクリプトに合わせて必要ならここを調整）
RUN pip3 install --no-cache-dir ultralytics

# --- Workspace ---
WORKDIR /ros2_ws

# ソースをコンテナへ
COPY ros2_ws/src /ros2_ws/src

# ビルド
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

