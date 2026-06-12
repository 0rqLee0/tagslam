# TagSLAM marker-baseline runtime for the Table dataset.
# All ROS/GTSAM/apriltag deps live in this image; the host only needs Docker.
# tagslam itself is compiled at run time into a mounted volume (docker/build_ws.sh),
# so editing the Python bridge / yaml configs needs no image rebuild.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# tooling + OpenCV (cv2 with findChessboardCornersSB) + evo for ATE
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-rosdep python3-colcon-common-extensions \
      python3-opencv python3-numpy python3-pip git \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir evo

# resolve tagslam's package.xml deps (gtsam, apriltag_msgs, apriltag_detector,
# flex_sync, cv_bridge, image_transport, rosbag2_cpp, yaml-cpp, ...) into the image
COPY package.xml /tmp/tagslam/package.xml
RUN apt-get update && \
    ([ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init) && \
    rosdep update && \
    rosdep install --from-paths /tmp/tagslam --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# evo pulls numpy 2.x, which breaks ROS cv_bridge/cv2 (built against numpy 1.x)
RUN pip3 install --no-cache-dir "numpy<2"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /opt/tagslam_ws/install/setup.bash ] && source /opt/tagslam_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /home/lee/tagslam
CMD ["bash"]
