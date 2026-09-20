FROM osrf/ros@sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0 AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates \
    python3-colcon-common-extensions \
    libeigen3-dev libyaml-cpp-dev libpcl-dev libtbb-dev liblz4-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY third_party/patchworkpp /ws/third_party/patchworkpp
COPY argus_core   /ws/argus_core
COPY argus_msgs   /ws/argus_msgs
COPY argus_node_preprocess   /ws/argus_node_preprocess
COPY argus_node_odometry     /ws/argus_node_odometry
COPY argus_node_tunnel_model /ws/argus_node_tunnel_model
COPY argus_node_detector     /ws/argus_node_detector
COPY argus_node_fusion       /ws/argus_node_fusion
COPY argus_launch /ws/argus_launch
COPY argus_eval   /ws/argus_eval

RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM osrf/ros@sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    ros-humble-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/detect_watch.py /opt/argus/detect_watch.py
COPY docker/bag_topic.py /opt/argus/bag_topic.py
COPY docker/fastdds.xml /opt/argus/fastdds.xml
RUN chmod +x /entrypoint.sh /opt/argus/detect_watch.py /opt/argus/bag_topic.py

ENV WS=/ws \
    PYTHONUNBUFFERED=1 \
    ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/argus/fastdds.xml
VOLUME ["/data"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["help"]
