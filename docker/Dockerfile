FROM osrf/ros@sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0 AS build_environment

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates \
    python3-colcon-common-extensions \
    libeigen3-dev libyaml-cpp-dev libpcl-dev libtbb-dev liblz4-dev \
    clang-format \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN git clone https://github.com/url-kaist/patchwork-plusplus.git /ws/third_party/patchworkpp && \
    git -C /ws/third_party/patchworkpp checkout --detach 3e6903a1d5537a4cc2ace897b0bbb98a92d6014c

FROM build_environment AS development
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pytest python3-numpy \
    ros-humble-ament-cmake-pytest \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint-auto \
    ros-humble-ament-cmake-flake8 \
    ros-humble-ament-cmake-lint-cmake \
    ros-humble-ament-cmake-pep257 \
    ros-humble-ament-cmake-xmllint \
    && rm -rf /var/lib/apt/lists/*

FROM build_environment AS builder
COPY argus_core   /ws/argus_core
COPY argus_msgs   /ws/argus_msgs
COPY argus_node_preprocess   /ws/argus_node_preprocess
COPY argus_node_odometry     /ws/argus_node_odometry
COPY argus_node_tunnel_model /ws/argus_node_tunnel_model
COPY argus_node_detector     /ws/argus_node_detector
COPY argus_node_fusion       /ws/argus_node_fusion
COPY argus_node_watchdog     /ws/argus_node_watchdog
COPY argus_node_recorder     /ws/argus_node_recorder
COPY argus_launch /ws/argus_launch
COPY argus_eval   /ws/argus_eval
COPY argus_web    /ws/argus_web

RUN . /opt/ros/humble/setup.sh && \
    MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 \
    colcon build --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM osrf/ros@sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-numpy \
    tar \
    zstd \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-compression \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-interfaces \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-transport \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir zstandard

COPY --from=builder /ws/install /ws/install
COPY argus_web /ws/argus_web
COPY argus_launch/config /ws/argus_launch/config
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/detect_watch.py /opt/argus/detect_watch.py
COPY docker/bag_topic.py /opt/argus/bag_topic.py
COPY docker/bag_play.py /opt/argus/bag_play.py
COPY docker/fastdds.xml /opt/argus/fastdds.xml
RUN chmod +x /entrypoint.sh /opt/argus/detect_watch.py /opt/argus/bag_topic.py /opt/argus/bag_play.py

ENV WS=/ws \
    ARGUS_ROOT=/ws \
    PYTHONPATH=/ws/argus_web \
    PYTHONUNBUFFERED=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EXPOSE 8080
VOLUME ["/data"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["help"]
