# ---------- Этап 1: сборка ----------
FROM osrf/ros:humble-perception-jammy AS builder

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-vcstool \
    libeigen3-dev libyaml-cpp-dev libpcl-dev \
    libtbb-dev liblz4-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY third_party/argus.repos third_party/
RUN mkdir -p src && vcs import src < third_party/argus.repos

COPY argus_core   src/argus_core
COPY argus_msgs   src/argus_msgs
COPY argus_node_preprocess   src/argus_node_preprocess
COPY argus_node_odometry     src/argus_node_odometry
COPY argus_node_tunnel_model src/argus_node_tunnel_model
COPY argus_node_detector     src/argus_node_detector
COPY argus_node_fusion       src/argus_node_fusion
COPY argus_launch src/argus_launch
COPY argus_eval   src/argus_eval

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------- Этап 2: рантайм ----------
FROM osrf/ros:humble-perception-jammy

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev libyaml-cpp-dev libpcl-dev libtbb-dev liblz4-dev \
    python3-numpy python3-scipy python3-matplotlib python3-pandas \
    ros-humble-rviz2 ros-humble-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV WS=/ws
# Данные монтируются, а НЕ встраиваются в образ (конфиденциальность ТЗ)
VOLUME ["/data"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["help"]
