# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Основной launch-файл Argus (PLAN.md 10.4).
#
# Примеры:
#   ros2 launch argus_launch argus.launch.py \
#       bag:=/ws/data/recordings/roundT_doubleT
#   ros2 launch argus_launch argus.launch.py rviz:=true rate:=0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('argus_launch')
    default_params = os.path.join(pkg_share, 'config', 'argus_params.yaml')

    bag = LaunchConfiguration('bag')
    lidar_topic = LaunchConfiguration('lidar_topic')
    rate = LaunchConfiguration('rate')
    loop = LaunchConfiguration('loop')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    arguments = [
        DeclareLaunchArgument(
            'bag', default_value='',
            description='Путь к bag-директории. Пусто = слушать живой лидар'),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/lidar_points',
            description='Топик PointCloud2'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Скорость воспроизведения bag. 0 = без задержек'),
        DeclareLaunchArgument(
            'loop', default_value='false', description='Зациклить bag'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML со всеми параметрами конвейера'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Запустить RViz2 с готовым layout'),
    ]

    nodes = [
        Node(
            package='argus_node_preprocess', executable='preprocess_node',
            name='argus_preprocess', output='screen',
            parameters=[params_file, {'cloud.input_topic': lidar_topic}],
        ),
        Node(
            package='argus_node_odometry', executable='argus_odometry_node',
            name='argus_odometry', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='argus_node_tunnel_model',
            executable='argus_tunnel_model_node',
            name='argus_tunnel_model', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='argus_node_detector', executable='argus_detector_node',
            name='argus_detector', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='argus_node_fusion', executable='argus_fusion_node',
            name='argus_fusion', output='screen',
            parameters=[params_file],
        ),
    ]

    # bag-play подключается только если bag задан; loop выбирает ветку с --loop.
    bag_with_loop = PythonExpression(
        ["'", bag, "' != '' and '", loop, "'.lower() == 'true'"])
    bag_no_loop = PythonExpression(
        ["'", bag, "' != '' and '", loop, "'.lower() == 'false'"])
    processes = [
        ExecuteProcess(
            condition=IfCondition(bag_no_loop),
            cmd=['ros2', 'bag', 'play', bag, '--rate', rate],
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(bag_with_loop),
            cmd=['ros2', 'bag', 'play', bag, '--rate', rate, '--loop'],
            output='screen',
        ),
    ]

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'argus.rviz')],
        output='screen',
    )

    return LaunchDescription(arguments + nodes + processes + [rviz_node])
