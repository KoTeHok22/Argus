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
# Демо-запуск: конвейер + RViz2. Bag воспроизводится отдельно командой:
#   ros2 bag play <bag> --loop

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('argus_launch')
    default_params = os.path.join(pkg_share, 'config', 'argus_params.yaml')

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2',
        condition=None,
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'argus.rviz')],
        output='screen',
    )

    nodes = [
        Node(package='argus_node_preprocess', executable='preprocess_node',
             name='argus_preprocess', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='argus_node_odometry', executable='argus_odometry_node',
             name='argus_odometry', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='argus_node_tunnel_model',
             executable='argus_tunnel_model_node',
             name='argus_tunnel_model', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='argus_node_detector', executable='argus_detector_node',
             name='argus_detector', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='argus_node_fusion', executable='argus_fusion_node',
             name='argus_fusion', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ]

    return LaunchDescription(arguments + nodes + [rviz])
