import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('argus_launch')
    default_params = os.path.join(pkg_share, 'config', 'argus_params.yaml')

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'bag', default_value='',
            description='Путь к bag-директории для воспроизведения'),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/lidar_points',
            description='Топик PointCloud2'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Скорость воспроизведения bag. 0 = без задержек'),
        DeclareLaunchArgument(
            'publish_markers', default_value='true',
            description='Публиковать маркеры габарита и кластеров для RViz2'),
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
             parameters=[LaunchConfiguration('params_file'),
                         {'cloud.input_topic': LaunchConfiguration('lidar_topic')}]),
        Node(package='argus_node_odometry', executable='argus_odometry_node',
             name='argus_odometry', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='argus_node_tunnel_model',
             executable='argus_tunnel_model_node',
             name='argus_tunnel_model', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'tunnel_model.publish_points': LaunchConfiguration('publish_markers')}]),
        Node(package='argus_node_detector', executable='argus_detector_node',
             name='argus_detector', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='argus_node_fusion', executable='argus_fusion_node',
             name='argus_fusion', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'fusion.publish_markers': LaunchConfiguration('publish_markers')}]),
    ]

    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
             '--rate', LaunchConfiguration('rate')],
        output='screen',
    )

    return LaunchDescription(arguments + nodes + [rviz, play])
