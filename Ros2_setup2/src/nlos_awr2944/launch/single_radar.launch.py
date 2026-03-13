"""
단일 AWR2944 레이더 노드 실행
Usage: ros2 launch nlos_awr2944 single_radar.launch.py

옵션:
  use_rviz:=true/false    - RViz 실행 여부
  record_bag:=true/false  - Rosbag 녹화 여부
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nlos_awr2944')
    
    rviz_config = os.path.join(pkg_share, 'rviz', 'radar.rviz')
    param_file = os.path.join(pkg_share, 'config', 'single_radar.yaml')
    
    # Rosbag 저장 경로 (패키지 src 폴더 내 rosbag 디렉토리)
    pkg_src_dir = os.path.expanduser('~/Nlos/Ros2_setup/src/nlos_awr2944')
    rosbag_base_dir = os.path.join(pkg_src_dir, 'rosbag')
    
    # rosbag 폴더 없으면 생성
    os.makedirs(rosbag_base_dir, exist_ok=True)
    
    # 타임스탬프로 폴더명 생성
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    rosbag_path = os.path.join(rosbag_base_dir, timestamp)
    
    # Launch Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', 
        default_value='true',
        description='Launch RViz2'
    )
    
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', 
        default_value='false',
        description='Record rosbag'
    )
    
    # 레이더 노드 (순수하게 토픽 발행만)
    radar_node = Node(
        package='nlos_awr2944',
        executable='radar_node',
        name='radar_node',
        output='screen',
        parameters=[param_file]
    )
    
    # Static TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_link']
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Rosbag 녹화 (2초 후 시작, 타임스탬프 폴더에 저장)
    rosbag_record = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '/radar/points',
                    '/radar/timestamp', 
                    '/radar/range_profile',
                    '/radar/noise_profile',
                    '/radar/heatmap/azimuth',
                    '/tf',
                    '/tf_static',
                    '-o', rosbag_path
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('record_bag'))
            )
        ]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        record_bag_arg,
        radar_node,
        static_tf_node,
        rviz_node,
        rosbag_record,
    ])