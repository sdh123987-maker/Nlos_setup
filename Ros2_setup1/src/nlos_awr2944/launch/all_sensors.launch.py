"""
전체 센서 동기화 실행
- 레이더 4개 (AWR2944)
- 라이다
- 카메라
- 마이크 (16채널)

Usage: ros2 launch nlos_awr2944 all_sensors.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('nlos_awr2944')
    
    # 설정 파일 경로
    multi_radar_config = os.path.join(pkg_share, 'config', 'multi_radar.yaml')
    sensor_sync_config = os.path.join(pkg_share, 'config', 'sensor_sync.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'all_sensors.rviz')
    
    # =========================================
    # Launch Arguments
    # =========================================
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    enable_radar_arg = DeclareLaunchArgument(
        'enable_radar',
        default_value='true',
        description='Enable radar sensors'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable lidar sensor'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera sensor'
    )
    
    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio',
        default_value='true',
        description='Enable microphone array (16ch)'
    )
    
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Record rosbag'
    )
    
    # =========================================
    # Static TF (센서 프레임 정의)
    # =========================================
    
    # map -> base_link
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )
    
    # base_link -> lidar_link
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    # base_link -> camera_link
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.3', '0', '0.4', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # base_link -> mic_array_link
    mic_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mic_array_tf',
        arguments=['0', '0', '0.6', '0', '0', '0', 'base_link', 'mic_array_link']
    )
    
    # =========================================
    # 센서 노드들
    # =========================================
    
    # 멀티 레이더 매니저 (4개 레이더 동기화)
    multi_radar_node = Node(
        package='nlos_awr2944',
        executable='multi_radar_manager',
        name='multi_radar_manager',
        output='screen',
        parameters=[multi_radar_config],
        condition=IfCondition(LaunchConfiguration('enable_radar'))
    )
    
    # 센서 동기화 노드
    sensor_sync_node = Node(
        package='nlos_awr2944',
        executable='sensor_sync_node',
        name='sensor_sync_node',
        output='screen',
        parameters=[
            sensor_sync_config,
            {
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'enable_camera': LaunchConfiguration('enable_camera'),
                'enable_audio': LaunchConfiguration('enable_audio'),
            }
        ]
    )
    
    # =========================================
    # 라이다 드라이버 (예: Velodyne, Ouster 등)
    # 실제 사용 시 해당 드라이버 패키지로 교체 필요
    # =========================================
    # lidar_node = Node(
    #     package='velodyne_driver',
    #     executable='velodyne_driver_node',
    #     name='lidar_driver',
    #     parameters=[...],
    #     condition=IfCondition(LaunchConfiguration('enable_lidar'))
    # )
    
    # =========================================
    # 카메라 드라이버 (예: usb_cam, realsense 등)
    # 실제 사용 시 해당 드라이버 패키지로 교체 필요
    # =========================================
    # camera_node = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='camera_driver',
    #     parameters=[...],
    #     condition=IfCondition(LaunchConfiguration('enable_camera'))
    # )
    
    # =========================================
    # 마이크 드라이버 (16채널)
    # 실제 사용 시 오디오 드라이버 패키지로 교체 필요
    # =========================================
    # audio_node = Node(
    #     package='audio_capture',
    #     executable='audio_capture_node',
    #     name='microphone_array',
    #     parameters=[{
    #         'channels': 16,
    #         'sample_rate': 48000,
    #     }],
    #     condition=IfCondition(LaunchConfiguration('enable_audio'))
    # )
    
    # =========================================
    # RViz2
    # =========================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # =========================================
    # ROS Bag 녹화 (선택사항)
    # =========================================
    # rosbag_node = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a', '-o', 'sensor_data'],
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('record_bag'))
    # )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        enable_radar_arg,
        enable_lidar_arg,
        enable_camera_arg,
        enable_audio_arg,
        record_bag_arg,
        
        # TF
        base_tf,
        lidar_tf,
        camera_tf,
        mic_tf,
        
        # Nodes
        multi_radar_node,
        sensor_sync_node,
        rviz_node,
        
        # 다른 센서 드라이버들은 주석 해제 후 사용
        # lidar_node,
        # camera_node,
        # audio_node,
        # rosbag_node,
    ])
