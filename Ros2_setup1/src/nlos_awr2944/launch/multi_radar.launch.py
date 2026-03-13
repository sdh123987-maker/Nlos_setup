import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nlos_awr2944')
    
    # 1. 설정 파일 경로 (User/Ros2_setup/.../config/radar_config.json)
    # 실제로는 패키지 share 디렉토리로 복사된 것을 사용하거나 직접 경로 지정 가능
    default_json_config = os.path.join(pkg_share, 'config', 'radar_config.json')
    
    # 2. 멀티 레이더 YAML 파일
    multi_radar_params = os.path.join(pkg_share, 'config', 'multi_radar.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'multi_radar.rviz') # 기존 rviz 재사용

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        
        # 멀티 레이더 매니저 노드
        Node(
            package='nlos_awr2944',
            executable='multi_radar_manager',
            name='multi_radar_manager',
            output='screen',
            parameters=[
                multi_radar_params,           # 4개 포트 및 위치 정보
                {'config_file': default_json_config}, # JSON 명령어 파일 경로
                {'target_freq_hz': 10.0},     # 10Hz 동기화 목표
                {'sync_tolerance_ms': 25.0}   # 20Hz(50ms) 주기이므로 허용오차 25ms
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
        
        # 기본 TF (Map -> Base Link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','map','base_link']
        )
    ])