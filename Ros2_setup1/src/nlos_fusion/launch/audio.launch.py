import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 설정 파일 경로 (나중에 XML 파일 읽을 때 필요)
    config = os.path.join(
        get_package_share_directory('nlos_fusion'),
        'config',
        'mic_geometry.xml'
    )

    return LaunchDescription([
        # [수정됨] executable='audio_publisher' -> 'audio_node'
        Node(
            package='nlos_fusion',
            executable='audio_node.py',  # setup.py에 등록된 이름과 일치해야 함
            name='audio_driver',
            output='screen'
        ),
        Node(
            package='nlos_fusion',
            executable='sound_localizer.py',
            name='sound_localizer',
            output='screen',
            parameters=[{'mic_config_file': config}]
        )
    ])
