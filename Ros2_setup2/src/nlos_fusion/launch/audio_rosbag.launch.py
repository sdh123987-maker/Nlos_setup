import os
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 기존 오디오 런치 파일 경로
    audio_launch_path = os.path.join(
        get_package_share_directory('nlos_fusion'),
        'launch', 
        'audio.launch.py' 
    )

    # 2. 저장할 Rosbag 파일 이름 (현재 시간)
    # 예: bag_2026_01_27-17_30_00
    timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_name = f"bag_{timestamp}"

    # 3. Rosbag 기록 프로세스
    # CSV 변환에 필요한 '/audio/spectrum'과 디버깅용 '/audio/direction'을 녹화합니다.
    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_name,
            '/audio/spectrum',   # [필수] 각도별 강도 데이터
            '/audio/direction'   # [참고] 계산된 최고 각도
        ],
        output='screen'
    )

    return LaunchDescription([
        # 오디오 시스템 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(audio_launch_path)
        ),
        # 녹화 시작
        record_process
    ])