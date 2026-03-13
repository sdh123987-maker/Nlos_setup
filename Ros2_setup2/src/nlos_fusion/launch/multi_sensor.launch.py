"""
==============================================
NLOS Multi-Sensor Launch v6
==============================================
변경점 (v5 → v6):
  - 실시간 동기화 노드(multi_sensor_sync) 완전 제거
  - 모든 센서의 raw 토픽을 그대로 rosbag에 저장
  - 후처리(process_rosbag.py)에서 10Hz nearest-neighbor 동기화 수행
  - audio/raw 토픽 추가 녹화 (원본 음원 데이터 보존)
  - CPU 부하 감소 + 동기화 정밀도 향상
"""

import os
import yaml
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction, LogInfo, RegisterEventHandler, EmitEvent)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def count_enabled_radars(yaml_path):
    """multi_radar.yaml에서 enabled: true인 레이더 ID 목록 반환"""
    try:
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        params = config.get('/multi_radar_manager', {}).get('ros__parameters', {})
        
        enabled_ids = []
        for i in range(4):
            radar_key = f'radar_{i}'
            radar_cfg = params.get(radar_key, {})
            if isinstance(radar_cfg, dict) and radar_cfg.get('enabled', False):
                enabled_ids.append(i)
        
        return enabled_ids
    except Exception as e:
        print(f"⚠️ YAML 파싱 실패: {e}, 기본값 [0, 1] 사용")
        return [0, 1]


def generate_launch_description():
    pkg_fusion = get_package_share_directory('nlos_fusion')
    pkg_radar = get_package_share_directory('nlos_awr2944')
    pkg_camera = get_package_share_directory('realsense2_camera')
    
    # ============================================================
    # YAML에서 활성 레이더 자동 감지
    # ============================================================
    yaml_path = os.path.join(pkg_radar, 'config', 'multi_radar.yaml')
    enabled_radar_ids = count_enabled_radars(yaml_path)
    num_radars = len(enabled_radar_ids)
    
    # Rosbag 경로 (raw_ 접두사)
    workspace_dir = os.path.expanduser('~/Ros2_setup')
    rosbag_dir = os.path.join(workspace_dir, 'rosbag')
    os.makedirs(rosbag_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    rosbag_path = os.path.join(rosbag_dir, f'raw_{timestamp}')

    args = [
        DeclareLaunchArgument('enable_lidar', default_value='true', description='Enable Ouster Lidar'),
        DeclareLaunchArgument('record_bag', default_value='true', description='Record Raw Topics'),
    ]

    # ================================================================
    # 1. 카메라 (RealSense)
    # ================================================================
    pkg_camera = get_package_share_directory('realsense2_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_camera, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'rgb_camera.profile': '640,480,30',
            'pointcloud.enable': 'false'
        }.items()
    )

    # ================================================================
    # 2. 멀티 레이더
    # ================================================================
    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_radar, 'launch', 'multi_radar.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # ================================================================
    # 3. 오디오
    # ================================================================
    audio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_fusion, 'launch', 'audio.launch.py'))
    )

    # ================================================================
    # 4. 라이다 (Ouster) - LifecycleNode 방식
    # ================================================================
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_lidar_params = str(Path(ouster_ros_pkg_dir) / 'config' / 'ouster_config.yaml')

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace='ouster',
        parameters=[default_lidar_params],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='inactive',
            entities=[
                LogInfo(msg="🔄 Ouster os_driver activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    sensor_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='finalized',
            entities=[
                LogInfo(msg="⚠️ Ouster: Failed to communicate with sensor."),
            ],
        )
    )

    # ================================================================
    # 5. Rosbag 녹화 - Raw 토픽 직접 저장 (동기화 노드 없음!)
    # ================================================================
    topics_to_record = [
        # ========== 카메라 ==========
        '/camera/camera/color/image_raw',
        # ========== 오디오 ==========
        '/audio/raw',           # [신규] 원본 음원 데이터 (WAV 복원용)
        '/audio/spectrum',
        '/audio/direction',
        '/audio/header',        # 오디오 타임스탬프
        # ========== 레이더 (통합) ==========
        '/radar/merged/points',
        # ========== 라이다 ==========
        '/ouster/points',
        '/ouster/imu',
        '/ouster/range_image',
        # ========== TF ==========
        '/tf', '/tf_static',
    ]
    # 개별 레이더 토픽 (multi_radar_manager가 발행)
    for rid in enabled_radar_ids:
        prefix = f'/radar_{rid}'
        topics_to_record += [
            f'{prefix}/points',
            f'{prefix}/heatmap/azimuth',
            f'{prefix}/range_profile',
            f'{prefix}/noise_profile',
            f'{prefix}/timestamp',
        ]
    
    # QoS 오버라이드 YAML 생성
    qos_override_path = os.path.join(rosbag_dir, f'qos_override_{timestamp}.yaml')
    qos_overrides = {}
    for topic in topics_to_record:
        if topic in ['/tf', '/tf_static']:
            continue
        qos_overrides[topic] = {
            'reliability': 'best_effort',
            'durability': 'volatile',
            'history': 'keep_last',
            'depth': 30,  # 데이터 유실(Drop)을 막기 위해 대기실 크기는 넉넉하게 유지
        }
    
    import yaml as yaml_lib
    with open(qos_override_path, 'w') as f:
        yaml_lib.dump(qos_overrides, f, default_flow_style=False)
    
    rosbag_cmd = [
        'ros2', 'bag', 'record',
        '-o', rosbag_path,
        '--qos-profile-overrides-path', qos_override_path,
        '--max-cache-size', '200000000',
    ] + topics_to_record
    rosbag_process = ExecuteProcess(
        cmd=rosbag_cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    # ================================================================
    # 6. RViz
    # ================================================================
    rviz_config = os.path.join(pkg_fusion, 'rviz', 'multi_sensor.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ================================================================
    # 실행 순서 (USB 대역폭 고려)
    #   0s: 카메라
    #   2s: 레이더
    #   3s: 오디오
    #   4s: 라이다 (Ouster LifecycleNode + 상태전환)
    #   6s: Rosbag 녹화 (센서 안정화 후)
    #
    # ※ 동기화 노드 없음 - 후처리에서 10Hz 동기화 수행
    # ================================================================
    radar_ids_str = ', '.join([f'radar_{i}' for i in enabled_radar_ids])
    return LaunchDescription(args + [
        LogInfo(msg=f"🚀 Multi-Sensor Launch v6 (Raw Recording - No Sync Node)"),
        LogInfo(msg=f"📡 Active radars ({num_radars}): {radar_ids_str}"),
        LogInfo(msg=f"📁 Rosbag path: {rosbag_path}"),
        LogInfo(msg=f"ℹ️  동기화는 후처리(process_rosbag.py)에서 10Hz로 수행됩니다."),
        
        # 센서 시작
        camera_launch,
        TimerAction(period=2.0, actions=[radar_launch]),
        TimerAction(period=3.0, actions=[audio_launch]),
        
        # 라이다: LifecycleNode + 상태 전환 이벤트
        TimerAction(period=4.0, actions=[
            LogInfo(msg="🔵 Starting Ouster Lidar..."),
            os_driver,
            sensor_configure_event,
            sensor_activate_event,
            sensor_finalized_event,
        ]),
        
        # Rosbag 녹화 (센서 안정화 후)
        TimerAction(period=6.0, actions=[rosbag_process]),
        
        rviz_node,
    ])
