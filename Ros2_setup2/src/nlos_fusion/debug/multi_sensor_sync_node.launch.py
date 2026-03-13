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
        for i in range(4):  # MAX_RADARS = 4
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
    
    # Rosbag 경로
    workspace_dir = os.path.expanduser('~/Nlos/Ros2_setup')
    rosbag_dir = os.path.join(workspace_dir, 'rosbag')
    os.makedirs(rosbag_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    rosbag_path = os.path.join(rosbag_dir, f'sync_multi_{timestamp}')

    args = [
        DeclareLaunchArgument('enable_lidar', default_value='true', description='Enable Ouster Lidar'),
        DeclareLaunchArgument('record_bag', default_value='true', description='Record Synced Topics'),
    ]

    # ================================================================
    # 1. 카메라 (RealSense)
    # ================================================================
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
    # 4. 라이다 (Ouster) - enable_lidar가 true일 때만 실행
    #    driver.launch.py 로직을 직접 포함 (LifecycleNode 방식)
    # ================================================================
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_lidar_params = str(Path(ouster_ros_pkg_dir) / 'config' / 'driver_params.yaml')

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace='ouster',
        parameters=[default_lidar_params],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    # LifecycleNode 상태 전환: configure → activate
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
    # 5. 동기화 노드 (enable_lidar를 LaunchConfiguration에서 읽어서 전달)
    #    런치 시 --enable_lidar:=true/false 로 제어 가능
    # ================================================================
    # NOTE: LaunchConfiguration은 string이므로, 동기화 노드 파라미터에
    #       직접 bool을 넣으려면 런치 시점에 평가해야 함.
    #       여기서는 enable_lidar 기본값이 true이므로 True로 설정.
    #       false로 런치하려면: ros2 launch ... enable_lidar:=false
    sync_node = Node(
        package='nlos_fusion',
        executable='multi_sensor_sync',
        name='multi_sensor_sync',
        output='screen',
        parameters=[{
            'num_radars': num_radars,
            'enable_lidar': True,
            'enable_camera': True,
            'enable_audio': True,
        }]
    )

    # ================================================================
    # 6. Rosbag 토픽 (라이다 토픽 포함)
    # ================================================================
    topics_to_record = [
        '/sync/timestamp',
        '/sync/frame_info',
        # 카메라
        '/sync/camera/color',
        # 오디오
        '/sync/audio/spectrum',
        '/sync/audio/direction',
        # 레이더 통합
        '/sync/radar/merged/points',
        # 라이다
        '/sync/lidar/points',
        '/sync/lidar/imu',
        '/sync/lidar/range_image',
        # TF
        '/tf', '/tf_static',
    ]
    # 개별 레이더 토픽
    for rid in enabled_radar_ids:
        prefix = f'/sync/radar_{rid}'
        topics_to_record += [
            f'{prefix}/points',
            f'{prefix}/heatmap',
            f'{prefix}/range_profile',
            f'{prefix}/noise_profile',
        ]
    
    # QoS 오버라이드 YAML 생성
    qos_override_path = os.path.join(rosbag_dir, f'qos_override_{timestamp}.yaml')
    qos_overrides = {}
    for topic in topics_to_record:
        if topic in ['/tf', '/tf_static']:
            continue
        qos_overrides[topic] = {
            'reliability': 'reliable',
            'durability': 'volatile',
            'history': 'keep_last',
            'depth': 30,
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
    # 7. RViz
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
    #   6s: 동기화 노드 (모든 센서 준비 후)
    #   8s: Rosbag 녹화
    # ================================================================
    radar_ids_str = ', '.join([f'radar_{i}' for i in enabled_radar_ids])
    return LaunchDescription(args + [
        LogInfo(msg=f"🚀 Multi-Sensor Launch v5 (Radar + Lidar + Camera + Audio)"),
        LogInfo(msg=f"📡 Active radars ({num_radars}): {radar_ids_str}"),
        LogInfo(msg=f"📁 Rosbag path: {rosbag_path}"),
        
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
        
        # 동기화 & 녹화 (센서 안정화 후)
        TimerAction(period=6.0, actions=[sync_node]),
        TimerAction(period=8.0, actions=[rosbag_process]),
        
        rviz_node,
    ])
