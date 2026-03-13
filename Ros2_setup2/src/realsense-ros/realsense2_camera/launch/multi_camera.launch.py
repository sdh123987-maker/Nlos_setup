"""
==============================================
NLOS Multi-Camera Launch
==============================================
multi_radar.launch.py와 동일한 패턴으로 구현

특징:
  - multi_camera.yaml에서 enabled: true인 카메라만 실행
  - 각 카메라는 별도 namespace로 분리 (cam_0, cam_1, ...)
  - serial_no로 특정 카메라 지정 가능
  - 카메라별 위치(TF) 자동 발행
  - USB 대역폭 고려하여 순차 실행 (1초 간격)

토픽 구조:
  /cam_0/cam_0/color/image_raw
  /cam_1/cam_1/color/image_raw
  ...

사용법:
  ros2 launch nlos_fusion multi_camera.launch.py
  ros2 launch nlos_fusion multi_camera.launch.py use_rviz:=false
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MAX_CAMERAS = 4


def load_camera_config(yaml_path):
    """multi_camera.yaml에서 카메라 설정 로드"""
    try:
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        multi_cam = config.get('multi_camera', {})
        
        # 전역 설정
        global_config = {
            'enable_depth': str(multi_cam.get('enable_depth', False)).lower(),
            'color_profile': multi_cam.get('color_profile', '640,480,30'),
            'pointcloud_enable': str(multi_cam.get('pointcloud_enable', False)).lower(),
        }
        
        # 개별 카메라 설정
        cameras = []
        for i in range(MAX_CAMERAS):
            cam_key = f'cam_{i}'
            cam_cfg = multi_cam.get(cam_key, {})
            if isinstance(cam_cfg, dict) and cam_cfg.get('enabled', False):
                cameras.append({
                    'id': i,
                    'serial_no': str(cam_cfg.get('serial_no', '')),
                    'camera_name': cam_cfg.get('camera_name', f'cam_{i}'),
                    'camera_namespace': cam_cfg.get('camera_namespace', f'cam_{i}'),
                    'pos_x': cam_cfg.get('pos_x', 0.0),
                    'pos_y': cam_cfg.get('pos_y', 0.0),
                    'pos_z': cam_cfg.get('pos_z', 0.0),
                    'roll': cam_cfg.get('roll', 0.0),
                    'pitch': cam_cfg.get('pitch', 0.0),
                    'yaw': cam_cfg.get('yaw', 0.0),
                })
        
        return global_config, cameras
    except Exception as e:
        print(f"⚠️ multi_camera.yaml 파싱 실패: {e}")
        print(f"   기본값: cam_0 1대만 실행")
        global_config = {
            'enable_depth': 'false',
            'color_profile': '640,480,30',
            'pointcloud_enable': 'false',
        }
        cameras = [{
            'id': 0, 'serial_no': '', 'camera_name': 'cam_0',
            'camera_namespace': 'cam_0',
            'pos_x': 0.0, 'pos_y': 0.0, 'pos_z': 0.3,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        }]
        return global_config, cameras


def generate_launch_description():
    # 1. YAML 파일은 nlos_fusion 패키지에서 찾도록 설정
    pkg_fusion = get_package_share_directory('nlos_fusion')
    yaml_path = os.path.join(pkg_fusion, 'config', 'multi_camera.yaml')
    
    # YAML 설정 로드
    global_config, cameras = load_camera_config(yaml_path)
    
    # 2. 카메라 런치 파일을 부르기 위해 realsense 패키지 경로도 선언 (yaml_path 덮어쓰기 삭제)
    pkg_camera = get_package_share_directory('realsense2_camera')
    
    num_cameras = len(cameras)
    
    args = [
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz for camera visualization'),
    ]
    
    # ================================================================
    # 로그
    # ================================================================
    cam_ids_str = ', '.join([f"cam_{c['id']}" for c in cameras])
    log_actions = [
        LogInfo(msg=f"📷 Multi-Camera Launch"),
        LogInfo(msg=f"📷 Active cameras ({num_cameras}): {cam_ids_str}"),
    ]
    
    # ================================================================
    # 각 카메라 노드 (순차 실행, USB 대역폭 고려)
    # ================================================================
    camera_actions = []
    tf_actions = []
    
    for idx, cam in enumerate(cameras):
        cam_id = cam['id']
        # serial_no를 작은따옴표로 감싸서 rs_launch.py가 문자열로 인식하도록 함
        serial = "'" + cam['serial_no'] + "'" if cam['serial_no'] else "''"
        
        # rs_launch.py 호출 (각 카메라별 다른 namespace + camera_name)
        cam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_camera, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_name': cam['camera_name'],
                'camera_namespace': cam['camera_namespace'],
                'serial_no': serial,
                'enable_color': 'true',
                'enable_depth': global_config['enable_depth'],
                'rgb_camera.color_profile': global_config['color_profile'],
                'pointcloud.enable': global_config['pointcloud_enable'],
            }.items()
        )
        
        # USB 대역폭 고려: 카메라별 1초 간격 순차 실행
        delay = idx * 1.5  # 1.5초 간격
        if delay == 0:
            camera_actions.append(cam_launch)
        else:
            camera_actions.append(
                TimerAction(
                    period=delay,
                    actions=[
                        LogInfo(msg=f"📷 Starting cam_{cam_id}..."),
                        cam_launch,
                    ]
                )
            )
        
        # TF: base_link → cam_X_link
        frame_id = f"cam_{cam_id}_link"
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'cam_{cam_id}_tf',
            arguments=[
                str(cam['pos_x']), str(cam['pos_y']), str(cam['pos_z']),
                str(cam['yaw']), str(cam['pitch']), str(cam['roll']),
                'base_link', frame_id,
            ]
        )
        tf_actions.append(tf_node)
    
    # ================================================================
    # RViz (선택)
    # ================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    
    return LaunchDescription(
        args
        + log_actions
        + tf_actions
        + camera_actions
        + [rviz_node]
    )