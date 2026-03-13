# NLOS AWR2944 Radar Driver (C++)

AWR2944 mmWave 레이더를 위한 ROS2 C++ 드라이버 패키지입니다.
다중 센서 동기화를 위해 최적화되었습니다.

## 특징

- **C++ 기반 고성능 드라이버**: Python 대비 낮은 레이턴시, 정밀한 타이밍 제어
- **멀티 레이더 지원**: 최대 4개의 AWR2944 레이더 동시 운용
- **센서 동기화**: 레이더, 라이다, 카메라, 마이크(16채널) 시간 동기화
- **유연한 설정**: YAML/JSON 기반 설정 파일
- **RViz2 시각화**: 다양한 시각화 설정 제공

## 의존성

```bash
# ROS2 Humble 또는 이후 버전
sudo apt install ros-humble-tf2-ros ros-humble-pcl-conversions libpcl-dev
```

## 빌드

```bash
cd ~/Nlos/Ros2_setup
colcon build --packages-select nlos_awr2944
source install/setup.bash
```

## 사용법

### 1. 단일 레이더 실행

```bash
ros2 launch nlos_awr2944 single_radar.launch.py
```

시리얼 포트 지정:
```bash
ros2 launch nlos_awr2944 single_radar.launch.py cli_port:=/dev/ttyACM0 data_port:=/dev/ttyACM1
```

### 2. 멀티 레이더 실행 (4개)

```bash
ros2 launch nlos_awr2944 multi_radar.launch.py
```

### 3. 전체 센서 실행 (레이더 + 라이다 + 카메라 + 마이크)

```bash
ros2 launch nlos_awr2944 all_sensors.launch.py
```

옵션:
```bash
ros2 launch nlos_awr2944 all_sensors.launch.py \
    enable_radar:=true \
    enable_lidar:=true \
    enable_camera:=true \
    enable_audio:=true \
    use_rviz:=true
```

## 토픽

### 단일 레이더
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/radar/points` | `sensor_msgs/PointCloud2` | 포인트 클라우드 |
| `/radar/timestamp` | `std_msgs/Header` | 타임스탬프 |
| `/radar/range_profile` | `std_msgs/Float32MultiArray` | Range Profile |
| `/radar/heatmap/azimuth` | `sensor_msgs/Image` | Azimuth Heatmap |

### 멀티 레이더
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/radar/merged/points` | `sensor_msgs/PointCloud2` | 병합된 포인트 클라우드 |
| `/radar/sync/timestamp` | `std_msgs/Header` | 동기화 타임스탬프 |
| `/radar_0/points` | `sensor_msgs/PointCloud2` | 레이더 0 포인트 |
| `/radar_1/points` | `sensor_msgs/PointCloud2` | 레이더 1 포인트 |
| `/radar_2/points` | `sensor_msgs/PointCloud2` | 레이더 2 포인트 |
| `/radar_3/points` | `sensor_msgs/PointCloud2` | 레이더 3 포인트 |

## 설정

### 시리얼 포트 설정

레이더 4개 사용 시 권장 포트 매핑:

| 레이더 | CLI 포트 | Data 포트 |
|--------|----------|-----------|
| Radar 0 | /dev/ttyACM0 | /dev/ttyACM1 |
| Radar 1 | /dev/ttyACM2 | /dev/ttyACM3 |
| Radar 2 | /dev/ttyACM4 | /dev/ttyACM5 |
| Radar 3 | /dev/ttyACM6 | /dev/ttyACM7 |

### udev 규칙 설정 (권장)

시리얼 포트를 고정하려면 udev 규칙을 설정하세요:

```bash
# /etc/udev/rules.d/99-awr2944.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{serial}=="RADAR0_SERIAL", SYMLINK+="radar0_cli"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{serial}=="RADAR0_SERIAL", SYMLINK+="radar0_data"
# ... 나머지 레이더도 동일하게 설정
```

### 레이더 위치 설정

`config/multi_radar.yaml`에서 각 레이더의 위치와 방향을 설정합니다:

```yaml
radar_0:
  enabled: true
  pos_x: 0.5    # base_link 기준 X 위치 (m)
  pos_y: 0.0    # base_link 기준 Y 위치 (m)
  pos_z: 0.3    # base_link 기준 Z 위치 (m)
  yaw: 0.0      # 방향 (rad) - 0: 전방
```

## TF 프레임 구조

```
map
 └── base_link
      ├── radar_0_link (전방)
      ├── radar_1_link (좌측)
      ├── radar_2_link (후방)
      ├── radar_3_link (우측)
      ├── radar_merged (병합)
      ├── lidar_link
      ├── camera_link
      └── mic_array_link
```

## 파일 구조

```
nlos_awr2944/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/nlos_awr2944/
│   ├── radar_driver_core.hpp      # 핵심 드라이버 헤더
│   ├── multi_radar_manager.hpp    # 멀티 레이더 매니저
│   └── sensor_sync.hpp            # 센서 동기화
├── src/
│   ├── radar_driver_core.cpp      # 드라이버 구현
│   ├── radar_serial.cpp           # 시리얼 통신
│   ├── radar_parser.cpp           # TLV 파싱
│   ├── radar_node.cpp             # 단일 레이더 노드
│   ├── multi_radar_manager.cpp    # 멀티 레이더 노드
│   └── sensor_sync_node.cpp       # 센서 동기화 노드
├── config/
│   ├── single_radar.yaml          # 단일 레이더 설정
│   ├── multi_radar.yaml           # 멀티 레이더 설정
│   ├── radar_config.json          # 레이더 명령어
│   └── sensor_sync.yaml           # 센서 동기화 설정
├── launch/
│   ├── single_radar.launch.py     # 단일 레이더 런치
│   ├── multi_radar.launch.py      # 멀티 레이더 런치
│   └── all_sensors.launch.py      # 전체 센서 런치
└── rviz/
    ├── radar.rviz                 # 단일 레이더 시각화
    ├── multi_radar.rviz           # 멀티 레이더 시각화
    └── all_sensors.rviz           # 전체 센서 시각화
```

## 성능 비교 (Python vs C++)

| 항목 | Python | C++ |
|------|--------|-----|
| 프레임 처리 지연 | ~5-10ms | <1ms |
| CPU 사용률 | 높음 | 낮음 |
| 동기화 정밀도 | ~10ms | <1ms |
| 멀티스레딩 | GIL 제한 | 네이티브 |

## 문제 해결

### 시리얼 포트 권한
```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인
```

### 포트를 찾을 수 없음
```bash
# 연결된 장치 확인
ls -la /dev/ttyACM*
# 또는
dmesg | grep tty
```

### 빌드 오류
```bash
# 의존성 재설치
rosdep install --from-paths src --ignore-src -r -y
```

## 라이선스

MIT License
