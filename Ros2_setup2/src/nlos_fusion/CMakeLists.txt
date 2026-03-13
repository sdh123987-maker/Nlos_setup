cmake_minimum_required(VERSION 3.8)
project(nlos_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. C++ 노드 빌드
# NOTE: multi_sensor_sync는 v6에서 제거됨 (후처리 동기화로 전환)

# 3. 설치 (Install) 규칙

# 3-1. Python 스크립트 설치
install(PROGRAMS
  scripts/audio_node.py
  scripts/sound_localizer.py
  scripts/process_rosbag.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3-2. 기타 리소스 설치 (launch, rviz, config)
install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()