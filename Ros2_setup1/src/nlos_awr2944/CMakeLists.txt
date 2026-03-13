cmake_minimum_required(VERSION 3.10)
project(nlos_awr2944)

# C++17 표준 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# JSON 라이브러리 찾기 (추가됨)
find_package(nlohmann_json REQUIRED)

# Include 디렉토리
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ===========================================
# 라이브러리: 레이더 드라이버 코어
# ===========================================
add_library(radar_driver_core SHARED
  src/radar_driver_core.cpp
  src/radar_parser.cpp
  src/radar_serial.cpp
)
ament_target_dependencies(radar_driver_core
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
)
target_link_libraries(radar_driver_core ${PCL_LIBRARIES})

# ===========================================
# 실행 파일: 단일 레이더 노드
# ===========================================
add_executable(radar_node
  src/radar_node.cpp
)
# JSON 라이브러리 링크 추가 (추가됨)
target_link_libraries(radar_node 
  radar_driver_core 
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(radar_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ===========================================
# 실행 파일: 멀티 레이더 매니저
# ===========================================
add_executable(multi_radar_manager
  src/multi_radar_manager.cpp
)
target_link_libraries(multi_radar_manager radar_driver_core)
ament_target_dependencies(multi_radar_manager
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
)

# ===========================================
# 실행 파일: 센서 동기화 노드
# ===========================================
add_executable(sensor_sync_node
  src/sensor_sync_node.cpp
)
ament_target_dependencies(sensor_sync_node
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
)

# ===========================================
# 설치
# ===========================================
install(TARGETS
  radar_driver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  radar_node
  multi_radar_manager
  sensor_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
