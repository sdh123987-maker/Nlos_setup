#ifndef NLOS_AWR2944__SENSOR_SYNC_HPP_
#define NLOS_AWR2944__SENSOR_SYNC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>
#include <string>
#include <vector>

namespace nlos_awr2944
{

// ===========================================
// 센서 타입 정의
// ===========================================
enum class SensorType {
    RADAR,
    LIDAR,
    CAMERA,
    MICROPHONE
};

// ===========================================
// 동기화된 센서 데이터 패킷
// ===========================================
struct SyncedSensorData {
    rclcpp::Time sync_timestamp;
    
    // 레이더 (최대 4개)
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> radar_points;
    
    // 라이다
    sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_points;
    
    // 카메라
    sensor_msgs::msg::Image::ConstSharedPtr camera_image;
    
    // 마이크 (16채널 오디오)
    std_msgs::msg::Float32MultiArray::ConstSharedPtr audio_data;
    
    // 동기화 품질 정보
    double max_time_diff_ms;  // 센서 간 최대 시간 차이
    bool all_sensors_present;
};

// ===========================================
// 센서 동기화 노드
// ===========================================
class SensorSyncNode : public rclcpp::Node
{
public:
    explicit SensorSyncNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SensorSyncNode();

private:
    void loadConfig();
    void setupSubscribers();
    void setupPublishers();
    
    // 개별 센서 콜백 (동기화 없이 저장)
    void onRadarPoints(
        size_t radar_idx,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    
    void onLidarPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void onCameraImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onAudioData(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
    
    // 동기화 타이머 콜백
    void syncCallback();
    
    // 동기화 수행
    bool trySynchronize(SyncedSensorData& synced);
    
    // 동기화된 데이터 발행
    void publishSyncedData(const SyncedSensorData& synced);

    // 설정
    double sync_tolerance_ms_;
    size_t queue_size_;
    double sync_rate_hz_;
    
    // Subscribers
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> radar_subs_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr audio_sub_;
    
    // 최신 데이터 버퍼
    std::mutex buffer_mutex_;
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> latest_radar_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_lidar_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_camera_;
    std_msgs::msg::Float32MultiArray::ConstSharedPtr latest_audio_;
    
    // Publishers (동기화된 타임스탬프)
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_sync_header_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr sync_timer_;
    
    // 통계
    uint64_t sync_count_{0};
    uint64_t miss_count_{0};
};

}  // namespace nlos_awr2944

#endif  // NLOS_AWR2944__SENSOR_SYNC_HPP_
