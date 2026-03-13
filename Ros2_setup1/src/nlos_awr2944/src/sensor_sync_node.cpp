#include "nlos_awr2944/sensor_sync.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace nlos_awr2944
{

SensorSyncNode::SensorSyncNode(const rclcpp::NodeOptions& options)
    : Node("sensor_sync_node", options)
{
    // 파라미터 선언
    this->declare_parameter<double>("sync_tolerance_ms", 20.0);
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<double>("sync_rate_hz", 50.0);
    
    // 센서 토픽 파라미터
    this->declare_parameter<int>("num_radars", 4);
    this->declare_parameter<std::string>("lidar_topic", "/lidar/points");
    this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("audio_topic", "/audio/data");
    
    // 센서 활성화 파라미터
    this->declare_parameter<bool>("enable_lidar", true);
    this->declare_parameter<bool>("enable_camera", true);
    this->declare_parameter<bool>("enable_audio", true);

    loadConfig();
    setupSubscribers();
    setupPublishers();
    
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "   Multi-Modal Sensor Synchronization    ");
    RCLCPP_INFO(this->get_logger(), "   Tolerance: %.1f ms, Rate: %.1f Hz     ", 
        sync_tolerance_ms_, sync_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "==========================================");
}

SensorSyncNode::~SensorSyncNode()
{
    RCLCPP_INFO(this->get_logger(), "Sync stats - Success: %lu, Miss: %lu",
        sync_count_, miss_count_);
}

void SensorSyncNode::loadConfig()
{
    sync_tolerance_ms_ = this->get_parameter("sync_tolerance_ms").as_double();
    queue_size_ = static_cast<size_t>(this->get_parameter("queue_size").as_int());
    sync_rate_hz_ = this->get_parameter("sync_rate_hz").as_double();
}

void SensorSyncNode::setupSubscribers()
{
    // QoS 설정 - 센서 데이터용
    rclcpp::QoS sensor_qos(10);
    sensor_qos.best_effort();
    sensor_qos.durability_volatile();
    
    // 레이더 구독자 (최대 4개)
    int num_radars = this->get_parameter("num_radars").as_int();
    latest_radar_.resize(static_cast<size_t>(num_radars));
    
    for (int i = 0; i < num_radars; ++i) {
        std::string topic = "radar_" + std::to_string(i) + "/points";
        
        auto callback = [this, i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->onRadarPoints(static_cast<size_t>(i), msg);
        };
        
        auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, sensor_qos, callback);
        radar_subs_.push_back(sub);
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to radar: %s", topic.c_str());
    }
    
    // 라이다 구독자
    if (this->get_parameter("enable_lidar").as_bool()) {
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, sensor_qos,
            std::bind(&SensorSyncNode::onLidarPoints, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to lidar: %s", lidar_topic.c_str());
    }
    
    // 카메라 구독자
    if (this->get_parameter("enable_camera").as_bool()) {
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, sensor_qos,
            std::bind(&SensorSyncNode::onCameraImage, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to camera: %s", camera_topic.c_str());
    }
    
    // 오디오 구독자 (16채널 마이크)
    if (this->get_parameter("enable_audio").as_bool()) {
        std::string audio_topic = this->get_parameter("audio_topic").as_string();
        audio_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            audio_topic, sensor_qos,
            std::bind(&SensorSyncNode::onAudioData, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to audio: %s", audio_topic.c_str());
    }
}

void SensorSyncNode::setupPublishers()
{
    // 동기화 타임스탬프 발행자
    pub_sync_header_ = this->create_publisher<std_msgs::msg::Header>(
        "sync/timestamp", 10);
    
    // 동기화 타이머 설정
    auto period = std::chrono::duration<double>(1.0 / sync_rate_hz_);
    sync_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&SensorSyncNode::syncCallback, this));
}

void SensorSyncNode::onRadarPoints(
    size_t radar_idx,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (radar_idx < latest_radar_.size()) {
        latest_radar_[radar_idx] = msg;
    }
}

void SensorSyncNode::onLidarPoints(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_lidar_ = msg;
}

void SensorSyncNode::onCameraImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_camera_ = msg;
}

void SensorSyncNode::onAudioData(
    const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_audio_ = msg;
}

void SensorSyncNode::syncCallback()
{
    SyncedSensorData synced;
    
    if (trySynchronize(synced)) {
        publishSyncedData(synced);
        sync_count_++;
    } else {
        miss_count_++;
    }
}

bool SensorSyncNode::trySynchronize(SyncedSensorData& synced)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // 기준 시간 결정 (가장 최근 데이터)
    rclcpp::Time reference_time = this->now();
    bool has_any_data = false;
    
    // 레이더 데이터 수집
    for (const auto& radar : latest_radar_) {
        if (radar) {
            synced.radar_points.push_back(radar);
            has_any_data = true;
        }
    }
    
    // 라이다 데이터
    if (latest_lidar_) {
        synced.lidar_points = latest_lidar_;
        has_any_data = true;
    }
    
    // 카메라 데이터
    if (latest_camera_) {
        synced.camera_image = latest_camera_;
        has_any_data = true;
    }
    
    // 오디오 데이터
    if (latest_audio_) {
        synced.audio_data = latest_audio_;
        has_any_data = true;
    }
    
    if (!has_any_data) {
        return false;
    }
    
    // 동기화 품질 계산
    synced.sync_timestamp = reference_time;
    synced.max_time_diff_ms = 0.0;
    synced.all_sensors_present = true;
    
    // 모든 센서 데이터가 있는지 확인
    bool enable_lidar = this->get_parameter("enable_lidar").as_bool();
    bool enable_camera = this->get_parameter("enable_camera").as_bool();
    bool enable_audio = this->get_parameter("enable_audio").as_bool();
    
    if (enable_lidar && !synced.lidar_points) {
        synced.all_sensors_present = false;
    }
    if (enable_camera && !synced.camera_image) {
        synced.all_sensors_present = false;
    }
    if (enable_audio && !synced.audio_data) {
        synced.all_sensors_present = false;
    }
    
    // 시간 차이 계산 (간단한 구현)
    std::vector<rclcpp::Time> timestamps;
    
    for (const auto& radar : synced.radar_points) {
        if (radar) {
            timestamps.push_back(rclcpp::Time(radar->header.stamp));
        }
    }
    if (synced.lidar_points) {
        timestamps.push_back(rclcpp::Time(synced.lidar_points->header.stamp));
    }
    if (synced.camera_image) {
        timestamps.push_back(rclcpp::Time(synced.camera_image->header.stamp));
    }
    
    if (timestamps.size() >= 2) {
        auto [min_it, max_it] = std::minmax_element(timestamps.begin(), timestamps.end());
        synced.max_time_diff_ms = (*max_it - *min_it).nanoseconds() / 1e6;
    }
    
    // 버퍼 초기화 (데이터 사용 후)
    // 참고: 실제 구현에서는 큐 기반으로 변경 권장
    
    return true;
}

void SensorSyncNode::publishSyncedData(const SyncedSensorData& synced)
{
    // 동기화 타임스탬프 발행
    std_msgs::msg::Header header;
    header.stamp = synced.sync_timestamp;
    header.frame_id = "synced_sensors";
    pub_sync_header_->publish(header);
    
    // 디버그 로그 (10프레임마다)
    if (sync_count_ % 10 == 0) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Sync #%lu: radars=%zu, lidar=%d, camera=%d, audio=%d, diff=%.2fms",
            sync_count_,
            synced.radar_points.size(),
            synced.lidar_points ? 1 : 0,
            synced.camera_image ? 1 : 0,
            synced.audio_data ? 1 : 0,
            synced.max_time_diff_ms);
    }
}

}  // namespace nlos_awr2944

// ===========================================
// main 함수
// ===========================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<nlos_awr2944::SensorSyncNode>();
    
    RCLCPP_INFO(node->get_logger(), "🔄 Sensor synchronization node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
