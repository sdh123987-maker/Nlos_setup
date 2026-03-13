#ifndef NLOS_AWR2944__RADAR_DRIVER_CORE_HPP_
#define NLOS_AWR2944__RADAR_DRIVER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

namespace nlos_awr2944
{

// ===========================================
// 상수 정의
// ===========================================
constexpr size_t MAGIC_WORD_SIZE = 8;
constexpr uint8_t MAGIC_WORD[MAGIC_WORD_SIZE] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
constexpr size_t FRAME_HEADER_SIZE = 40;
constexpr size_t TLV_HEADER_SIZE = 8;

// TLV Types
enum class TLVType : uint32_t {
    DETECTED_POINTS = 1,
    RANGE_PROFILE = 2,
    NOISE_PROFILE = 3,
    AZIMUTH_STATIC_HEATMAP = 4,
    RANGE_DOPPLER_HEATMAP = 5,
    STATS = 6,
    DETECTED_POINTS_SIDE_INFO = 7,
    AZIMUTH_ELEVATION_HEATMAP = 8,
    TEMPERATURE_STATS = 9
};

// ===========================================
// 데이터 구조체
// ===========================================

#pragma pack(push, 1)
struct FrameHeader {
    uint8_t magic_word[8];
    uint32_t version;
    uint32_t total_packet_len;
    uint32_t platform;
    uint32_t frame_number;
    uint32_t time_cpu_cycles;
    uint32_t num_detected_obj;
    uint32_t num_tlvs;
    uint32_t sub_frame_number;
};

struct TLVHeader {
    uint32_t type;
    uint32_t length;
};

struct DetectedPoint {
    float x;
    float y;
    float z;
    float velocity;
};

struct PointSideInfo {
    int16_t snr;
    int16_t noise;
};
#pragma pack(pop)

// ROS 좌표계로 변환된 포인트
struct RadarPoint {
    float x;
    float y;
    float z;
    float velocity;
    float intensity;
    uint32_t frame_id;
};

// 프레임 데이터
struct RadarFrame {
    uint32_t frame_number;
    rclcpp::Time timestamp;
    std::vector<RadarPoint> points;
    std::vector<uint16_t> range_profile;
    std::vector<uint16_t> noise_profile;
    std::vector<float> azimuth_heatmap;
    uint32_t heatmap_width;
    uint32_t heatmap_height;
    bool valid;
};

// ===========================================
// 설정 구조체
// ===========================================
struct SerialConfig {
    std::string cli_port;
    std::string data_port;
    int cli_baudrate;
    int data_baudrate;
    double timeout;
};

struct RadarConfig {
    std::string radar_id;
    std::string frame_id;
    SerialConfig serial;
    std::vector<std::string> commands;
    
    double pos_x;
    double pos_y;
    double pos_z;
    double roll;
    double pitch;
    double yaw;
};

// ===========================================
// 콜백 타입 정의
// ===========================================
using FrameCallback = std::function<void(const RadarFrame&)>;

// ===========================================
// 레이더 드라이버 클래스
// ===========================================
class RadarDriver
{
public:
    explicit RadarDriver(const RadarConfig& config);
    ~RadarDriver();

    bool connect();
    void disconnect();
    bool isConnected() const;

    bool sendConfiguration();
    bool startSensor();
    bool stopSensor();

    void setFrameCallback(FrameCallback callback);
    void startReceiving();
    void stopReceiving();

    std::string getRadarId() const { return config_.radar_id; }
    uint32_t getFrameCount() const { return frame_count_.load(); }

private:
    void receiveThread();
    
    RadarConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint32_t> frame_count_{0};
    
    int cli_fd_{-1};
    int data_fd_{-1};
    
    std::thread receive_thread_;
    std::mutex callback_mutex_;
    FrameCallback frame_callback_;
    
    std::vector<uint8_t> buffer_;
};

// ===========================================
// 레이더 노드 클래스
// ===========================================
class RadarNode : public rclcpp::Node
{
public:
    explicit RadarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RadarNode();

private:
    void loadConfig();
    void setupPublishers();
    void setupTF();
    void onFrame(const RadarFrame& frame);
    
    sensor_msgs::msg::PointCloud2 createPointCloud2(
        const RadarFrame& frame, 
        const rclcpp::Time& stamp);
    
    std::unique_ptr<RadarDriver> driver_;
    RadarConfig config_;
    
    // 시간 관리
    rclcpp::Time start_time_;
    bool time_initialized_{false};
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_range_profile_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_noise_profile_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_heatmap_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_timestamp_;
    
    // TF
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace nlos_awr2944

#endif  // NLOS_AWR2944__RADAR_DRIVER_CORE_HPP_