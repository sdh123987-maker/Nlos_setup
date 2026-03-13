#include "nlos_awr2944/radar_driver_core.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cstring>
#include <algorithm>
#include <fstream> // 파일 입출력용
#include <ament_index_cpp/get_package_share_directory.hpp> // 패키지 경로 찾기용
#include <nlohmann/json.hpp> // JSON 파싱용 (ROS 2 기본 라이브러리)

using json = nlohmann::json;

// Forward declarations
namespace nlos_awr2944 {
namespace serial {
    int openSerialPort(const std::string& port, int baudrate, double timeout_sec);
    ssize_t writeSerial(int fd, const void* data, size_t len);
    ssize_t readSerial(int fd, void* data, size_t max_len);
    int availableBytes(int fd);
    void closeSerialPort(int fd);
    void flushSerial(int fd);
}

int findMagicWord(const std::vector<uint8_t>& buffer, size_t start_pos);
bool parseFullFrame(const std::vector<uint8_t>& data, RadarFrame& frame, rclcpp::Clock::SharedPtr clock);
}

namespace nlos_awr2944
{

// ===========================================
// RadarDriver 구현 (변경 없음)
// ===========================================

RadarDriver::RadarDriver(const RadarConfig& config)
    : config_(config)
{
    buffer_.reserve(65536);
}

RadarDriver::~RadarDriver()
{
    stopReceiving();
    disconnect();
}

bool RadarDriver::connect()
{
    if (connected_) {
        return true;
    }

    cli_fd_ = serial::openSerialPort(
        config_.serial.cli_port,
        config_.serial.cli_baudrate,
        config_.serial.timeout);
    
    if (cli_fd_ < 0) {
        return false;
    }

    data_fd_ = serial::openSerialPort(
        config_.serial.data_port,
        config_.serial.data_baudrate,
        config_.serial.timeout);
    
    if (data_fd_ < 0) {
        serial::closeSerialPort(cli_fd_);
        cli_fd_ = -1;
        return false;
    }

    connected_ = true;
    return true;
}

void RadarDriver::disconnect()
{
    stopReceiving();
    
    if (cli_fd_ >= 0) {
        serial::closeSerialPort(cli_fd_);
        cli_fd_ = -1;
    }
    
    if (data_fd_ >= 0) {
        serial::closeSerialPort(data_fd_);
        data_fd_ = -1;
    }
    
    connected_ = false;
}

bool RadarDriver::isConnected() const
{
    return connected_;
}

bool RadarDriver::sendConfiguration()
{
    if (!connected_) {
        return false;
    }

    serial::flushSerial(cli_fd_);

    for (const auto& cmd : config_.commands) {
        if (cmd.empty() || cmd[0] == '%') {
            continue;
        }

        std::string full_cmd = cmd + "\r";
        ssize_t written = serial::writeSerial(cli_fd_, full_cmd.c_str(), full_cmd.size());
        
        if (written < 0) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        
        char response[1024];
        serial::readSerial(cli_fd_, response, sizeof(response));
    }

    return true;
}

bool RadarDriver::startSensor()
{
    if (!connected_) {
        return false;
    }

    std::string cmd = "sensorStart\r";
    return serial::writeSerial(cli_fd_, cmd.c_str(), cmd.size()) > 0;
}

bool RadarDriver::stopSensor()
{
    if (!connected_) {
        return false;
    }

    std::string cmd = "sensorStop\r";
    return serial::writeSerial(cli_fd_, cmd.c_str(), cmd.size()) > 0;
}

void RadarDriver::setFrameCallback(FrameCallback callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = std::move(callback);
}

void RadarDriver::startReceiving()
{
    if (running_) {
        return;
    }

    running_ = true;
    receive_thread_ = std::thread(&RadarDriver::receiveThread, this);
}

void RadarDriver::stopReceiving()
{
    running_ = false;
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

void RadarDriver::receiveThread()
{
    constexpr size_t READ_BUFFER_SIZE = 4096;
    std::vector<uint8_t> read_buffer(READ_BUFFER_SIZE);
    
    auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    
    while (running_) {
        int available = serial::availableBytes(data_fd_);
        
        if (available > 0) {
            size_t to_read = std::min(static_cast<size_t>(available), READ_BUFFER_SIZE);
            ssize_t bytes_read = serial::readSerial(data_fd_, read_buffer.data(), to_read);
            
            if (bytes_read > 0) {
                buffer_.insert(buffer_.end(), read_buffer.begin(), read_buffer.begin() + bytes_read);
            }
        }

        while (buffer_.size() >= FRAME_HEADER_SIZE) {
            int magic_pos = findMagicWord(buffer_, 0);
            
            if (magic_pos < 0) {
                if (buffer_.size() > MAGIC_WORD_SIZE) {
                    buffer_.erase(buffer_.begin(), buffer_.end() - MAGIC_WORD_SIZE);
                }
                break;
            }
            
            if (magic_pos > 0) {
                buffer_.erase(buffer_.begin(), buffer_.begin() + magic_pos);
            }
            
            if (buffer_.size() < FRAME_HEADER_SIZE) {
                break;
            }
            
            uint32_t packet_len;
            std::memcpy(&packet_len, buffer_.data() + 12, sizeof(uint32_t));
            
            if (buffer_.size() < packet_len) {
                break;
            }
            
            std::vector<uint8_t> frame_data(buffer_.begin(), buffer_.begin() + packet_len);
            buffer_.erase(buffer_.begin(), buffer_.begin() + packet_len);
            
            RadarFrame frame;
            if (parseFullFrame(frame_data, frame, clock)) {
                frame_count_++;
                
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (frame_callback_) {
                    frame_callback_(frame);
                }
            }
        }

        if (available <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

// ===========================================
// RadarNode 구현
// ===========================================

RadarNode::RadarNode(const rclcpp::NodeOptions& options)
    : Node("radar_node", options)
{
    // 파라미터 선언
    this->declare_parameter<std::string>("radar_id", "radar_0");
    this->declare_parameter<std::string>("frame_id", "radar_link");
    this->declare_parameter<std::string>("cli_port", "/dev/ttyACM0");
    this->declare_parameter<std::string>("data_port", "/dev/ttyACM1");
    this->declare_parameter<int>("cli_baudrate", 115200);
    this->declare_parameter<int>("data_baudrate", 3125000);
    this->declare_parameter<double>("timeout", 0.05);
    
    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<double>("pos_z", 0.0);
    this->declare_parameter<double>("roll", 0.0);
    this->declare_parameter<double>("pitch", 0.0);
    this->declare_parameter<double>("yaw", 0.0);

    // [추가] JSON 설정 파일 경로 파라미터 (기본값: 패키지 share 디렉토리 내 config/radar_config.json)
    std::string default_config_path = "";
    try {
        default_config_path = ament_index_cpp::get_package_share_directory("nlos_awr2944") + "/config/radar_config.json";
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find package share directory: %s", e.what());
    }
    this->declare_parameter<std::string>("config_file", default_config_path);

    loadConfig();
    setupPublishers();
    setupTF();

    // 드라이버 생성 및 연결
    driver_ = std::make_unique<RadarDriver>(config_);
    
    if (!driver_->connect()) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to connect to radar: %s", 
            config_.radar_id.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Serial Ports Connected.");

    if (!driver_->sendConfiguration()) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to send configuration");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Radar configured from JSON file");

    driver_->setFrameCallback(
        [this](const RadarFrame& frame) {
            this->onFrame(frame);
        });

    driver_->startReceiving();
    
    RCLCPP_INFO(this->get_logger(), "🚀 Radar node started: %s", config_.radar_id.c_str());
}

RadarNode::~RadarNode()
{
    RCLCPP_INFO(this->get_logger(), "🛑 Stopping radar node...");
    
    if (driver_) {
        driver_->stopSensor();
        driver_->disconnect();
    }
}

void RadarNode::loadConfig()
{
    // 1. ROS 파라미터 로드
    config_.radar_id = this->get_parameter("radar_id").as_string();
    config_.frame_id = this->get_parameter("frame_id").as_string();
    config_.serial.cli_port = this->get_parameter("cli_port").as_string();
    config_.serial.data_port = this->get_parameter("data_port").as_string();
    config_.serial.cli_baudrate = this->get_parameter("cli_baudrate").as_int();
    config_.serial.data_baudrate = this->get_parameter("data_baudrate").as_int();
    config_.serial.timeout = this->get_parameter("timeout").as_double();
    
    config_.pos_x = this->get_parameter("pos_x").as_double();
    config_.pos_y = this->get_parameter("pos_y").as_double();
    config_.pos_z = this->get_parameter("pos_z").as_double();
    config_.roll = this->get_parameter("roll").as_double();
    config_.pitch = this->get_parameter("pitch").as_double();
    config_.yaw = this->get_parameter("yaw").as_double();

    // 2. JSON 파일 로드 및 파싱 (하드코딩 제거됨)
    std::string config_file_path = this->get_parameter("config_file").as_string();
    RCLCPP_INFO(this->get_logger(), "📂 Loading configuration from: %s", config_file_path.c_str());

    std::ifstream file(config_file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to open config file! Using empty commands.");
        return;
    }

    try {
        json j;
        file >> j; // JSON 파싱

        // "radar_commands" 키 확인 및 로드
        if (j.contains("radar_commands") && j["radar_commands"].is_array()) {
            config_.commands.clear();
            for (const auto& cmd : j["radar_commands"]) {
                if (cmd.is_string()) {
                    config_.commands.push_back(cmd.get<std::string>());
                }
            }
            RCLCPP_INFO(this->get_logger(), "✅ Loaded %zu commands from JSON.", config_.commands.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 'radar_commands' not found or not an array in JSON.");
        }
    } catch (const json::parse_error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ JSON Parse Error: %s", e.what());
    }
}

void RadarNode::setupPublishers()
{
    pub_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar/points", 10);
    
    pub_range_profile_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "radar/range_profile", 10);
    
    pub_noise_profile_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "radar/noise_profile", 10);
    
    pub_heatmap_ = this->create_publisher<sensor_msgs::msg::Image>(
        "radar/heatmap/azimuth", 10);
    
    pub_timestamp_ = this->create_publisher<std_msgs::msg::Header>(
        "radar/timestamp", 10);
}

void RadarNode::setupTF()
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = config_.frame_id;
    
    transform.transform.translation.x = config_.pos_x;
    transform.transform.translation.y = config_.pos_y;
    transform.transform.translation.z = config_.pos_z;
    
    tf2::Quaternion q;
    q.setRPY(config_.roll, config_.pitch, config_.yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    static_tf_broadcaster_->sendTransform(transform);
}

void RadarNode::onFrame(const RadarFrame& frame)
{
    auto stamp = this->now();
    
    // 시간 초기화
    if (!time_initialized_) {
        start_time_ = stamp;
        time_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "⏰ First Frame Received! Time Reset to 0.0s");
    }
    
    // 타임스탬프 발행
    std_msgs::msg::Header header_msg;
    header_msg.stamp = stamp;
    header_msg.frame_id = config_.frame_id;
    pub_timestamp_->publish(header_msg);
    
    // 포인트 클라우드 발행
    if (!frame.points.empty()) {
        auto pc_msg = createPointCloud2(frame, stamp);
        pub_points_->publish(pc_msg);
    }
    
    // Range Profile 발행
    if (!frame.range_profile.empty()) {
        std_msgs::msg::Float32MultiArray range_msg;
        range_msg.data.reserve(frame.range_profile.size());
        for (auto val : frame.range_profile) {
            range_msg.data.push_back(static_cast<float>(val));
        }
        pub_range_profile_->publish(range_msg);
    }
    
    // Noise Profile 발행
    if (!frame.noise_profile.empty()) {
        std_msgs::msg::Float32MultiArray noise_msg;
        noise_msg.data.reserve(frame.noise_profile.size());
        for (auto val : frame.noise_profile) {
            noise_msg.data.push_back(static_cast<float>(val));
        }
        pub_noise_profile_->publish(noise_msg);
    }
    
    // Heatmap 발행
    if (!frame.azimuth_heatmap.empty()) {
        sensor_msgs::msg::Image heatmap_msg;
        heatmap_msg.header.stamp = stamp;
        heatmap_msg.header.frame_id = config_.frame_id;
        heatmap_msg.height = frame.heatmap_height;
        heatmap_msg.width = frame.heatmap_width;
        heatmap_msg.encoding = "mono8";
        heatmap_msg.step = frame.heatmap_width;
        
        float max_val = *std::max_element(frame.azimuth_heatmap.begin(), frame.azimuth_heatmap.end());
        heatmap_msg.data.reserve(frame.azimuth_heatmap.size());
        for (auto val : frame.azimuth_heatmap) {
            uint8_t normalized = (max_val > 0) ? 
                static_cast<uint8_t>(val / max_val * 255.0f) : 0;
            heatmap_msg.data.push_back(normalized);
        }
        pub_heatmap_->publish(heatmap_msg);
    }
}

sensor_msgs::msg::PointCloud2 RadarNode::createPointCloud2(
    const RadarFrame& frame,
    const rclcpp::Time& stamp)
{
    sensor_msgs::msg::PointCloud2 msg;
    
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.height = 1;
    msg.width = frame.points.size();
    
    msg.fields.resize(6);
    
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    
    msg.fields[3].name = "velocity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;
    
    msg.fields[4].name = "intensity";
    msg.fields[4].offset = 16;
    msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[4].count = 1;
    
    msg.fields[5].name = "frame";
    msg.fields[5].offset = 20;
    msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[5].count = 1;
    
    msg.is_bigendian = false;
    msg.point_step = 24;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    
    msg.data.resize(msg.row_step);
    uint8_t* data_ptr = msg.data.data();
    
    for (const auto& point : frame.points) {
        std::memcpy(data_ptr, &point.x, sizeof(float));
        std::memcpy(data_ptr + 4, &point.y, sizeof(float));
        std::memcpy(data_ptr + 8, &point.z, sizeof(float));
        std::memcpy(data_ptr + 12, &point.velocity, sizeof(float));
        std::memcpy(data_ptr + 16, &point.intensity, sizeof(float));
        float frame_f = static_cast<float>(point.frame_id);
        std::memcpy(data_ptr + 20, &frame_f, sizeof(float));
        data_ptr += msg.point_step;
    }
    
    return msg;
}

}  // namespace nlos_awr2944
