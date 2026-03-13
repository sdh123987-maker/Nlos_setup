#include "nlos_awr2944/multi_radar_manager.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>

using json = nlohmann::json;

namespace nlos_awr2944
{

MultiRadarManager::MultiRadarManager(const rclcpp::NodeOptions& options)
    : Node("multi_radar_manager", options)
{
    // 1. 파라미터 선언
    this->declare_parameter<std::string>("config_file", ""); 
    this->declare_parameter<double>("sync_tolerance_ms", 20.0);
    this->declare_parameter<double>("target_freq_hz", 10.0);
    this->declare_parameter<bool>("merge_pointclouds", true);
    this->declare_parameter<std::string>("merged_frame_id", "radar_merged");

    // 4개 레이더 파라미터 선언
    for (size_t i = 0; i < MAX_RADARS; ++i) {
        std::string prefix = "radar_" + std::to_string(i) + ".";
        this->declare_parameter<bool>(prefix + "enabled", false);
        this->declare_parameter<std::string>(prefix + "cli_port", "");
        this->declare_parameter<std::string>(prefix + "data_port", "");
        this->declare_parameter<std::string>(prefix + "frame_id", "radar_" + std::to_string(i) + "_link");
        
        this->declare_parameter<double>(prefix + "pos_x", 0.0);
        this->declare_parameter<double>(prefix + "pos_y", 0.0);
        this->declare_parameter<double>(prefix + "pos_z", 0.0);
        this->declare_parameter<double>(prefix + "roll", 0.0);
        this->declare_parameter<double>(prefix + "pitch", 0.0);
        this->declare_parameter<double>(prefix + "yaw", 0.0);
    }

    // 2. 공통 설정(JSON) 로드
    if (!loadCommonConfig()) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to load JSON config. Shutting down.");
        return; 
    }

    // 3. 개별 레이더 설정 로드 및 연결
    initRadars();

    setupPublishers();
    setupTF();
    setupSyncTimer();
    
    RCLCPP_INFO(this->get_logger(), "=======================================");
    RCLCPP_INFO(this->get_logger(), "   Multi-Radar Manager (Synced 10Hz)   ");
    RCLCPP_INFO(this->get_logger(), "   Active Radars: %zu / 4             ", drivers_.size());
    RCLCPP_INFO(this->get_logger(), "=======================================");
}

MultiRadarManager::~MultiRadarManager()
{
    stopAll();
}

bool MultiRadarManager::loadCommonConfig()
{
    std::string config_path = this->get_parameter("config_file").as_string();
    
    if (config_path.empty()) {
        try {
            config_path = ament_index_cpp::get_package_share_directory("nlos_awr2944") 
                        + "/config/radar_config.json";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not find package path: %s", e.what());
            return false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "📂 Loading common config from: %s", config_path.c_str());

    std::ifstream file(config_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "❌ File open error!");
        return false;
    }

    try {
        json j;
        file >> j;
        if (j.contains("radar_commands") && j["radar_commands"].is_array()) {
            common_commands_.clear();
            for (const auto& cmd : j["radar_commands"]) {
                if (cmd.is_string()) {
                    common_commands_.push_back(cmd.get<std::string>());
                }
            }
            RCLCPP_INFO(this->get_logger(), "✅ Loaded %zu commands.", common_commands_.size());
            return true;
        }
    } catch (const json::parse_error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ JSON Parse Error: %s", e.what());
    }
    return false;
}

void MultiRadarManager::initRadars()
{
    for (size_t i = 0; i < MAX_RADARS; ++i) {
        std::string prefix = "radar_" + std::to_string(i) + ".";
        
        if (!this->get_parameter(prefix + "enabled").as_bool()) {
            continue;
        }

        RadarConfig config;
        config.radar_id = "radar_" + std::to_string(i);
        config.frame_id = this->get_parameter(prefix + "frame_id").as_string();
        config.serial.cli_port = this->get_parameter(prefix + "cli_port").as_string();
        config.serial.data_port = this->get_parameter(prefix + "data_port").as_string();
        config.serial.cli_baudrate = 115200;
        config.serial.data_baudrate = 3125000;
        config.serial.timeout = 0.05;
        
        config.pos_x = this->get_parameter(prefix + "pos_x").as_double();
        config.pos_y = this->get_parameter(prefix + "pos_y").as_double();
        config.pos_z = this->get_parameter(prefix + "pos_z").as_double();
        config.roll = this->get_parameter(prefix + "roll").as_double();
        config.pitch = this->get_parameter(prefix + "pitch").as_double();
        config.yaw = this->get_parameter(prefix + "yaw").as_double();

        config.commands = common_commands_;
        
        auto driver = std::make_unique<RadarDriver>(config);
        if (driver->connect()) {
            if (driver->sendConfiguration()) {
                std::string radar_id = config.radar_id;
                driver->setFrameCallback(
                    [this, radar_id](const RadarFrame& frame) {
                        this->onRadarFrame(radar_id, frame);
                    });
                
                driver->startReceiving();
                
                drivers_[radar_id] = std::move(driver);
                radar_configs_[radar_id] = config;
                
                RCLCPP_INFO(this->get_logger(), "🚀 %s Started (Port: %s)", 
                    radar_id.c_str(), config.serial.data_port.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Config failed: %s", config.radar_id.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Connect failed: %s", config.radar_id.c_str());
        }
    }
}

void MultiRadarManager::setupPublishers()
{
    // 1. 통합 포인트 클라우드 퍼블리셔
    pub_merged_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar/merged/points", 10);

    // 2. [추가] 개별 레이더 퍼블리셔들 생성 (Range, Noise, Heatmap, Timestamp 포함)
    for (const auto& [id, config] : radar_configs_) {
        // Topic Names: radar_0/points, radar_0/range_profile, etc.
        pub_points_[id]        = create_publisher<sensor_msgs::msg::PointCloud2>(id + "/points", 10);
        pub_range_profile_[id] = create_publisher<std_msgs::msg::Float32MultiArray>(id + "/range_profile", 10);
        pub_noise_profile_[id] = create_publisher<std_msgs::msg::Float32MultiArray>(id + "/noise_profile", 10);
        pub_heatmap_[id]       = create_publisher<sensor_msgs::msg::Image>(id + "/heatmap/azimuth", 10);
        pub_timestamp_[id]     = create_publisher<std_msgs::msg::Header>(id + "/timestamp", 10);
        
        RCLCPP_INFO(this->get_logger(), "📢 Publishers created for: %s", id.c_str());
    }
}

void MultiRadarManager::setupTF()
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    
    for (const auto& [id, config] : radar_configs_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = config.frame_id;
        
        t.transform.translation.x = config.pos_x;
        t.transform.translation.y = config.pos_y;
        t.transform.translation.z = config.pos_z;
        
        tf2::Quaternion q;
        q.setRPY(config.roll, config.pitch, config.yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        transforms.push_back(t);
    }
    
    if (!transforms.empty()) {
        static_tf_broadcaster_->sendTransform(transforms);
    }
}

void MultiRadarManager::setupSyncTimer()
{
    double rate = this->get_parameter("target_freq_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    
    sync_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MultiRadarManager::syncTimerCallback, this));
}

void MultiRadarManager::onRadarFrame(const std::string& radar_id, const RadarFrame& frame)
{
    // 개별 발행은 syncTimerCallback에서 동기화 후 일괄 처리
    std::lock_guard<std::mutex> lock(queue_mutex_);
    frame_queues_[radar_id].push_back(frame);
    
    if (frame_queues_[radar_id].size() > 20) {
        frame_queues_[radar_id].pop_front();
    }
}

void MultiRadarManager::syncTimerCallback()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (drivers_.empty()) return;

    for (const auto& [id, driver] : drivers_) {
        if (frame_queues_[id].empty()) return;
    }

    std::string base_id = drivers_.begin()->first;
    const auto& base_frame = frame_queues_[base_id].back(); 
    rclcpp::Time base_time = base_frame.timestamp;
    
    SyncedFrame synced_set;
    synced_set.complete = true;
    double tolerance_ns = this->get_parameter("sync_tolerance_ms").as_double() * 1e6;

    for (const auto& [id, driver] : drivers_) {
        auto& queue = frame_queues_[id];
        
        double min_diff = 1e15;
        auto best_it = queue.end();
        
        for (auto it = queue.rbegin(); it != queue.rend(); ++it) {
            double diff = std::abs((it->timestamp - base_time).nanoseconds());
            
            if (diff < min_diff) {
                min_diff = diff;
                best_it = it.base() - 1;
            } else {
                break; 
            }
        }

        if (min_diff <= tolerance_ns && best_it != queue.end()) {
            synced_set.radar_frames[id] = *best_it;
            queue.erase(queue.begin(), best_it + 1);
        } else {
            synced_set.complete = false;
            return; 
        }
    }

    if (synced_set.complete) {
        // 1. 동기화된 프레임들을 개별 토픽으로 발행 (10Hz 샘플링)
        for (const auto& [id, frame] : synced_set.radar_frames) {
            publishIndividualFrame(id, frame);
        }

        // 2. 통합 포인트 클라우드 발행
        publishMergedPointcloud(synced_set);
    }
}

void MultiRadarManager::publishMergedPointcloud(const SyncedFrame& synced)
{
    sensor_msgs::msg::PointCloud2 merged_msg;
    merged_msg.header.stamp = this->now();
    merged_msg.header.frame_id = this->get_parameter("merged_frame_id").as_string();
    
    std::vector<RadarPoint> all_points;
    
    for (const auto& [id, frame] : synced.radar_frames) {
        const auto& config = radar_configs_[id];
        
        double cr = std::cos(config.roll), sr = std::sin(config.roll);
        double cp = std::cos(config.pitch), sp = std::sin(config.pitch);
        double cy = std::cos(config.yaw), sy = std::sin(config.yaw);

        double r11 = cy * cp;
        double r12 = cy * sp * sr - sy * cr;
        double r13 = cy * sp * cr + sy * sr;
        
        double r21 = sy * cp;
        double r22 = sy * sp * sr + cy * cr;
        double r23 = sy * sp * cr - cy * sr;
        
        double r31 = -sp;
        double r32 = cp * sr;
        double r33 = cp * cr;

        for (const auto& pt : frame.points) {
            RadarPoint tf_pt;
            
            double x_rot = r11 * pt.x + r12 * pt.y + r13 * pt.z;
            double y_rot = r21 * pt.x + r22 * pt.y + r23 * pt.z;
            double z_rot = r31 * pt.x + r32 * pt.y + r33 * pt.z;
            
            tf_pt.x = static_cast<float>(x_rot + config.pos_x);
            tf_pt.y = static_cast<float>(y_rot + config.pos_y);
            tf_pt.z = static_cast<float>(z_rot + config.pos_z);
            
            tf_pt.velocity = pt.velocity;
            tf_pt.intensity = pt.intensity;
            
            all_points.push_back(tf_pt);
        }
    }
    
    sensor_msgs::msg::PointCloud2 msg;
    msg.header = merged_msg.header;
    msg.height = 1;
    msg.width = all_points.size();
    
    msg.fields.resize(5);
    msg.fields[0].name = "x"; msg.fields[0].offset = 0; msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[0].count = 1;
    msg.fields[1].name = "y"; msg.fields[1].offset = 4; msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[1].count = 1;
    msg.fields[2].name = "z"; msg.fields[2].offset = 8; msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[2].count = 1;
    msg.fields[3].name = "velocity"; msg.fields[3].offset = 12; msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[3].count = 1;
    msg.fields[4].name = "intensity"; msg.fields[4].offset = 16; msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[4].count = 1;
    
    msg.point_step = 20;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);
    msg.is_dense = true;
    
    uint8_t* ptr = msg.data.data();
    for(const auto& p : all_points) {
        memcpy(ptr, &p.x, 4); memcpy(ptr+4, &p.y, 4); memcpy(ptr+8, &p.z, 4);
        memcpy(ptr+12, &p.velocity, 4); memcpy(ptr+16, &p.intensity, 4);
        ptr += 20;
    }
    
    pub_merged_points_->publish(msg);
}

// [수정됨] 개별 프레임 전체 데이터 발행 (포인트, 히트맵, 프로파일)
void MultiRadarManager::publishIndividualFrame(const std::string& radar_id, const RadarFrame& frame)
{
    // 퍼블리셔가 존재하는지 확인
    if (pub_points_.find(radar_id) == pub_points_.end()) return;

    rclcpp::Time stamp = this->now(); // 동기화된 현재 시간 사용

    // 1. 타임스탬프 발행
    std_msgs::msg::Header header_msg;
    header_msg.stamp = stamp;
    header_msg.frame_id = radar_configs_[radar_id].frame_id;
    pub_timestamp_[radar_id]->publish(header_msg);

    // 2. 포인트 클라우드 발행
    sensor_msgs::msg::PointCloud2 pc_msg;
    pc_msg.header = header_msg;
    pc_msg.height = 1;
    pc_msg.width = frame.points.size();
    pc_msg.is_dense = true;

    pc_msg.fields.resize(5);
    pc_msg.fields[0].name = "x"; pc_msg.fields[0].offset = 0; pc_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; pc_msg.fields[0].count = 1;
    pc_msg.fields[1].name = "y"; pc_msg.fields[1].offset = 4; pc_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; pc_msg.fields[1].count = 1;
    pc_msg.fields[2].name = "z"; pc_msg.fields[2].offset = 8; pc_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; pc_msg.fields[2].count = 1;
    pc_msg.fields[3].name = "velocity"; pc_msg.fields[3].offset = 12; pc_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; pc_msg.fields[3].count = 1;
    pc_msg.fields[4].name = "intensity"; pc_msg.fields[4].offset = 16; pc_msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32; pc_msg.fields[4].count = 1;

    pc_msg.point_step = 20;
    pc_msg.row_step = pc_msg.point_step * pc_msg.width;
    pc_msg.data.resize(pc_msg.row_step);

    uint8_t* ptr = pc_msg.data.data();
    for (const auto& p : frame.points) {
        memcpy(ptr, &p.x, 4); 
        memcpy(ptr+4, &p.y, 4); 
        memcpy(ptr+8, &p.z, 4);
        memcpy(ptr+12, &p.velocity, 4); 
        memcpy(ptr+16, &p.intensity, 4);
        ptr += 20;
    }
    pub_points_[radar_id]->publish(pc_msg);

    // 3. Range Profile 발행
    if (!frame.range_profile.empty()) {
        std_msgs::msg::Float32MultiArray range_msg;
        range_msg.data.reserve(frame.range_profile.size());
        for (auto val : frame.range_profile) {
            range_msg.data.push_back(static_cast<float>(val));
        }
        pub_range_profile_[radar_id]->publish(range_msg);
    }

    // 4. Noise Profile 발행
    if (!frame.noise_profile.empty()) {
        std_msgs::msg::Float32MultiArray noise_msg;
        noise_msg.data.reserve(frame.noise_profile.size());
        for (auto val : frame.noise_profile) {
            noise_msg.data.push_back(static_cast<float>(val));
        }
        pub_noise_profile_[radar_id]->publish(noise_msg);
    }

    // 5. Heatmap 발행
    if (!frame.azimuth_heatmap.empty()) {
        sensor_msgs::msg::Image heatmap_msg;
        heatmap_msg.header = header_msg;
        heatmap_msg.height = frame.heatmap_height;
        heatmap_msg.width = frame.heatmap_width;
        heatmap_msg.encoding = "mono8";
        heatmap_msg.step = frame.heatmap_width;
        
        float max_val = 0.0f;
        for (float val : frame.azimuth_heatmap) {
            if (val > max_val) max_val = val;
        }

        heatmap_msg.data.reserve(frame.azimuth_heatmap.size());
        for (auto val : frame.azimuth_heatmap) {
            uint8_t normalized = (max_val > 0) ? 
                static_cast<uint8_t>(val / max_val * 255.0f) : 0;
            heatmap_msg.data.push_back(normalized);
        }
        pub_heatmap_[radar_id]->publish(heatmap_msg);
    }
}

// 필요 시 외부 호출용 함수 구현
bool MultiRadarManager::startAll() { return true; }
void MultiRadarManager::stopAll() {
    for (auto& [id, driver] : drivers_) {
        if (driver) {
            driver->stopReceiving();
            driver->stopSensor();
            driver->disconnect();
        }
    }
}
size_t MultiRadarManager::getActiveRadarCount() const { return drivers_.size(); }

} // namespace nlos_awr2944

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<nlos_awr2944::MultiRadarManager>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}