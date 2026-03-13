/**
 * ==============================================
 * NLOS Multi-Sensor Time Synchronizer v5
 * ==============================================
 * 
 * "레이더 트리거 + Hold" 방식 동기화
 * 
 * v4 대비 변경점:
 * - 데이터가 없으면 이전 프레임 데이터를 유지(hold)하여 프레임 드랍 방지
 * - consumed 플래그로 "새 데이터 vs 재사용" 구분
 * - 모든 센서가 매 프레임 동일한 프레임 수를 가짐
 * - 오디오 11.7Hz → 10Hz에서 빈 프레임은 이전 데이터 재사용
 * - 카메라/히트맵 타이밍 미스 시에도 이전 프레임 유지
 * 
 * 동기화 흐름:
 *   1. 각 센서 콜백 → latest 버퍼에 최신 데이터 저장 + consumed=false
 *   2. /radar/merged/points 수신 → onMergedTrigger()
 *   3. 모든 센서의 latest 데이터를 /sync/* 로 발행
 *   4. consumed=true로 마킹 (데이터는 클리어하지 않음!)
 *   5. 다음 트리거 시 새 데이터 없으면 이전 데이터 재사용
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <sstream>
#include <iomanip>
#include <map>
#include <string>

namespace nlos_fusion
{

template<typename T>
struct HoldData
{
    typename T::SharedPtr data;
    rclcpp::Time original_time;
    bool has_data{false};     // 한 번이라도 데이터를 받았는지
    bool consumed{true};      // 이미 발행했는지 (false=새 데이터)
    
    void update(typename T::SharedPtr msg, rclcpp::Time time) {
        data = msg;
        original_time = time;
        has_data = true;
        consumed = false;  // 새 데이터 도착
    }
    
    // 발행 후 consumed만 마킹 (데이터는 유지!)
    void markConsumed() {
        consumed = true;
    }
    
    // 새 데이터가 있는지
    bool isNew() const { return has_data && !consumed; }
    
    // 발행 가능한지 (새 데이터든 이전 데이터든)
    bool isAvailable() const { return has_data; }
};

struct RadarData
{
    HoldData<sensor_msgs::msg::PointCloud2> points;
    HoldData<sensor_msgs::msg::Image> heatmap;
    HoldData<std_msgs::msg::Float32MultiArray> range_profile;
    HoldData<std_msgs::msg::Float32MultiArray> noise_profile;
    
    void markAllConsumed() {
        points.markConsumed();
        heatmap.markConsumed();
        range_profile.markConsumed();
        noise_profile.markConsumed();
    }
};

struct RadarPublishers
{
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr heatmap;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr range_profile;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr noise_profile;
};

class MultiSensorSync : public rclcpp::Node
{
public:
    MultiSensorSync()
        : Node("multi_sensor_sync")
    {
        this->declare_parameter<int>("num_radars", 2);
        this->declare_parameter<bool>("enable_lidar", false);
        this->declare_parameter<bool>("enable_camera", true);
        this->declare_parameter<bool>("enable_audio", true);
        
        num_radars_ = this->get_parameter("num_radars").as_int();
        enable_lidar_ = this->get_parameter("enable_lidar").as_bool();
        enable_camera_ = this->get_parameter("enable_camera").as_bool();
        enable_audio_ = this->get_parameter("enable_audio").as_bool();
        
        setupSubscribers();
        setupPublishers();
        
        printStartupBanner();
    }

private:
    void printStartupBanner()
    {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║   Multi-Sensor Synchronizer v5 (Radar-Triggered + Hold)     ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  Trigger: /radar/merged/points (~10Hz from radar manager)   ║");
        RCLCPP_INFO(this->get_logger(), "║  Radars: %d  Lidar[%s] Camera[%s] Audio[%s]                 ║",
            num_radars_,
            enable_lidar_ ? "ON" : "--", 
            enable_camera_ ? "ON" : "--",
            enable_audio_ ? "ON" : "--");
        RCLCPP_INFO(this->get_logger(), "║  Mode: Hold (no frame drops - reuse previous if no new)     ║");
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════════════╝");
    }

    void setupSubscribers()
    {
        auto qos = rclcpp::SensorDataQoS();
        
        // ========== 멀티 레이더 개별 ==========
        for (int i = 0; i < num_radars_; ++i) {
            std::string id = "radar_" + std::to_string(i);
            radar_data_[id] = RadarData{};
            
            auto sub_pts = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/" + id + "/points", qos,
                [this, id](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    radar_data_[id].points.update(msg, rclcpp::Time(msg->header.stamp));
                });
            radar_subs_points_.push_back(sub_pts);
            
            auto sub_hm = this->create_subscription<sensor_msgs::msg::Image>(
                "/" + id + "/heatmap/azimuth", qos,
                [this, id](sensor_msgs::msg::Image::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    radar_data_[id].heatmap.update(msg, rclcpp::Time(msg->header.stamp));
                });
            radar_subs_heatmap_.push_back(sub_hm);
            
            auto sub_rp = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/" + id + "/range_profile", qos,
                [this, id](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    radar_data_[id].range_profile.update(msg, this->now());
                });
            radar_subs_range_.push_back(sub_rp);
            
            auto sub_np = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/" + id + "/noise_profile", qos,
                [this, id](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    radar_data_[id].noise_profile.update(msg, this->now());
                });
            radar_subs_noise_.push_back(sub_np);
        }
        
        // ========== merged = 트리거 ==========
        sub_radar_merged_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/radar/merged/points", qos,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->onMergedTrigger(msg);
            });
        
        // ========== 라이다 ==========
        if (enable_lidar_) {
            sub_lidar_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ouster/points", qos,
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_lidar_points_.update(msg, rclcpp::Time(msg->header.stamp));
                });
            sub_lidar_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/ouster/imu", qos,
                [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_lidar_imu_.update(msg, rclcpp::Time(msg->header.stamp));
                });
        }
        
        // ========== 카메라 ==========
        if (enable_camera_) {
            sub_camera_color_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/camera/color/image_raw", qos,
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_camera_color_.update(msg, rclcpp::Time(msg->header.stamp));
                });
        }
        
        // ========== 오디오 ==========
        if (enable_audio_) {
            sub_audio_spectrum_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/audio/spectrum", qos,
                [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_audio_spectrum_.update(msg, this->now());
                });
            sub_audio_direction_ = this->create_subscription<std_msgs::msg::Float32>(
                "/audio/direction", qos,
                [this](std_msgs::msg::Float32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_audio_direction_.update(msg, this->now());
                });
        }
    }
    
    void setupPublishers()
    {
        // 모든 /sync/ 퍼블리셔를 reliable로 통일 (rosbag reliable 구독과 매칭)
        auto reliable_qos = rclcpp::QoS(30).reliable();
        
        pub_sync_timestamp_ = this->create_publisher<std_msgs::msg::Header>("/sync/timestamp", reliable_qos);
        pub_frame_info_ = this->create_publisher<std_msgs::msg::String>("/sync/frame_info", reliable_qos);
        
        for (int i = 0; i < num_radars_; ++i) {
            std::string id = "radar_" + std::to_string(i);
            RadarPublishers pubs;
            pubs.points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/" + id + "/points", reliable_qos);
            pubs.heatmap = this->create_publisher<sensor_msgs::msg::Image>("/sync/" + id + "/heatmap", reliable_qos);
            pubs.range_profile = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sync/" + id + "/range_profile", reliable_qos);
            pubs.noise_profile = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sync/" + id + "/noise_profile", reliable_qos);
            radar_pubs_[id] = pubs;
        }
        pub_radar_merged_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/radar/merged/points", reliable_qos);
        
        if (enable_lidar_) {
            pub_lidar_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/lidar/points", reliable_qos);
            pub_lidar_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/sync/lidar/imu", reliable_qos);
        }
        if (enable_camera_) {
            pub_camera_color_ = this->create_publisher<sensor_msgs::msg::Image>("/sync/camera/color", reliable_qos);
        }
        if (enable_audio_) {
            pub_audio_spectrum_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sync/audio/spectrum", reliable_qos);
            pub_audio_direction_ = this->create_publisher<std_msgs::msg::Float32>("/sync/audio/direction", reliable_qos);
        }
    }
    
    void onMergedTrigger(sensor_msgs::msg::PointCloud2::SharedPtr merged_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        frame_count_++;
        rclcpp::Time sync_time = rclcpp::Time(merged_msg->header.stamp);
        
        int sensors_available = 0;  // 새 데이터가 있는 센서 수
        int sensors_held = 0;       // 이전 데이터 재사용 센서 수
        double max_diff_ms = 0;
        
        std::map<std::string, double> radar_orig_times;
        std::map<std::string, double> radar_diff_ms_map;
        double camera_orig = 0, camera_diff = 0;
        double audio_orig = 0, audio_diff = 0;
        double lidar_orig = 0, lidar_diff = 0;
        bool camera_held = false, audio_held = false;
        
        // ========== 동기화 타임스탬프 ==========
        std_msgs::msg::Header sync_header;
        sync_header.stamp = sync_time;
        sync_header.frame_id = "sync_frame_" + std::to_string(frame_count_);
        pub_sync_timestamp_->publish(sync_header);
        
        // ========== Merged (트리거 자체 - 항상 새 데이터) ==========
        auto merged_out = *merged_msg;
        merged_out.header.stamp = sync_time;
        pub_radar_merged_->publish(merged_out);
        sensors_available++;
        
        // ========== 개별 레이더 ==========
        for (auto& [id, rdata] : radar_data_) {
            radar_orig_times[id] = 0;
            radar_diff_ms_map[id] = 0;
            
            // points: 새 데이터든 이전 데이터든 발행
            if (rdata.points.isAvailable()) {
                radar_orig_times[id] = rdata.points.original_time.seconds();
                double diff = (sync_time - rdata.points.original_time).seconds() * 1000.0;
                radar_diff_ms_map[id] = diff;
                max_diff_ms = std::max(max_diff_ms, std::abs(diff));
                
                auto pts_msg = *rdata.points.data;
                pts_msg.header.stamp = sync_time;
                radar_pubs_[id].points->publish(pts_msg);
                
                // heatmap (있으면 발행 - 새 데이터든 이전이든)
                if (rdata.heatmap.isAvailable()) {
                    auto hm_msg = *rdata.heatmap.data;
                    hm_msg.header.stamp = sync_time;
                    radar_pubs_[id].heatmap->publish(hm_msg);
                }
                
                // range_profile
                if (rdata.range_profile.isAvailable()) {
                    radar_pubs_[id].range_profile->publish(*rdata.range_profile.data);
                }
                
                // noise_profile
                if (rdata.noise_profile.isAvailable()) {
                    radar_pubs_[id].noise_profile->publish(*rdata.noise_profile.data);
                }
            }
            rdata.markAllConsumed();  // 클리어가 아닌 consumed 마킹!
        }
        
        // ========== 라이다 ==========
        if (enable_lidar_ && latest_lidar_points_.isAvailable()) {
            lidar_orig = latest_lidar_points_.original_time.seconds();
            lidar_diff = (sync_time - latest_lidar_points_.original_time).seconds() * 1000.0;
            max_diff_ms = std::max(max_diff_ms, std::abs(lidar_diff));
            
            if (latest_lidar_points_.isNew()) sensors_available++;
            else sensors_held++;
            
            auto pts_msg = *latest_lidar_points_.data;
            pts_msg.header.stamp = sync_time;
            pub_lidar_points_->publish(pts_msg);
            latest_lidar_points_.markConsumed();
            
            if (latest_lidar_imu_.isAvailable()) {
                auto imu_msg = *latest_lidar_imu_.data;
                imu_msg.header.stamp = sync_time;
                pub_lidar_imu_->publish(imu_msg);
                latest_lidar_imu_.markConsumed();
            }
        }
        
        // ========== 카메라 ==========
        if (enable_camera_ && latest_camera_color_.isAvailable()) {
            camera_orig = latest_camera_color_.original_time.seconds();
            camera_diff = (sync_time - latest_camera_color_.original_time).seconds() * 1000.0;
            max_diff_ms = std::max(max_diff_ms, std::abs(camera_diff));
            
            camera_held = !latest_camera_color_.isNew();
            if (!camera_held) sensors_available++;
            else sensors_held++;
            
            auto color_msg = *latest_camera_color_.data;
            color_msg.header.stamp = sync_time;
            pub_camera_color_->publish(color_msg);
            latest_camera_color_.markConsumed();
        }
        
        // ========== 오디오 ==========
        if (enable_audio_ && latest_audio_spectrum_.isAvailable()) {
            audio_orig = latest_audio_spectrum_.original_time.seconds();
            audio_diff = (sync_time - latest_audio_spectrum_.original_time).seconds() * 1000.0;
            max_diff_ms = std::max(max_diff_ms, std::abs(audio_diff));
            
            audio_held = !latest_audio_spectrum_.isNew();
            if (!audio_held) sensors_available++;
            else sensors_held++;
            
            pub_audio_spectrum_->publish(*latest_audio_spectrum_.data);
            latest_audio_spectrum_.markConsumed();
            
            if (latest_audio_direction_.isAvailable()) {
                pub_audio_direction_->publish(*latest_audio_direction_.data);
                latest_audio_direction_.markConsumed();
            }
        }
        
        // ========== 프레임 정보 CSV ==========
        // frame,sync_time,sensors_new,sensors_held,max_diff,
        //   radar_0_orig,radar_0_diff, ...,
        //   camera_orig,camera_diff,camera_held,
        //   audio_orig,audio_diff,audio_held,
        //   lidar_orig,lidar_diff
        std::stringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << frame_count_ << ",";
        ss << sync_time.seconds() << ",";
        ss << sensors_available << ",";
        ss << sensors_held << ",";
        ss << std::setprecision(2) << max_diff_ms;
        
        for (int i = 0; i < num_radars_; ++i) {
            std::string id = "radar_" + std::to_string(i);
            ss << "," << std::setprecision(6) << radar_orig_times[id];
            ss << "," << std::setprecision(2) << radar_diff_ms_map[id];
        }
        
        ss << "," << std::setprecision(6) << camera_orig;
        ss << "," << std::setprecision(2) << camera_diff;
        ss << "," << (camera_held ? 1 : 0);
        ss << "," << std::setprecision(6) << audio_orig;
        ss << "," << std::setprecision(2) << audio_diff;
        ss << "," << (audio_held ? 1 : 0);
        ss << "," << std::setprecision(6) << lidar_orig;
        ss << "," << std::setprecision(2) << lidar_diff;
        
        std_msgs::msg::String info_msg;
        info_msg.data = ss.str();
        pub_frame_info_->publish(info_msg);
        
        // 5초마다 상태 출력
        if (frame_count_ % 50 == 0) {
            std::string radar_status;
            for (int i = 0; i < num_radars_; ++i) {
                std::string id = "radar_" + std::to_string(i);
                char buf[64];
                snprintf(buf, sizeof(buf), " R%d:%.1f", i, radar_diff_ms_map[id]);
                radar_status += buf;
            }
            RCLCPP_INFO(this->get_logger(), 
                "📡 Frame %lu | New:%d Hold:%d | MaxDiff:%.1fms |%s C:%.1f%s A:%.1f%s L:%.1f ms",
                frame_count_, sensors_available, sensors_held, max_diff_ms,
                radar_status.c_str(),
                camera_diff, camera_held ? "(H)" : "",
                audio_diff, audio_held ? "(H)" : "",
                lidar_diff);
        }
    }
    
    int num_radars_;
    bool enable_lidar_, enable_camera_, enable_audio_;
    uint64_t frame_count_{0};
    std::mutex mutex_;
    
    std::map<std::string, RadarData> radar_data_;
    
    HoldData<sensor_msgs::msg::PointCloud2> latest_lidar_points_;
    HoldData<sensor_msgs::msg::Imu> latest_lidar_imu_;
    HoldData<sensor_msgs::msg::Image> latest_camera_color_;
    HoldData<std_msgs::msg::Float32MultiArray> latest_audio_spectrum_;
    HoldData<std_msgs::msg::Float32> latest_audio_direction_;
    
    // 구독자
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> radar_subs_points_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> radar_subs_heatmap_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> radar_subs_range_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> radar_subs_noise_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_radar_merged_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_lidar_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_color_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_audio_spectrum_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_audio_direction_;
    
    // 퍼블리셔
    std::map<std::string, RadarPublishers> radar_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_radar_merged_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_points_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_lidar_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_color_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_audio_spectrum_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_audio_direction_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_sync_timestamp_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_frame_info_;
};

}  // namespace nlos_fusion

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nlos_fusion::MultiSensorSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}