#ifndef NLOS_AWR2944__MULTI_RADAR_MANAGER_HPP_
#define NLOS_AWR2944__MULTI_RADAR_MANAGER_HPP_

#include "nlos_awr2944/radar_driver_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>                // [추가] 히트맵용
#include <std_msgs/msg/float32_multi_array.hpp>     // [추가] 프로파일용
#include <std_msgs/msg/header.hpp>                  // [추가] 타임스탬프용
#include <tf2_ros/static_transform_broadcaster.h>

#include <map>
#include <deque>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

namespace nlos_awr2944
{

// ===========================================
// 동기화된 프레임 구조체
// ===========================================
struct SyncedFrame {
    bool complete = false;
    std::map<std::string, RadarFrame> radar_frames;
};

// ===========================================
// 멀티 레이더 매니저 클래스
// ===========================================
class MultiRadarManager : public rclcpp::Node
{
public:
    static constexpr size_t MAX_RADARS = 4;

    explicit MultiRadarManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MultiRadarManager();

    // 외부 제어 함수
    bool startAll();
    void stopAll();
    size_t getActiveRadarCount() const;

private:
    bool loadCommonConfig();
    void initRadars();
    
    void setupPublishers();
    void setupTF();
    void setupSyncTimer();
    
    // 콜백 함수
    void onRadarFrame(const std::string& radar_id, const RadarFrame& frame);
    void syncTimerCallback();
    
    // 데이터 처리 함수 (통합 발행)
    void publishMergedPointcloud(const SyncedFrame& synced);

    // [수정] 개별 데이터(포인트, 히트맵, 프로파일 등) 모두 발행
    void publishIndividualFrame(const std::string& radar_id, const RadarFrame& frame);

    // ===========================================
    // 멤버 변수
    // ===========================================
    
    std::vector<std::string> common_commands_;

    std::map<std::string, RadarConfig> radar_configs_; 
    
    std::map<std::string, std::unique_ptr<RadarDriver>> drivers_;
    
    std::map<std::string, std::deque<RadarFrame>> frame_queues_;
    std::mutex queue_mutex_;
    
    // ROS 통신 (통합)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_points_;

    // ROS 통신 (개별) - 각 레이더 ID별로 관리
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_points_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> pub_range_profile_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> pub_noise_profile_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_heatmap_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr> pub_timestamp_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
};

}  // namespace nlos_awr2944

#endif  // NLOS_AWR2944__MULTI_RADAR_MANAGER_HPP_