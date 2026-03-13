#include "nlos_awr2944/radar_driver_core.hpp"

#include <sys/stat.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>

namespace nlos_awr2944
{

DataLogger::DataLogger(const std::string& base_dir, rclcpp::Node* node)
    : node_(node)
{
    // 타임스탬프로 폴더 생성
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    
    std::ostringstream oss;
    oss << std::put_time(tm_now, "%Y%m%d_%H%M%S");
    std::string timestamp_str = oss.str();
    
    // 기본 경로 확장
    std::string expanded_base = base_dir;
    if (base_dir[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) {
            expanded_base = std::string(home) + base_dir.substr(1);
        }
    }
    
    save_dir_ = expanded_base + "/" + timestamp_str;
    others_dir_ = save_dir_ + "/others";
    
    // 디렉토리 생성
    mkdir(expanded_base.c_str(), 0755);
    mkdir(save_dir_.c_str(), 0755);
    mkdir(others_dir_.c_str(), 0755);
    
    RCLCPP_INFO(node_->get_logger(), "📂 Data saving to: %s", save_dir_.c_str());
    
    initCSVFiles();
}

DataLogger::~DataLogger()
{
    stopRosbag();
    
    if (pc_file_.is_open()) pc_file_.close();
    if (range_file_.is_open()) range_file_.close();
    if (noise_file_.is_open()) noise_file_.close();
    if (heatmap_file_.is_open()) heatmap_file_.close();
    
    RCLCPP_INFO(node_->get_logger(), "💾 CSV files saved");
}

void DataLogger::initCSVFiles()
{
    // PointCloud CSV
    std::string pc_path = save_dir_ + "/pointcloud.csv";
    pc_file_.open(pc_path);
    if (pc_file_.is_open()) {
        pc_file_ << "Frame,Rel_Time(s),Unix_Time(s),X(m),Y(m),Z(m),V(m/s),RCS(dB)\n";
    }
    
    // Range Profile CSV
    std::string range_path = others_dir_ + "/range_profile.csv";
    range_file_.open(range_path);
    // 헤더는 첫 데이터에서 크기를 알 때 작성
    
    // Noise Profile CSV
    std::string noise_path = others_dir_ + "/noise_profile.csv";
    noise_file_.open(noise_path);
    // 헤더는 첫 데이터에서 크기를 알 때 작성
    
    // Heatmap CSV
    std::string hm_path = others_dir_ + "/azimuth_heatmap.csv";
    heatmap_file_.open(hm_path);
    if (heatmap_file_.is_open()) {
        heatmap_file_ << "Frame,Rel_Time(s),Width,Height,Data_Array\n";
    }
    
    csv_initialized_ = true;
}

void DataLogger::logPointCloud(const RadarFrame& frame, double rel_time, double unix_time)
{
    if (!pc_file_.is_open()) return;
    
    for (const auto& pt : frame.points) {
        pc_file_ << frame.frame_number << ","
                 << std::fixed << std::setprecision(3) << rel_time << ","
                 << std::fixed << std::setprecision(3) << unix_time << ","
                 << pt.x << "," << pt.y << "," << pt.z << ","
                 << pt.velocity << "," << pt.intensity << "\n";
    }
}

void DataLogger::logRangeProfile(uint32_t frame_num, double rel_time, const std::vector<uint16_t>& data)
{
    if (!range_file_.is_open() || data.empty()) return;
    
    // 첫 데이터일 때 헤더 작성
    static bool header_written = false;
    if (!header_written) {
        range_file_ << "Frame,Rel_Time(s)";
        for (size_t i = 0; i < data.size(); ++i) {
            range_file_ << ",bin_" << i;
        }
        range_file_ << "\n";
        header_written = true;
    }
    
    range_file_ << frame_num << "," << std::fixed << std::setprecision(3) << rel_time;
    for (const auto& val : data) {
        range_file_ << "," << val;
    }
    range_file_ << "\n";
}

void DataLogger::logNoiseProfile(uint32_t frame_num, double rel_time, const std::vector<uint16_t>& data)
{
    if (!noise_file_.is_open() || data.empty()) return;
    
    // 첫 데이터일 때 헤더 작성
    static bool header_written = false;
    if (!header_written) {
        noise_file_ << "Frame,Rel_Time(s)";
        for (size_t i = 0; i < data.size(); ++i) {
            noise_file_ << ",bin_" << i;
        }
        noise_file_ << "\n";
        header_written = true;
    }
    
    noise_file_ << frame_num << "," << std::fixed << std::setprecision(3) << rel_time;
    for (const auto& val : data) {
        noise_file_ << "," << val;
    }
    noise_file_ << "\n";
}

void DataLogger::logHeatmap(uint32_t frame_num, double rel_time, uint32_t width, uint32_t height,
                            const std::vector<float>& data)
{
    if (!heatmap_file_.is_open() || data.empty()) return;
    
    heatmap_file_ << frame_num << "," 
                  << std::fixed << std::setprecision(3) << rel_time << ","
                  << width << "," << height;
    
    for (const auto& val : data) {
        heatmap_file_ << "," << val;
    }
    heatmap_file_ << "\n";
}

void DataLogger::startRosbag(const std::vector<std::string>& topics)
{
    std::string bag_path = save_dir_ + "/rosbag";
    
    pid_t pid = fork();
    if (pid == 0) {
        // 자식 프로세스: ros2 bag record 실행
        std::vector<const char*> args;
        args.push_back("ros2");
        args.push_back("bag");
        args.push_back("record");
        args.push_back("-o");
        args.push_back(bag_path.c_str());
        
        // 토픽 추가
        for (const auto& topic : topics) {
            args.push_back(topic.c_str());
        }
        
        // -a 대신 특정 토픽만 녹화 (너무 많은 데이터 방지)
        if (topics.empty()) {
            args.push_back("-a");  // 모든 토픽
        }
        
        args.push_back(nullptr);
        
        // stdout/stderr 리다이렉트
        int devnull = open("/dev/null", O_WRONLY);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);
        
        execvp("ros2", const_cast<char* const*>(args.data()));
        _exit(1);  // exec 실패 시
    } else if (pid > 0) {
        rosbag_pid_ = pid;
        RCLCPP_INFO(node_->get_logger(), "🔴 Rosbag recording started (PID: %d)", pid);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to start rosbag recording");
    }
}

void DataLogger::stopRosbag()
{
    if (rosbag_pid_ > 0) {
        // SIGINT 전송 (Ctrl+C와 동일)
        kill(rosbag_pid_, SIGINT);
        
        // 최대 5초 대기
        int status;
        int wait_count = 0;
        while (waitpid(rosbag_pid_, &status, WNOHANG) == 0 && wait_count < 50) {
            usleep(100000);  // 100ms
            wait_count++;
        }
        
        // 아직 안 끝났으면 강제 종료
        if (wait_count >= 50) {
            kill(rosbag_pid_, SIGKILL);
            waitpid(rosbag_pid_, &status, 0);
        }
        
        RCLCPP_INFO(node_->get_logger(), "⏹️ Rosbag recording stopped");
        rosbag_pid_ = -1;
    }
}

}  // namespace nlos_awr2944
