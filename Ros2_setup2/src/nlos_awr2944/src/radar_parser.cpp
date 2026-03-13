#include "nlos_awr2944/radar_driver_core.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace nlos_awr2944
{

// ===========================================
// Magic Word 찾기
// ===========================================
int findMagicWord(const std::vector<uint8_t>& buffer, size_t start_pos)
{
    if (buffer.size() < start_pos + MAGIC_WORD_SIZE) {
        return -1;
    }

    for (size_t i = start_pos; i <= buffer.size() - MAGIC_WORD_SIZE; ++i) {
        if (std::memcmp(&buffer[i], MAGIC_WORD, MAGIC_WORD_SIZE) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

// ===========================================
// 헤더 파싱
// ===========================================
bool parseFrameHeader(const uint8_t* data, size_t len, FrameHeader& header)
{
    if (len < FRAME_HEADER_SIZE) {
        return false;
    }

    std::memcpy(&header, data, sizeof(FrameHeader));

    // Magic word 검증
    if (std::memcmp(header.magic_word, MAGIC_WORD, MAGIC_WORD_SIZE) != 0) {
        return false;
    }

    return true;
}

// ===========================================
// TLV 파싱
// ===========================================
bool parseTLV(const uint8_t* data, size_t len, TLVHeader& header, const uint8_t*& payload)
{
    if (len < TLV_HEADER_SIZE) {
        return false;
    }

    std::memcpy(&header, data, sizeof(TLVHeader));

    if (len < TLV_HEADER_SIZE + header.length) {
        return false;
    }

    payload = data + TLV_HEADER_SIZE;
    return true;
}

// ===========================================
// 포인트 클라우드 파싱 (TLV Type 1)
// ===========================================
std::vector<DetectedPoint> parseDetectedPoints(const uint8_t* data, size_t len)
{
    std::vector<DetectedPoint> points;
    
    constexpr size_t POINT_SIZE = sizeof(DetectedPoint);
    size_t num_points = len / POINT_SIZE;
    
    points.reserve(num_points);
    
    for (size_t i = 0; i < num_points; ++i) {
        DetectedPoint point;
        std::memcpy(&point, data + i * POINT_SIZE, POINT_SIZE);
        points.push_back(point);
    }
    
    return points;
}

// ===========================================
// 포인트 사이드 정보 파싱 (TLV Type 7)
// ===========================================
std::vector<PointSideInfo> parseSideInfo(const uint8_t* data, size_t len)
{
    std::vector<PointSideInfo> info;
    
    constexpr size_t INFO_SIZE = sizeof(PointSideInfo);
    size_t num_points = len / INFO_SIZE;
    
    info.reserve(num_points);
    
    for (size_t i = 0; i < num_points; ++i) {
        PointSideInfo side_info;
        std::memcpy(&side_info, data + i * INFO_SIZE, INFO_SIZE);
        info.push_back(side_info);
    }
    
    return info;
}

// ===========================================
// Range/Noise Profile 파싱 (TLV Type 2, 3)
// ===========================================
std::vector<uint16_t> parseProfile(const uint8_t* data, size_t len)
{
    std::vector<uint16_t> profile;
    
    size_t num_bins = len / sizeof(uint16_t);
    profile.reserve(num_bins);
    
    for (size_t i = 0; i < num_bins; ++i) {
        uint16_t value;
        std::memcpy(&value, data + i * sizeof(uint16_t), sizeof(uint16_t));
        profile.push_back(value);
    }
    
    return profile;
}

// ===========================================
// Azimuth Heatmap 파싱 (TLV Type 4)
// ===========================================
std::vector<float> parseAzimuthHeatmap(
    const uint8_t* data, 
    size_t len,
    uint32_t& out_width,
    uint32_t& out_height)
{
    // AWR2944: 4 Tx * 4 Rx = 16 virtual antennas (but using 12 typically)
    constexpr uint32_t NUM_VIRTUAL_ANT = 12;
    
    std::vector<float> magnitude;
    
    // 데이터: I/Q 쌍으로 구성 (각 16비트)
    size_t num_samples = len / sizeof(int32_t);  // I/Q 쌍
    size_t num_range_bins = num_samples / NUM_VIRTUAL_ANT;
    
    out_width = NUM_VIRTUAL_ANT;
    out_height = static_cast<uint32_t>(num_range_bins);
    
    magnitude.reserve(num_samples);
    
    const int16_t* iq_data = reinterpret_cast<const int16_t*>(data);
    
    for (size_t i = 0; i < num_samples; ++i) {
        float imag = static_cast<float>(iq_data[i * 2]);
        float real = static_cast<float>(iq_data[i * 2 + 1]);
        float mag = std::sqrt(real * real + imag * imag);
        magnitude.push_back(mag);
    }
    
    return magnitude;
}

// ===========================================
// 레이더 좌표 -> ROS 좌표 변환
// ===========================================
RadarPoint convertToROSCoordinates(
    const DetectedPoint& radar_point,
    const PointSideInfo* side_info,
    uint32_t frame_id)
{
    RadarPoint ros_point;
    
    // 레이더 좌표계 (X: 앞, Y: 왼쪽, Z: 위) -> ROS 좌표계
    // ROS: X = radar Y, Y = -radar X, Z = radar Z
    ros_point.x = radar_point.y;
    ros_point.y = -radar_point.x;
    ros_point.z = radar_point.z;
    ros_point.velocity = radar_point.velocity;
    ros_point.frame_id = frame_id;
    
    // RCS 계산
    if (side_info != nullptr) {
        float snr = static_cast<float>(side_info->snr);
        float range = std::sqrt(
            ros_point.x * ros_point.x + 
            ros_point.y * ros_point.y + 
            ros_point.z * ros_point.z);
        
        // RCS = SNR + 40 * log10(range)
        if (range > 0) {
            ros_point.intensity = snr + 40.0f * std::log10(range);
        } else {
            ros_point.intensity = snr;
        }
    } else {
        ros_point.intensity = 0.0f;
    }
    
    return ros_point;
}

// ===========================================
// 전체 프레임 파싱
// ===========================================
bool parseFullFrame(
    const std::vector<uint8_t>& data,
    RadarFrame& frame,
    rclcpp::Clock::SharedPtr clock)
{
    frame.valid = false;
    frame.points.clear();
    frame.range_profile.clear();
    frame.noise_profile.clear();
    frame.azimuth_heatmap.clear();
    
    if (data.size() < FRAME_HEADER_SIZE) {
        return false;
    }
    
    // 헤더 파싱
    FrameHeader header;
    if (!parseFrameHeader(data.data(), data.size(), header)) {
        return false;
    }
    
    if (data.size() < header.total_packet_len) {
        return false;
    }
    
    frame.frame_number = header.frame_number;
    frame.timestamp = clock->now();
    
    // TLV 파싱
    size_t offset = FRAME_HEADER_SIZE;
    
    std::vector<DetectedPoint> detected_points;
    std::vector<PointSideInfo> side_info;
    
    for (uint32_t i = 0; i < header.num_tlvs && offset < data.size(); ++i) {
        TLVHeader tlv_header;
        const uint8_t* tlv_payload;
        
        if (!parseTLV(data.data() + offset, data.size() - offset, tlv_header, tlv_payload)) {
            break;
        }
        
        switch (static_cast<TLVType>(tlv_header.type)) {
            case TLVType::DETECTED_POINTS:
                detected_points = parseDetectedPoints(tlv_payload, tlv_header.length);
                break;
                
            case TLVType::RANGE_PROFILE:
                frame.range_profile = parseProfile(tlv_payload, tlv_header.length);
                break;
                
            case TLVType::NOISE_PROFILE:
                frame.noise_profile = parseProfile(tlv_payload, tlv_header.length);
                break;
                
            case TLVType::AZIMUTH_STATIC_HEATMAP:
                frame.azimuth_heatmap = parseAzimuthHeatmap(
                    tlv_payload, 
                    tlv_header.length,
                    frame.heatmap_width,
                    frame.heatmap_height);
                break;
                
            case TLVType::DETECTED_POINTS_SIDE_INFO:
                side_info = parseSideInfo(tlv_payload, tlv_header.length);
                break;
                
            default:
                // 알 수 없는 TLV 타입은 무시
                break;
        }
        
        offset += TLV_HEADER_SIZE + tlv_header.length;
    }
    
    // 포인트 변환
    frame.points.reserve(detected_points.size());
    for (size_t i = 0; i < detected_points.size(); ++i) {
        const PointSideInfo* info = (i < side_info.size()) ? &side_info[i] : nullptr;
        frame.points.push_back(
            convertToROSCoordinates(detected_points[i], info, frame.frame_number));
    }
    
    frame.valid = true;
    return true;
}

}  // namespace nlos_awr2944
