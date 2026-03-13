#include "nlos_awr2944/radar_driver_core.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<nlos_awr2944::RadarNode>();
    
    RCLCPP_INFO(node->get_logger(), "=================================");
    RCLCPP_INFO(node->get_logger(), "   AWR2944 Radar Driver (C++)    ");
    RCLCPP_INFO(node->get_logger(), "   Multi-Sensor Sync Ready       ");
    RCLCPP_INFO(node->get_logger(), "=================================");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
