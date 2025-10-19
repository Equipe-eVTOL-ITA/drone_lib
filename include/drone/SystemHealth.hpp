#pragma once

#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/system_health.hpp>
#include <chrono>
#include <fstream>
#include <sys/statvfs.h>

class SystemHealth : public rclcpp::Node {
public:
    SystemHealth();

private:
    void publishSystemHealth();
    
    float getCpuUsage();
    float getMemoryUsage();
    float getCpuTemperature();
    float getDiskUsage();
    float getGpuUsage();
    
    // GPU-specific methods
    float getNvidiaGpuUsage();
    float getIntelGpuUsage();
    float getAmdGpuUsage();
    
    rclcpp::Publisher<custom_msgs::msg::SystemHealth>::SharedPtr system_health_pub_;
    rclcpp::TimerBase::SharedPtr health_timer_;
};
