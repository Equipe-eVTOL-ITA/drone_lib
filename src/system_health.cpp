#include "drone/SystemHealth.hpp"
#include <sstream>
#include <cstdio>
#include <chrono>
#include <algorithm>

SystemHealth::SystemHealth() : Node("system_health") {
    // Create publisher for system health telemetry
    system_health_pub_ = this->create_publisher<custom_msgs::msg::SystemHealth>("/telemetry/system_health", 10);
    
    // Create timer for 1Hz health monitoring
    health_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SystemHealth::publishSystemHealth, this));
    
    RCLCPP_INFO(this->get_logger(), "System health monitoring node initialized");
}

void SystemHealth::publishSystemHealth() {
    auto msg = custom_msgs::msg::SystemHealth();
    msg.header.stamp = this->get_clock()->now();
    
    // Get system metrics
    msg.cpu_percent = getCpuUsage();
    msg.memory_percent = getMemoryUsage();
    msg.temperature = getCpuTemperature();
    msg.disk_usage_percent = getDiskUsage();
    msg.gpu_percent = getGpuUsage();
    
    system_health_pub_->publish(msg);
}

float SystemHealth::getCpuUsage() {
    // Read CPU stats from /proc/stat for accurate CPU usage calculation
    static unsigned long long prev_idle = 0, prev_total = 0;
    
    std::ifstream file("/proc/stat");
    if (!file.is_open()) return 0.0f;
    
    std::string line;
    std::getline(file, line);
    file.close();
    
    // Parse CPU line: cpu user nice system idle iowait irq softirq steal guest guest_nice
    std::istringstream iss(line);
    std::string cpu_label;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
    
    iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;
    
    // Calculate total and idle time
    unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;
    unsigned long long current_idle = idle + iowait;
    
    // Calculate differences
    unsigned long long total_diff = total - prev_total;
    unsigned long long idle_diff = current_idle - prev_idle;
    
    float cpu_percent = 0.0f;
    if (total_diff > 0) {
        cpu_percent = 100.0f * (1.0f - (float)idle_diff / (float)total_diff);
    }
    
    // Update previous values for next calculation
    prev_total = total;
    prev_idle = current_idle;
    
    return std::max(0.0f, std::min(100.0f, cpu_percent));
}

float SystemHealth::getMemoryUsage() {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) return 0.0f;
    
    std::string line;
    float total = 0, available = 0;
    
    while (std::getline(file, line)) {
        if (line.find("MemTotal:") == 0) {
            sscanf(line.c_str(), "MemTotal: %f kB", &total);
        } else if (line.find("MemAvailable:") == 0) {
            sscanf(line.c_str(), "MemAvailable: %f kB", &available);
            break; // We have both values
        }
    }
    file.close();
    
    if (total > 0) {
        return ((total - available) / total) * 100.0f;
    }
    return 0.0f;
}

float SystemHealth::getCpuTemperature() {
    // Try multiple temperature sources in order of preference
    std::vector<std::string> temp_paths = {
        "/sys/class/thermal/thermal_zone0/temp",  // Common CPU thermal zone
        "/sys/class/thermal/thermal_zone1/temp",  // Alternative CPU thermal zone
        "/sys/class/hwmon/hwmon0/temp1_input",    // Hardware monitoring
        "/sys/class/hwmon/hwmon1/temp1_input",    // Alternative hwmon
        "/sys/class/hwmon/hwmon2/temp1_input"     // Another alternative
    };
    
    for (const auto& path : temp_paths) {
        std::ifstream file(path);
        if (file.is_open()) {
            float temp;
            if (file >> temp) {
                file.close();
                
                // Convert from millidegrees to degrees Celsius
                float celsius = temp / 1000.0f;
                
                // Sanity check: temperature should be between -40°C and 125°C
                if (celsius >= -40.0f && celsius <= 125.0f) {
                    return celsius;
                }
            }
        }
    }
    
    // If all thermal zones fail, try reading CPU package temperature
    std::ifstream coretemp("/sys/devices/platform/coretemp.0/hwmon/hwmon*/temp*_input");
    if (coretemp.is_open()) {
        float temp;
        if (coretemp >> temp) {
            float celsius = temp / 1000.0f;
            if (celsius >= -40.0f && celsius <= 125.0f) {
                return celsius;
            }
        }
    }
    
    return 0.0f; // No valid temperature found
}

float SystemHealth::getDiskUsage() {
    struct statvfs stat;
    if (statvfs("/", &stat) != 0) return 0.0f;
    
    unsigned long total = stat.f_blocks * stat.f_frsize;
    unsigned long free = stat.f_bavail * stat.f_frsize;
    
    if (total > 0) {
        return ((float)(total - free) / total) * 100.0f;
    }
    return 0.0f;
}

float SystemHealth::getGpuUsage() {
    // Try multiple methods to get GPU usage
    
    // Method 1: Try NVIDIA GPU via nvidia-smi
    float nvidia_usage = getNvidiaGpuUsage();
    if (nvidia_usage >= 0.0f) {
        return nvidia_usage;
    }
    
    // Method 2: Try Intel GPU via intel_gpu_top (common on Intel systems)
    float intel_usage = getIntelGpuUsage();
    if (intel_usage >= 0.0f) {
        return intel_usage;
    }
    
    // Method 3: Try AMD GPU via radeontop
    float amd_usage = getAmdGpuUsage();
    if (amd_usage >= 0.0f) {
        return amd_usage;
    }
    
    // No GPU detected or supported
    return 0.0f;
}

float SystemHealth::getNvidiaGpuUsage() {
    // Use nvidia-smi to get GPU utilization
    FILE* pipe = popen("nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null", "r");
    if (!pipe) return -1.0f;
    
    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof buffer, pipe) != nullptr) {
        result += buffer;
    }
    pclose(pipe);
    
    if (result.empty()) return -1.0f;
    
    try {
        return std::stof(result);
    } catch (...) {
        return -1.0f;
    }
}

float SystemHealth::getIntelGpuUsage() {
    // Check for Intel GPU usage via /sys/class/drm/card*/engine/*/busy
    std::ifstream file("/sys/class/drm/card0/engine/rcs0/busy");
    if (!file.is_open()) {
        // Try alternative path
        file.open("/sys/class/drm/card1/engine/rcs0/busy");
        if (!file.is_open()) return -1.0f;
    }
    
    unsigned long long busy_time;
    file >> busy_time;
    file.close();
    
    // This is a simplified approach - busy time accumulates, so we need to calculate percentage
    // For now, return a basic indication (would need proper time-based calculation)
    static unsigned long long prev_busy = 0;
    static auto prev_time = std::chrono::steady_clock::now();
    
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - prev_time).count();
    
    if (time_diff > 100 && prev_busy > 0) { // Only calculate after some time has passed
        unsigned long long busy_diff = busy_time - prev_busy;
        float usage = (float)busy_diff / (time_diff * 1000.0f) * 100.0f; // Rough estimation
        
        prev_busy = busy_time;
        prev_time = current_time;
        
        return std::min(100.0f, usage);
    }
    
    prev_busy = busy_time;
    prev_time = current_time;
    return 0.0f;
}

float SystemHealth::getAmdGpuUsage() {
    // Try reading AMD GPU usage from sysfs
    std::ifstream file("/sys/class/drm/card0/device/gpu_busy_percent");
    if (!file.is_open()) {
        // Try alternative path
        file.open("/sys/class/drm/card1/device/gpu_busy_percent");
        if (!file.is_open()) return -1.0f;
    }
    
    float usage;
    file >> usage;
    file.close();
    
    return std::min(100.0f, std::max(0.0f, usage));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemHealth>());
    rclcpp::shutdown();
    return 0;
}
