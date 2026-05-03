#include "drone/transformations.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

Eigen::Vector3d adjust_velocity_using_yaw(const Eigen::Vector3d& velocity, float yaw) {
    Eigen::Matrix2d rotation;
    rotation << cos(yaw), -sin(yaw),
                sin(yaw), cos(yaw);
    
    Eigen::Vector2d rotated_xy = rotation * velocity.head<2>();
    
    Eigen::Vector3d result;
    result.head<2>() = rotated_xy;
    result(2) = velocity(2);  // Preserva componente Z
    
    return result;
}