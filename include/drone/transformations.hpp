#pragma once

#include <Eigen/Eigen>

Eigen::Vector3d adjust_velocity_using_yaw(const Eigen::Vector3d& velocity, float yaw);