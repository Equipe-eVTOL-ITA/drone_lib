#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "drone/Drone.hpp"

// Type alias for direction vectors
using Direction = Eigen::Vector3d;

namespace Sentido {
    extern const Direction FRENTE;
    extern const Direction TRAS;
    extern const Direction ESQUERDA;
    extern const Direction DIREITA;
    extern const Direction BAIXO;
    extern const Direction CIMA;
}

float normalizeYawError(float yaw_error);

void move_local_by_speed(std::shared_ptr<Drone> drone, Eigen::Vector3d direction, float speed);

void move_local_by_speed(std::shared_ptr<Drone> drone, float vx, float vy, float vz);

#include <limits>

bool move_local_by_waypoint(std::shared_ptr<Drone> drone, Eigen::Vector3d waypoint, float speed, float tolerance = 0.1f, float target_yaw = std::numeric_limits<float>::quiet_NaN());

bool move_local_by_sentido(std::shared_ptr<Drone> drone, const Direction& sentido, 
                          float distance, float speed, float tolerance = 0.1f, 
                          bool with_timeout = false, float timeout_seconds = 10.0f);

bool rotateYaw(std::shared_ptr<Drone> drone, float target_yaw, float yaw_rate = 0.3f, float tolerance = 0.05f);

bool rotateAngle(std::shared_ptr<Drone> drone, float angle, float yaw_rate = 0.3f, float tolerance = 0.05f);

bool lookToPoint(std::shared_ptr<Drone> drone, Eigen::Vector3d goal_point, float yaw_rate = 0.3f, float tolerance = 0.05f);