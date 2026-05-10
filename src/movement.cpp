#include "drone/movement.hpp"
#include <Eigen/Eigen>
#include <cmath>
#include "drone/Drone.hpp"
#include "drone/transformations.hpp"
#include <map>
#include <unordered_map>
#include <chrono>
#include <thread>

namespace Sentido {
    extern const Direction FRENTE = Direction({1.0, 0.0, 0.0});
    extern const Direction TRAS = Direction({-1.0, 0.0, 0.0});
    extern const Direction ESQUERDA = Direction({0.0, -1.0, 0.0});
    extern const Direction DIREITA = Direction({0.0, 1.0, 0.0});
    extern const Direction BAIXO = Direction({0.0, 0.0, 1.0});
    extern const Direction CIMA = Direction({0.0, 0.0, -1.0});
}

// Utility function to normalize yaw error to [-π, π]
float normalizeYawError(float yaw_error) {
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    return yaw_error;
}

void move_local_by_speed(std::shared_ptr<Drone> drone, Eigen::Vector3d direction, float speed) {
    if (drone == nullptr) return;

    Eigen::Vector3d local_velocity = direction * speed;
    Eigen::Vector3d adjusted_velocity = adjust_velocity_using_yaw(local_velocity, drone->getOrientation()[2]);
    drone->setLocalVelocity(adjusted_velocity.x(), adjusted_velocity.y(), adjusted_velocity.z(), 0.0f);
}

void move_local_by_speed(std::shared_ptr<Drone> drone, float vx, float vy, float vz) {
    if (drone == nullptr) return;

    Eigen::Vector3d local_velocity(vx, vy, vz);
    Eigen::Vector3d adjusted_velocity = adjust_velocity_using_yaw(local_velocity, drone->getOrientation()[2]);
    drone->setLocalVelocity(adjusted_velocity.x(), adjusted_velocity.y(), adjusted_velocity.z(), 0.0f);
}

void move_local_by_vel_as_position(std::shared_ptr<Drone> drone, float vx, float vy, float vz) {
    if (drone == nullptr) return;

    // Rotate body-frame velocity into NED world frame (same as move_local_by_speed)
    Eigen::Vector3d local_velocity(vx, vy, vz);
    Eigen::Vector3d world_velocity = adjust_velocity_using_yaw(local_velocity, drone->getOrientation()[2]);

    // Integrate one FSM tick ahead (50 ms) to get a position target
    constexpr float dt = 0.05f;
    Eigen::Vector3d pos = drone->getLocalPosition();

    drone->setLocalPosition(
        static_cast<float>(pos.x()) + world_velocity.x() * dt,
        static_cast<float>(pos.y()) + world_velocity.y() * dt,
        static_cast<float>(pos.z()) + world_velocity.z() * dt,
        drone->getOrientation()[2]
    );
}

bool move_local_by_waypoint(std::shared_ptr<Drone> drone, Eigen::Vector3d waypoint, float speed, float tolerance, float target_yaw) {
    Eigen::Vector3d local_position = drone->getLocalPosition();

    Eigen::Vector3d diff = waypoint - local_position;
    bool pos_reached = diff.norm() < tolerance;
    bool yaw_reached = true;
    float applied_yaw = drone->getOrientation()[2];

    if (!std::isnan(target_yaw)) {
        float yaw_diff = normalizeYawError(target_yaw - applied_yaw);
        // Default yaw tolerance of 0.05 radians
        yaw_reached = std::abs(yaw_diff) < 0.05f;
        applied_yaw = target_yaw;
    }

    if (pos_reached && yaw_reached) return true;

    Eigen::Vector3d little_goal = local_position;
    if (!pos_reached) {
        little_goal = local_position + (diff.norm() > speed ? diff.normalized() * speed : diff);
    }

    drone->setLocalPosition(
        little_goal.x(),
        little_goal.y(),
        little_goal.z(),
        applied_yaw
    );

    return false;
}

/**
 * @brief Move drone in a specific direction by distance
 * @param drone Shared pointer to drone object
 * @param sentido Direction vector from Sentido namespace
 * @param distance Distance to move in meters
 * @param speed Movement speed
 * @param tolerance Position tolerance for arrival
 * @param with_timeout Enable timeout protection
 * @param timeout_seconds Maximum time allowed for movement
 * @return true if movement completed successfully, false if timeout or error
 */
bool move_local_by_sentido(std::shared_ptr<Drone> drone, const Direction& sentido, 
                          float distance, float speed, float tolerance, 
                          bool with_timeout, float timeout_seconds) {

    if (!drone) return false;

    const Direction start_position = drone->getLocalPosition();
    const Direction goal = start_position + sentido * distance;

    const auto start_time = std::chrono::high_resolution_clock::now();
    
    int log_counter = 0;
    while (!move_local_by_waypoint(drone, goal, speed, tolerance)) {
        
        // Check timeout if enabled
        if (with_timeout) {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            
            if (elapsed.count() > timeout_seconds * 1000) {
                drone->log("TIMEOUT: Movement failed after " + std::to_string(timeout_seconds) + "s");
                drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                return false;
            }
        }
        
        // Throttled logging (every 50 iterations to avoid spam)
        if (log_counter++ % 50 == 0) {
            Eigen::Vector3d current_pos = drone->getLocalPosition();
            double error = (goal - current_pos).norm();
            drone->log("Moving... Distance to goal: " + std::to_string(error) + "m");
        }
        
        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // parar o drone
    drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    
    Eigen::Vector3d final_pos = drone->getLocalPosition();
    double final_error = (goal - final_pos).norm();
    drone->log("Movement completed! Final error: " + std::to_string(final_error) + "m");
    
    return true;
}

bool rotateYaw(std::shared_ptr<Drone> drone, float target_yaw, float yaw_rate, float tolerance) {
    float current_yaw = drone->getOrientation()[2];
    float yaw_diff = normalizeYawError(target_yaw - current_yaw);

    if (std::abs(yaw_diff) < tolerance) {
        drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop rotation
        return true;
    }

    float applied_yaw_rate = (yaw_diff > 0 ? yaw_rate : -yaw_rate);
    drone->setLocalVelocity(0.0f, 0.0f, 0.0f, applied_yaw_rate);
    
    return false;
}

bool rotateAngle(std::shared_ptr<Drone> drone, float angle, float yaw_rate, float tolerance) {
    float target_yaw = drone->getOrientation()[2] + angle;
    // Normalize target_yaw to [-π, π]
    while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;

    return rotateYaw(drone, target_yaw, yaw_rate, tolerance);
}

bool lookToPoint(std::shared_ptr<Drone> drone, Eigen::Vector3d goal_point, float yaw_rate, float tolerance) {
    Eigen::Vector3d current_pos = drone->getLocalPosition();
    float desired_yaw = std::atan2(goal_point.y() - current_pos.y(), goal_point.x() - current_pos.x());

    return rotateYaw(drone, desired_yaw, yaw_rate, tolerance);
}