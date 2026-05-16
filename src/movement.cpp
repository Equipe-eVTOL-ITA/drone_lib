#include "drone/movement.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include "drone/Drone.hpp"
#include "drone/transformations.hpp"

float normalizeYawError(float yaw_error) {
    while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
    return yaw_error;
}

void move_local_by_speed(std::shared_ptr<Drone> drone, Eigen::Vector3d direction, float speed) {
    if (!drone) return;
    Eigen::Vector3d vel = adjust_velocity_using_yaw(direction * speed, drone->getOrientation()[2]);
    drone->setLocalVelocity(vel.x(), vel.y(), vel.z(), 0.0f);
}

void move_local_by_speed(std::shared_ptr<Drone> drone, float vx, float vy, float vz) {
    if (!drone) return;
    Eigen::Vector3d vel = adjust_velocity_using_yaw(Eigen::Vector3d(vx, vy, vz), drone->getOrientation()[2]);
    drone->setLocalVelocity(vel.x(), vel.y(), vel.z(), 0.0f);
}

void move_local_by_vel_as_position(std::shared_ptr<Drone> drone, float vx, float vy, float vz) {
    if (!drone) return;
    constexpr float dt = 0.05f;
    Eigen::Vector3d vel = adjust_velocity_using_yaw(Eigen::Vector3d(vx, vy, vz), drone->getOrientation()[2]);
    Eigen::Vector3d pos = drone->getLocalPosition();
    drone->setLocalPosition(
        static_cast<float>(pos.x()) + vel.x() * dt,
        static_cast<float>(pos.y()) + vel.y() * dt,
        static_cast<float>(pos.z()) + vel.z() * dt,
        drone->getOrientation()[2]);
}

// ─────────────────────────────────────────────────────────────────────────────

bool move_local_constant_step(std::shared_ptr<Drone> drone, Eigen::Vector3d waypoint,
                               float step_m, float tolerance, float target_yaw) {
    if (!drone) return false;

    Eigen::Vector3d pos = drone->getLocalPosition();
    float cur_yaw       = static_cast<float>(drone->getOrientation()[2]);
    float applied_yaw   = std::isnan(target_yaw) ? cur_yaw : target_yaw;

    // Hold: segura posição atual
    if (step_m <= 0.0f) {
        drone->setLocalPosition(pos.x(), pos.y(), pos.z(), applied_yaw);
        return false;
    }

    Eigen::Vector3d diff = waypoint - pos;
    bool pos_ok = diff.norm() < tolerance;
    bool yaw_ok = std::isnan(target_yaw) ||
                  std::abs(normalizeYawError(target_yaw - cur_yaw)) < 0.05f;

    if (pos_ok && yaw_ok) return true;

    Eigen::Vector3d little_goal = pos;
    if (!pos_ok)
        little_goal = pos + (diff.norm() > step_m ? diff.normalized() * step_m : diff);

    drone->setLocalPosition(little_goal.x(), little_goal.y(), little_goal.z(), applied_yaw);
    return false;
}

bool TrapezoidalMover::update(std::shared_ptr<Drone> drone, Eigen::Vector3d waypoint,
                               float v_max, float a_max, float tolerance, float target_yaw) {
    if (!drone) return false;

    Eigen::Vector3d pos  = drone->getLocalPosition();
    Eigen::Vector3d diff = waypoint - pos;
    float dist           = static_cast<float>(diff.norm());
    float cur_yaw        = static_cast<float>(drone->getOrientation()[2]);
    float applied_yaw    = std::isnan(target_yaw) ? cur_yaw : target_yaw;

    bool yaw_ok = std::isnan(target_yaw) ||
                  std::abs(normalizeYawError(target_yaw - cur_yaw)) < 0.05f;

    // Chegou ao destino
    if (dist < tolerance && yaw_ok) {
        v_curr_ = 0.0f;
        drone->setLocalPosition(waypoint.x(), waypoint.y(), waypoint.z(), applied_yaw);
        return true;
    }

    // Restrição cinemática: velocidade máxima para parar dentro da tolerância
    //   d_brake = v² / (2·a)   →   v_brake = sqrt(2·a·(dist−tol))
    float d_to_stop = std::max(0.0f, dist - tolerance);
    float v_brake   = std::sqrt(2.0f * a_max * d_to_stop);
    float v_desired = std::min(v_max, v_brake);

    // Filtro iterativo trapezoidal: limita ∆v por tick (aceleração e frenagem)
    float dv     = v_desired - v_curr_;
    float dv_max = a_max * kDt;
    v_curr_ += std::clamp(dv, -dv_max, dv_max);
    v_curr_  = std::max(0.0f, v_curr_);

    // O setpoint é colocado kLookahead segundos à frente na velocidade atual.
    // PX4 usa controle proporcional de posição: v_PX4 = MPC_XY_P × dist_setpoint.
    // Com MPC_XY_P ≈ 0.95 (default PX4), kLookahead = 1.0 → v_PX4 ≈ v_curr.
    // Usar kDt (0.05s) colocaria o setpoint apenas 0.025m à frente a 0.5 m/s,
    // fazendo o PX4 voar a 0.025 m/s em vez de 0.5 m/s.
    if (dist > 1e-6f) {
        float step = v_curr_ * kLookahead;
        Eigen::Vector3d little_goal = pos + diff.normalized() * std::min(step, dist);
        drone->setLocalPosition(little_goal.x(), little_goal.y(), little_goal.z(), applied_yaw);
    }

    return false;
}
