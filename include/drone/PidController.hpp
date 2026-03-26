#pragma once

#include <chrono>
#include <algorithm>

/**
 * @brief Generic PID Controller with time-based computation.
 * 
 * Ported from cbr_2025's battle-tested implementation. 
 * Features: sample-time gating, setpoint tracking, tuning, and reset.
 *
 * Usage:
 *   PidController pid(1.0, 0.0, 0.5, target_value);
 *   float output = pid.compute(current_value);
 */
class PidController {
public:
    PidController(float kp, float ki, float kd, float setpoint, float sample_time = 0.05f)
        : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), sample_time_(sample_time),
          integral_(0.0f), previous_error_(0.0f), 
          previous_time_(std::chrono::high_resolution_clock::now()) {}

    /**
     * @brief Compute the PID output given the current measured value.
     * Only updates if enough time has elapsed (sample_time).
     * @return PID output, or 0.0 if sample_time hasn't elapsed yet.
     */
    float compute(float current_value) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - previous_time_;
        float dt = elapsed.count();

        if (dt >= sample_time_) {
            float error = setpoint_ - current_value;
            integral_ += error * dt;
            float derivative = (error - previous_error_) / dt;

            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

            previous_error_ = error;
            previous_time_ = current_time;

            return output;
        }

        return 0.0f;
    }

    /**
     * @brief Update PID gains at runtime.
     */
    void setTunings(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    /**
     * @brief Update the target setpoint.
     */
    void setSetpoint(float setpoint) {
        setpoint_ = setpoint;
    }

    /**
     * @brief Reset the integral and derivative terms.
     * Call this when switching states or re-engaging the controller.
     */
    void reset() {
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        previous_time_ = std::chrono::high_resolution_clock::now();
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float setpoint_;
    float sample_time_;

    float integral_;
    float previous_error_;

    std::chrono::high_resolution_clock::time_point previous_time_;
};