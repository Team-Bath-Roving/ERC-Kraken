#pragma once
#include <trajectory_msgs/msg/joint_trajectory.h>
#include "motor_driver.h"
#include <Arduino.h>

// TrajectoryController: Handles execution of JointTrajectory messages for a 7DOF arm
class TrajectoryController {
public:
    TrajectoryController(Motor* motors, size_t num_joints)
        : motors_(motors), num_joints_(num_joints), active_(false), current_point_(0), start_time_ms_(0) {}

    // Accept a new trajectory message (overwrites any in-progress trajectory)
    void setTrajectory(const trajectory_msgs__msg__JointTrajectory* traj_msg) {
        trajectory_ = *traj_msg; // Shallow copy; assumes message persists
        current_point_ = 0;
        start_time_ms_ = millis();
        active_ = (trajectory_.points.size > 0);
    }

    // Call this in the main loop to execute the trajectory
    void update() {
        if (!active_ || trajectory_.points.size == 0) return;
        uint32_t now = millis();
        // Find the next point to execute based on elapsed time
        while (current_point_ < trajectory_.points.size) {
            const auto& pt = trajectory_.points.data[current_point_];
            uint32_t pt_time_ms = pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1000000;
            if (now - start_time_ms_ < pt_time_ms) break;
            // Set joint targets for this point
            for (size_t j = 0; j < num_joints_ && j < pt.positions.size; ++j) {
                float pos = pt.positions.data[j];
                float vel = (pt.velocities.size > j) ? pt.velocities.data[j] : motors_[j].getSpeed();
                float acc = (pt.accelerations.size > j) ? pt.accelerations.data[j] : motors_[j].getAcceleration();
                motors_[j].moveToWithProfile(pos, vel, acc);
            }
            current_point_++;
        }
        // Trajectory complete
        if (current_point_ >= trajectory_.points.size) {
            active_ = false;
        }
    }

    bool isActive() const { return active_; }

private:
    Motor* motors_;
    size_t num_joints_;
    trajectory_msgs__msg__JointTrajectory trajectory_;
    bool active_;
    size_t current_point_;
    uint32_t start_time_ms_;
};
