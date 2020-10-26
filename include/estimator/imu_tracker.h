#pragma once

#include <Eigen/Geometry>
#include "common/time.h"

namespace carto_slam {
    namespace estimator {
        // Keeps track of the orientation using angular velocities and linear
        // accelerations from an IMU. Because averaged linear acceleration (assuming
        // slow movement) is a direct measurement of gravity, roll/pitch does not drift,
        // though yaw does.
        class ImuTracker {
        public:
            ImuTracker(double imu_gravity_time_constant, common::Time time);

            // Advances to the given 'time' and updates the orientation to reflect this.
            void Advance(common::Time time);

            // Updates from an IMU reading (in the IMU frame).
            void AddImuLinearAccelerationObservation(const Eigen::Vector3d& imu_linear_acceleration);
            void AddImuAngularVelocityObservation(const Eigen::Vector3d& imu_angular_velocity);

            // Query the current time.
            common::Time time() const { return time_; }

            // Query the current orientation estimate.
            Eigen::Quaterniond orientation() const { return orientation_; }

        private:
            const double imu_gravity_time_constant_;
            common::Time time_;
            common::Time last_linear_acceleration_time_;
            Eigen::Quaterniond orientation_;
            Eigen::Vector3d gravity_vector_;
            Eigen::Vector3d imu_angular_velocity_;
        };
    }
}