#include "estimator/imu_tracker.h"

#include "common/transform.h"

namespace carto_slam
{
    namespace estimator
    {
        ImuTracker::ImuTracker(double imu_gravity_time_constant, common::Time time) : imu_gravity_time_constant_(imu_gravity_time_constant),
                                                                                      time_(time),
                                                                                      last_linear_acceleration_time_(common::Time::min()),
                                                                                      orientation_(Eigen::Quaterniond::Identity()),
                                                                                      gravity_vector_(Eigen::Vector3d::UnitZ()),
                                                                                      imu_angular_velocity_(Eigen::Vector3d::Zero())
        {
        }

        void ImuTracker::Advance(const common::Time time)
        {
            const double delta_t = common::ToSeconds(time - time_);
            const Eigen::Quaterniond rotation = common::AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(imu_angular_velocity_ * delta_t));
            orientation_ = (orientation_ * rotation).normalized();
            gravity_vector_ = rotation.conjugate() * gravity_vector_;
            time_ = time;
        }

        void ImuTracker::AddImuLinearAccelerationObservation(const Eigen::Vector3d &imu_linear_acceleration)
        {
            // Update the 'gravity_vector_' with an exponential moving average using the
            // 'imu_gravity_time_constant'.
            const double delta_t =
                last_linear_acceleration_time_ > common::Time::min()
                    ? common::ToSeconds(time_ - last_linear_acceleration_time_)
                    : std::numeric_limits<double>::infinity();
            last_linear_acceleration_time_ = time_;
            const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
            gravity_vector_ =
                (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
            // Change the 'orientation_' so that it agrees with the current
            // 'gravity_vector_'.
            const Eigen::Quaterniond rotation = common::FromTwoVectors(
                gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
            orientation_ = (orientation_ * rotation).normalized();
            //CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
            //CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
        }

        void ImuTracker::AddImuAngularVelocityObservation(
            const Eigen::Vector3d &imu_angular_velocity)
        {
            imu_angular_velocity_ = imu_angular_velocity;
        }

    } // namespace estimator
} // namespace carto_slam