#pragma once

#include <deque>
#include <memory>

#include "common/basic_types.h"
#include "common/time.h"
#include "estimator/imu_tracker.h"
// #include "cartographer/mapping/pose_extrapolator_interface.h"
// #include "cartographer/sensor/imu_data.h"
// #include "cartographer/sensor/odometry_data.h"
#include "common/rigid_transform.h"

namespace carto_slam
{
  namespace estimator
  {

    // Keep poses for a certain duration to estimate linear and angular velocity.
    // Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
    // available to improve the extrapolation.
    class PoseExtrapolator
    {
    public:
      struct ExtrapolationResult
      {
        // The poses for the requested times at index 0 to N-1.
        std::vector<common::Rigid3f> previous_poses;
        // The pose for the requested time at index N.
        common::Rigid3d current_pose;
        Eigen::Vector3d current_velocity;
        Eigen::Quaterniond gravity_from_tracking;
      };
      explicit PoseExtrapolator(common::Duration pose_queue_duration, double imu_gravity_time_constant);

      PoseExtrapolator(const PoseExtrapolator &) = delete;
      PoseExtrapolator &operator=(const PoseExtrapolator &) = delete;

      static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
          common::Duration pose_queue_duration, double imu_gravity_time_constant,
          const common::ImuData &imu_data);

      // Returns the time of the last added pose or Time::min() if no pose was added
      // yet.
      common::Time GetLastPoseTime() const;
      common::Time GetLastExtrapolatedTime() const;

      void AddPose(common::Time time, const common::Rigid3d &pose);
      void AddImuData(const common::ImuData &imu_data);
      void AddOdometryData(const common::OdometryData &odometry_data);
      common::Rigid3d ExtrapolatePose(common::Time time);

      ExtrapolationResult ExtrapolatePosesWithGravity(const std::vector<common::Time> &times);

      // Returns the current gravity alignment estimate as a rotation from
      // the tracking frame into a gravity aligned frame.
      Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

    private:
      void UpdateVelocitiesFromPoses();
      void TrimImuData();
      void TrimOdometryData();
      void AdvanceImuTracker(common::Time time, ImuTracker *imu_tracker) const;
      Eigen::Quaterniond ExtrapolateRotation(common::Time time, ImuTracker *imu_tracker) const;
      Eigen::Vector3d ExtrapolateTranslation(common::Time time);

      const common::Duration pose_queue_duration_;
      struct TimedPose
      {
        common::Time time;
        common::Rigid3d pose;
      };
      std::deque<TimedPose> timed_pose_queue_;
      Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
      Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

      const double gravity_time_constant_;
      std::deque<common::ImuData> imu_data_;
      std::unique_ptr<ImuTracker> imu_tracker_;
      std::unique_ptr<ImuTracker> odometry_imu_tracker_;
      std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
      TimedPose cached_extrapolated_pose_;

      std::deque<common::OdometryData> odometry_data_;
      Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
      Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
    };

  } // namespace estimator
} // namespace carto_slam
