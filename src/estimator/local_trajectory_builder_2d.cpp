
#include "estimator/local_trajectory_builder_2d.h"
#include "common/pointcloud_transform.h"
#include "common/time.h"

#include <limits>
#include <memory>

// #include "cartographer/metrics/family_factory.h"
#include "common/basic_types.h"

namespace carto_slam
{
  using namespace common;
  namespace estimator
  {

    // static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
    // static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
    // static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
    // static auto* kRealTimeCorrelativeScanMatcherScoreMetric = metrics::Histogram::Null();
    // static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
    // static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
    // static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

    LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(const Config &options, const std::vector<std::string> &expected_range_sensor_ids)
        : options_(options),
          active_submaps_(options_),
          motion_filter_(options_),
          // real_time_correlative_scan_matcher_(options_.real_time_correlative_scan_matcher_options()),
          ceres_scan_matcher_(options_),
          range_data_collator_(expected_range_sensor_ids) {}

    LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

    common::RangeData LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
        const common::Rigid3f &transform_to_gravity_aligned_frame,
        const common::RangeData &range_data) const
    {
      const common::RangeData cropped =
          common::CropRangeData(common::TransformRangeData(range_data, transform_to_gravity_aligned_frame),
                                options_.min_z_, options_.max_z_);
      return common::RangeData{
          cropped.origin,
          common::VoxelFilter(cropped.returns, options_.voxel_filter_size_),
          common::VoxelFilter(cropped.misses, options_.voxel_filter_size_)};
    }

    std::unique_ptr<common::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
        const common::Time time, const common::Rigid2d &pose_prediction,
        const common::PointCloud &filtered_gravity_aligned_point_cloud)
    {
      if (active_submaps_.submaps().empty())
      {
        return std::make_unique<common::Rigid2d>(pose_prediction);
      }
      std::shared_ptr<const Submap2D> matching_submap = active_submaps_.submaps().front();
      // The online correlative scan matcher will refine the initial estimate for
      // the Ceres scan matcher.
      common::Rigid2d initial_ceres_pose = pose_prediction;

      if (options_.use_online_correlative_scan_matching_)
      {
        // const double score = real_time_correlative_scan_matcher_.Match(pose_prediction, filtered_gravity_aligned_point_cloud, *matching_submap->grid(), &initial_ceres_pose);
        // kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
      }

      /// scan match
      auto pose_observation = std::make_unique<common::Rigid2d>();
      ceres::Solver::Summary summary;
      ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose, filtered_gravity_aligned_point_cloud, *matching_submap->grid(), pose_observation.get(), &summary);
      if (pose_observation)
      {
        // kCeresScanMatcherCostMetric->Observe(summary.final_cost);
        const double residual_distance = (pose_observation->translation() - pose_prediction.translation()).norm();
        // kScanMatcherResidualDistanceMetric->Observe(residual_distance);
        const double residual_angle = std::abs(pose_observation->rotation().angle() - pose_prediction.rotation().angle());
        // kScanMatcherResidualAngleMetric->Observe(residual_angle);
      }
      return pose_observation;
    }

    std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
    LocalTrajectoryBuilder2D::AddRangeData(const std::string &sensor_id, const common::TimedPointCloudData &unsynchronized_data)
    {
      auto synchronized_data = range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
      if (synchronized_data.ranges.empty())
      {
        LOG(INFO) << "Range data collator filling buffer.";
        return nullptr;
      }

      const common::Time &time = synchronized_data.time;
      // Initialize extrapolator now if we do not ever use an IMU.
      if (!options_.use_imu_data_)
      {
        InitializeExtrapolator(time);
      }

      if (extrapolator_ == nullptr)
      {
        // Until we've initialized the extrapolator with our first IMU message, we
        // cannot compute the orientation of the rangefinder.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return nullptr;
      }

      CHECK(!synchronized_data.ranges.empty());
      // TODO(gaschler): Check if this can strictly be 0.
      CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
      const common::Time time_first_point = time + common::FromSeconds(synchronized_data.ranges.front().point_time.time);
      if (time_first_point < extrapolator_->GetLastPoseTime())
      {
        LOG(INFO) << "Extrapolator is still initializing.";
        return nullptr;
      }

      std::vector<common::Rigid3f> range_data_poses;
      range_data_poses.reserve(synchronized_data.ranges.size());
      bool warned = false;
      for (const auto &range : synchronized_data.ranges)
      {
        common::Time time_point = time + common::FromSeconds(range.point_time.time);
        if (time_point < extrapolator_->GetLastExtrapolatedTime())
        {
          if (!warned)
          {
            // LOG(ERROR) << "Timestamp of individual range data point jumps backwards from " << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
            warned = true;
          }
          time_point = extrapolator_->GetLastExtrapolatedTime();
        }
        range_data_poses.push_back(extrapolator_->ExtrapolatePose(time_point).cast<float>());
      }

      if (num_accumulated_ == 0)
      {
        // 'accumulated_range_data_.origin' is uninitialized until the last
        // accumulation.
        accumulated_range_data_ = common::RangeData{{}, {}, {}};
      }

      // Drop any returns below the minimum range and convert returns beyond the
      // maximum range into misses.
      for (size_t i = 0; i < synchronized_data.ranges.size(); ++i)
      {
        const common::TimedRangefinderPoint &hit = synchronized_data.ranges[i].point_time;
        const Eigen::Vector3f origin_in_local = range_data_poses[i] * synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
        common::RangefinderPoint hit_in_local = range_data_poses[i] * common::ToRangefinderPoint(hit);
        const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
        const float range = delta.norm();
        if (range >= options_.min_range_)
        {
          if (range <= options_.max_range_)
          {
            accumulated_range_data_.returns.push_back(hit_in_local);
          }
          else
          {
            hit_in_local.position = origin_in_local + options_.missing_data_ray_length_ / range * delta;
            accumulated_range_data_.misses.push_back(hit_in_local);
          }
        }
      }
      ++num_accumulated_;

      if (num_accumulated_ >= options_.num_accumulated_range_data_)
      {
        const common::Time current_sensor_time = synchronized_data.time;
        std::optional<common::Duration> sensor_duration;
        if (last_sensor_time_.has_value())
        {
          sensor_duration = current_sensor_time - last_sensor_time_.value();
        }
        last_sensor_time_ = current_sensor_time;
        num_accumulated_ = 0;
        const common::Rigid3d gravity_alignment = common::Rigid3d::Rotation(extrapolator_->EstimateGravityOrientation(time));
        // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
        // 'time'.
        accumulated_range_data_.origin = range_data_poses.back().translation();
        return AddAccumulatedRangeData(
            time,
            TransformToGravityAlignedFrameAndFilter(
                gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
                accumulated_range_data_),
            gravity_alignment, sensor_duration);
      }
      return nullptr;
    }

    std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
    LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
        const common::Time time,
        const common::RangeData &gravity_aligned_range_data,
        const common::Rigid3d &gravity_alignment,
        const std::optional<common::Duration> &sensor_duration)
    {
      if (gravity_aligned_range_data.returns.empty())
      {
        LOG(WARNING) << "Dropped empty horizontal range data.";
        return nullptr;
      }

      // Computes a gravity aligned pose prediction.
      const common::Rigid3d non_gravity_aligned_pose_prediction = extrapolator_->ExtrapolatePose(time);
      const common::Rigid2d pose_prediction = common::Project2D(non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

      const common::PointCloud &filtered_gravity_aligned_point_cloud = common::AdaptiveVoxelFilter(gravity_aligned_range_data.returns, options_);
      if (filtered_gravity_aligned_point_cloud.empty())
      {
        return nullptr;
      }

      // local map frame <- gravity-aligned frame
      // Scan Match !!!
      std::unique_ptr<common::Rigid2d> pose_estimate_2d = ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
      if (pose_estimate_2d == nullptr)
      {
        LOG(WARNING) << "Scan matching failed.";
        return nullptr;
      }
      const common::Rigid3d pose_estimate = common::Embed3D(*pose_estimate_2d) * gravity_alignment;
      extrapolator_->AddPose(time, pose_estimate);

      common::RangeData range_data_in_local = TransformRangeData(gravity_aligned_range_data, common::Embed3D(pose_estimate_2d->cast<float>()));

      // Insert Submap !!!
      std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(time, range_data_in_local, filtered_gravity_aligned_point_cloud, pose_estimate, gravity_alignment.rotation());

      // const auto wall_time = std::chrono::steady_clock::now();
      // if (last_wall_time_.has_value())
      // {
      //   const auto wall_time_duration = wall_time - last_wall_time_.value();
      //   kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
      //   if (sensor_duration.has_value())
      //   {
      //     kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) / common::ToSeconds(wall_time_duration));
      //   }
      // }
      // const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
      // if (last_thread_cpu_time_seconds_.has_value())
      // {
      //   const double thread_cpu_duration_seconds = thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
      //   if (sensor_duration.has_value())
      //   {
      //     kLocalSlamCpuRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) / thread_cpu_duration_seconds);
      //   }
      // }
      // last_wall_time_ = wall_time;
      // last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
      return std::make_unique<MatchingResult>(
          MatchingResult{time, pose_estimate, std::move(range_data_in_local), std::move(insertion_result)});
    }

    std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
    LocalTrajectoryBuilder2D::InsertIntoSubmap(
        const common::Time time, const common::RangeData &range_data_in_local,
        const common::PointCloud &filtered_gravity_aligned_point_cloud,
        const common::Rigid3d &pose_estimate,
        const Eigen::Quaterniond &gravity_alignment)
    {
      if (motion_filter_.IsSimilar(time, pose_estimate))
      {
        return nullptr;
      }
      std::vector<std::shared_ptr<const Submap2D>> insertion_submaps = active_submaps_.InsertRangeData(range_data_in_local);
      return std::make_unique<InsertionResult>(InsertionResult{
          std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
              time,
              gravity_alignment,
              filtered_gravity_aligned_point_cloud,
              {}, // 'high_resolution_point_cloud' is only used in 3D.
              {}, // 'low_resolution_point_cloud' is only used in 3D.
              {}, // 'rotational_scan_matcher_histogram' is only used in 3D.
              pose_estimate}),
          std::move(insertion_submaps)});
    }

    void LocalTrajectoryBuilder2D::AddImuData(const common::ImuData &imu_data)
    {
      // CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
      InitializeExtrapolator(imu_data.time);
      extrapolator_->AddImuData(imu_data);
    }

    void LocalTrajectoryBuilder2D::AddOdometryData(
        const common::OdometryData &odometry_data)
    {
      if (extrapolator_ == nullptr)
      {
        // Until we've initialized the extrapolator we cannot add odometry data.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return;
      }
      extrapolator_->AddOdometryData(odometry_data);
    }

    void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time)
    {
      if (extrapolator_ != nullptr)
      {
        return;
      }
      // CHECK(!options_.pose_extrapolator_options().use_imu_based());
      // TODO(gaschler): Consider using InitializeWithImu as 3D does.
      extrapolator_ = std::make_unique<PoseExtrapolator>(
          FromSeconds(options_.pose_queue_duration_),
          options_.imu_gravity_time_constant_);
      extrapolator_->AddPose(time, common::Rigid3d::Identity());
    }

  } // namespace estimator
} // namespace carto_slam
