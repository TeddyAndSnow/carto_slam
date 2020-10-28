#pragma once

#include <memory>
#include <vector>
#include <optional>

#include <Eigen/Core>
#include "common/time.h"
#include "common/basic_types.h"
#include "common/rigid_transform.h"

namespace carto_slam
{
  namespace estimator
  {
    struct TrajectoryNodePose
    {
      struct ConstantPoseData
      {
        common::Time time;
        common::Rigid3d local_pose;
      };
      // The node pose in the global SLAM frame.
      common::Rigid3d global_pose;

      std::optional<ConstantPoseData> constant_pose_data;
    };

    struct TrajectoryNode
    {
      struct Data
      {
        common::Time time;

        // Transform to approximately gravity align the tracking frame as
        // determined by local SLAM.
        Eigen::Quaterniond gravity_alignment;

        // Used for loop closure in 2D: voxel filtered returns in the
        // 'gravity_alignment' frame.
        common::PointCloud filtered_gravity_aligned_point_cloud;

        // Used for loop closure in 3D.
        common::PointCloud high_resolution_point_cloud;
        common::PointCloud low_resolution_point_cloud;
        Eigen::VectorXf rotational_scan_matcher_histogram;

        // The node pose in the local SLAM frame.
        common::Rigid3d local_pose;
      };

      common::Time time() const { return constant_data->time; }

      // This must be a shared_ptr. If the data is used for visualization while the
      // node is being trimmed, it must survive until all use finishes.
      std::shared_ptr<const Data> constant_data;

      // The node pose in the global SLAM frame.
      common::Rigid3d global_pose;
    };

  } // namespace estimator
} // namespace carto_slam
