#pragma once

#include <chrono>
#include <memory>

#include "common/time.h"
#include "estimator/2d/submap_2d.h"
#include "estimator/ceres_scan_matcher_2d.h"
//#include "estimator/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "estimator/motion_filter.h"
#include "estimator/range_data_collator.h"
#include "estimator/pose_extrapolator.h"
#include "estimator/trajectory_node.h"
// #include "cartographer/metrics/family_factory.h"
#include "common/voxel_filter.h"
#include "common/basic_types.h"
#include "common/rigid_transform.h"
#include "common/config.h"

namespace carto_slam
{
    namespace estimator
    {

        // Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
        // without loop closure.
        // TODO(gaschler): Add test for this class similar to the 3D test.
        class LocalTrajectoryBuilder2D
        {
        public:
            typedef std::shared_ptr<LocalTrajectoryBuilder2D> Ptr;
            struct InsertionResult
            {
                std::shared_ptr<const TrajectoryNode::Data> constant_data;
                std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
            };
            struct MatchingResult
            {
                common::Time time;
                common::Rigid3d local_pose;
                common::RangeData range_data_in_local;
                // 'nullptr' if dropped by the motion filter.
                std::unique_ptr<const InsertionResult> insertion_result;
            };

            LocalTrajectoryBuilder2D(const Config &options, const std::vector<std::string> &expected_range_sensor_ids);
            ~LocalTrajectoryBuilder2D();

            //LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D &) = delete;
            LocalTrajectoryBuilder2D &operator=(const LocalTrajectoryBuilder2D &) = delete;

            // Returns 'MatchingResult' when range data accumulation completed,
            // otherwise 'nullptr'. Range data must be approximately horizontal
            // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
            // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
            // relative time of point with respect to `TimedPointCloudData::time`.
            /**
            * Range Data
            */
            std::unique_ptr<MatchingResult> AddRangeData(const std::string &sensor_id, const common::TimedPointCloudData &range_data);

            /**
            * Imu Data
            */
            void AddImuData(const common::ImuData &imu_data);

            /**
            * Odometry Data
            */
            void AddOdometryData(const common::OdometryData &odometry_data);

            // static void RegisterMetrics(metrics::FamilyFactory *family_factory);

        private:
            std::unique_ptr<MatchingResult> AddAccumulatedRangeData(common::Time time,
                                                                    const common::RangeData &gravity_aligned_range_data,
                                                                    const common::Rigid3d &gravity_alignment,
                                                                    const std::optional<common::Duration> &sensor_duration);

            common::RangeData TransformToGravityAlignedFrameAndFilter(const common::Rigid3f &transform_to_gravity_aligned_frame,
                                                                      const common::RangeData &range_data) const;

            std::unique_ptr<InsertionResult> InsertIntoSubmap(common::Time time, const common::RangeData &range_data_in_local,
                                                              const common::PointCloud &filtered_gravity_aligned_point_cloud,
                                                              const common::Rigid3d &pose_estimate,
                                                              const Eigen::Quaterniond &gravity_alignment);

            // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
            // observed pose, or nullptr on failure.
            std::unique_ptr<common::Rigid2d> ScanMatch(common::Time time, const common::Rigid2d &pose_prediction,
                                                       const common::PointCloud &filtered_gravity_aligned_point_cloud);

            // Lazily constructs a PoseExtrapolator.
            void InitializeExtrapolator(common::Time time);

            const Config options_;
            ActiveSubmaps2D active_submaps_;

            MotionFilter motion_filter_;
            // scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;
            CeresScanMatcher2D ceres_scan_matcher_;

            std::unique_ptr<PoseExtrapolator> extrapolator_;

            int num_accumulated_ = 0;
            common::RangeData accumulated_range_data_;

            std::optional<std::chrono::steady_clock::time_point> last_wall_time_;
            std::optional<double> last_thread_cpu_time_seconds_;
            std::optional<common::Time> last_sensor_time_;

            RangeDataCollator range_data_collator_;
        };

    } // namespace estimator
} // namespace carto_slam
