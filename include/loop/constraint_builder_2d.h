#pragma once

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
// #include "absl/synchronization/mutex.h"
#include "common/fixed_ratio_sampler.h"
#include "common/histogram.h"
#include "common/math.h"
#include "common/task.h"
#include "common/thread_pool.h"
#include "estimator/trajectory_node.h"
#include "estimator/2d/submap_2d.h"
#include "estimator/ceres_scan_matcher_2d.h"
// #include "estimator/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "estimator/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "loop/pose_graph_interface.h"
// #include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
// #include "cartographer/metrics/family_factory.h"
#include "common/voxel_filter.h"
#include "common/basic_types.h"
#include "common/basic_types.h"

namespace carto_slam
{
    using namespace estimator;
    namespace loop
    {

        // Returns (map <- submap) where 'submap' is a coordinate system at the origin
        // of the Submap.
        common::Rigid2d ComputeSubmapPose(const Submap2D &submap);

        // Asynchronously computes constraints.
        //
        // Intermingle an arbitrary number of calls to 'MaybeAddConstraint',
        // 'MaybeAddGlobalConstraint', and 'NotifyEndOfNode', then call 'WhenDone' once.
        // After all computations are done the 'callback' will be called with the result
        // and another MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
        //
        // This class is thread-safe.
        class ConstraintBuilder2D
        {
        public:
            using Constraint = PoseGraphInterface::Constraint;
            using Result = std::vector<Constraint>;

            ConstraintBuilder2D(const Config &options, common::ThreadPoolInterface *thread_pool);
            ~ConstraintBuilder2D();

            ConstraintBuilder2D(const ConstraintBuilder2D &) = delete;
            ConstraintBuilder2D &operator=(const ConstraintBuilder2D &) = delete;

            // Schedules exploring a new constraint between 'submap' identified by
            // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
            // 'initial_relative_pose' is relative to the 'submap'.
            //
            // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
            // all computations are finished.
            void MaybeAddConstraint(const SubmapId &submap_id, const Submap2D *submap,
                                    const NodeId &node_id,
                                    const TrajectoryNode::Data *const constant_data,
                                    const common::Rigid2d &initial_relative_pose);

            // Schedules exploring a new constraint between 'submap' identified by
            // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
            // This performs full-submap matching.
            //
            // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
            // all computations are finished.
            void MaybeAddGlobalConstraint(
                const SubmapId &submap_id, const Submap2D *submap, const NodeId &node_id,
                const TrajectoryNode::Data *const constant_data);

            // Must be called after all computations related to one node have been added.
            void NotifyEndOfNode();

            // Registers the 'callback' to be called with the results, after all
            // computations triggered by 'MaybeAdd*Constraint' have finished.
            // 'callback' is executed in the 'ThreadPool'.
            void WhenDone(const std::function<void(const Result &)> &callback);

            // Returns the number of consecutive finished nodes.
            int GetNumFinishedNodes();

            // Delete data related to 'submap_id'.
            void DeleteScanMatcher(const SubmapId &submap_id);

            // static void RegisterMetrics(metrics::FamilyFactory *family_factory);

        private:
            struct SubmapScanMatcher
            {
                const Grid2D *grid = nullptr;
                std::unique_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher;
                std::weak_ptr<common::Task> creation_task_handle;
            };

            // The returned 'grid' and 'fast_correlative_scan_matcher' must only be
            // accessed after 'creation_task_handle' has completed.
            const SubmapScanMatcher *DispatchScanMatcherConstruction(
                const SubmapId &submap_id, const Grid2D *grid)
                EXCLUSIVE_LOCKS_REQUIRED(mutex_);

            // Runs in a background thread and does computations for an additional
            // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
            // anymore. As output, it may create a new Constraint in 'constraint'.
            void ComputeConstraint(const SubmapId &submap_id, const Submap2D *submap,
                                   const NodeId &node_id, bool match_full_submap,
                                   const TrajectoryNode::Data *const constant_data,
                                   const common::Rigid2d &initial_relative_pose,
                                   const SubmapScanMatcher &submap_scan_matcher,
                                   std::unique_ptr<Constraint> *constraint)
                LOCKS_EXCLUDED(mutex_);

            void RunWhenDoneCallback() LOCKS_EXCLUDED(mutex_);

            const Config options_;
            common::ThreadPoolInterface *thread_pool_;
            // absl::Mutex mutex_;

            // 'callback' set by WhenDone().
            std::unique_ptr<std::function<void(const Result &)>> when_done_
                GUARDED_BY(mutex_);

            // TODO(gaschler): Use atomics instead of mutex to access these counters.
            // Number of the node in reaction to which computations are currently
            // added. This is always the number of nodes seen so far, even when older
            // nodes are matched against a new submap.
            int num_started_nodes_ GUARDED_BY(mutex_) = 0;

            int num_finished_nodes_ GUARDED_BY(mutex_) = 0;

            std::unique_ptr<common::Task> finish_node_task_ GUARDED_BY(mutex_);

            std::unique_ptr<common::Task> when_done_task_ GUARDED_BY(mutex_);

            // Constraints currently being computed in the background. A deque is used to
            // keep pointers valid when adding more entries. Constraint search results
            // with below-threshold scores are also 'nullptr'.
            std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

            // Map of dispatched or constructed scan matchers by 'submap_id'.
            std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_
                GUARDED_BY(mutex_);
            std::map<SubmapId, common::FixedRatioSampler> per_submap_sampler_;

            estimator::CeresScanMatcher2D ceres_scan_matcher_;

            // Histogram of scan matcher scores.
            common::Histogram score_histogram_ GUARDED_BY(mutex_);
        };

    } // namespace loop
} // namespace carto_slam
