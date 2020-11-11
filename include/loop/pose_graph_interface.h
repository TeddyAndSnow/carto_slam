#pragma once

#include <chrono>
#include <vector>
#include <optional>

#include "estimator/id.h"
#include "estimator/submaps.h"
#include "estimator/trajectory_node.h"
#include "common/rigid_transform.h"

namespace carto_slam
{
  using namespace estimator;
  namespace loop
  {

    class PoseGraphInterface
    {
    public:
      // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
      // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
      // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
      struct Constraint
      {
        struct Pose
        {
          common::Rigid3d zbar_ij;
          double translation_weight;
          double rotation_weight;
        };

        SubmapId submap_id; // 'i' in the paper.
        NodeId node_id;     // 'j' in the paper.

        // Pose of the node 'j' relative to submap 'i'.
        Pose pose;

        // Differentiates between intra-submap (where node 'j' was inserted into
        // submap 'i') and inter-submap constraints (where node 'j' was not inserted
        // into submap 'i').
        enum Tag
        {
          INTRA_SUBMAP,
          INTER_SUBMAP
        } tag;
      };

      struct LandmarkNode
      {
        struct LandmarkObservation
        {
          int trajectory_id;
          common::Time time;
          common::Rigid3d landmark_to_tracking_transform;
          double translation_weight;
          double rotation_weight;
        };
        std::vector<LandmarkObservation> landmark_observations;
        std::optional<common::Rigid3d> global_landmark_pose;
        bool frozen = false;
      };

      struct SubmapPose
      {
        int version;
        common::Rigid3d pose;
      };

      struct SubmapData
      {
        std::shared_ptr<const Submap> submap;
        common::Rigid3d pose;
      };

      struct TrajectoryData
      {
        double gravity_constant = 9.8;
        std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
        std::optional<common::Rigid3d> fixed_frame_origin_in_map;
      };

      enum class TrajectoryState
      {
        ACTIVE,
        FINISHED,
        FROZEN,
        DELETED
      };

      using GlobalSlamOptimizationCallback =
          std::function<void(const std::map<int /* trajectory_id */, SubmapId> &,
                             const std::map<int /* trajectory_id */, NodeId> &)>;

      PoseGraphInterface() {}
      virtual ~PoseGraphInterface() {}

      PoseGraphInterface(const PoseGraphInterface &) = delete;
      PoseGraphInterface &operator=(const PoseGraphInterface &) = delete;

      // Waits for all computations to finish and computes optimized poses.
      virtual void RunFinalOptimization() = 0;

      // Returns data for all submaps.
      virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

      // Returns the global poses for all submaps.
      virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

      // Returns the transform converting data in the local map frame (i.e. the
      // continuous, non-loop-closed frame) into the global map frame (i.e. the
      // discontinuous, loop-closed frame).
      virtual common::Rigid3d GetLocalToGlobalTransform(
          int trajectory_id) const = 0;

      // Returns the current optimized trajectories.
      virtual MapById<NodeId, estimator::TrajectoryNode> GetTrajectoryNodes() const = 0;

      // Returns the current optimized trajectory poses.
      virtual MapById<NodeId, estimator::TrajectoryNodePose> GetTrajectoryNodePoses()
          const = 0;

      // Returns the states of trajectories.
      virtual std::map<int, TrajectoryState> GetTrajectoryStates() const = 0;

      // Returns the current optimized landmark poses.
      virtual std::map<std::string, common::Rigid3d> GetLandmarkPoses()
          const = 0;

      // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
      virtual void SetLandmarkPose(const std::string &landmark_id,
                                   const common::Rigid3d &global_pose,
                                   const bool frozen = false) = 0;

      // Deletes a trajectory asynchronously.
      virtual void DeleteTrajectory(int trajectory_id) = 0;

      // Checks if the given trajectory is finished.
      virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

      // Checks if the given trajectory is frozen.
      virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

      // Returns the trajectory data.
      virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;

      // Returns the collection of constraints.
      virtual std::vector<Constraint> constraints() const = 0;

      // Serializes the constraints and trajectories. If
      // 'include_unfinished_submaps' is set to 'true', unfinished submaps, i.e.
      // submaps that have not yet received all rangefinder data insertions, will
      // be included, otherwise not.
      // virtual proto::PoseGraph ToProto(bool include_unfinished_submaps) const = 0;

      // Sets the callback function that is invoked whenever the global optimization
      // problem is solved.
      virtual void SetGlobalSlamOptimizationCallback(
          GlobalSlamOptimizationCallback callback) = 0;
    };

  } // namespace loop
} // namespace carto_slam


