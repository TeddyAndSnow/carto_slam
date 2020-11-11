#pragma once

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "common/time.h"
#include "estimator/id.h"
#include "loop/optimization_problem_interface.h"
#include "loop/pose_graph_interface.h"
// #include "cartographer/mapping/proto/pose_graph/optimization_problem_options.pb.h"
#include "common/basic_types.h"
#include "common/map_by_time.h"
#include "common/transform.h"
#include "common/config.h"

namespace carto_slam
{
  using namespace estimator;
  namespace loop
  {

    struct NodeSpec2D
    {
      common::Time time;
      common::Rigid2d local_pose_2d;
      common::Rigid2d global_pose_2d;
      Eigen::Quaterniond gravity_alignment;
    };

    struct SubmapSpec2D
    {
      common::Rigid2d global_pose;
    };

    class OptimizationProblem2D
        : public OptimizationProblemInterface<NodeSpec2D, SubmapSpec2D,
                                              common::Rigid2d>
    {
    public:
      explicit OptimizationProblem2D(const Config &options);
      ~OptimizationProblem2D();

      OptimizationProblem2D(const OptimizationProblem2D &) = delete;
      OptimizationProblem2D &operator=(const OptimizationProblem2D &) = delete;

      void AddImuData(int trajectory_id, const common::ImuData &imu_data) override;
      void AddOdometryData(int trajectory_id,
                           const common::OdometryData &odometry_data) override;
      void AddTrajectoryNode(int trajectory_id,
                             const NodeSpec2D &node_data) override;
      void InsertTrajectoryNode(const NodeId &node_id,
                                const NodeSpec2D &node_data) override;
      void TrimTrajectoryNode(const NodeId &node_id) override;
      void AddSubmap(int trajectory_id,
                     const common::Rigid2d &global_submap_pose) override;
      void InsertSubmap(const SubmapId &submap_id,
                        const common::Rigid2d &global_submap_pose) override;
      void TrimSubmap(const SubmapId &submap_id) override;
      void SetMaxNumIterations(int32 max_num_iterations) override;

      void Solve(
          const std::vector<Constraint> &constraints,
          const std::map<int, PoseGraphInterface::TrajectoryState> &
              trajectories_state,
          const std::map<std::string, LandmarkNode> &landmark_nodes) override;

      const MapById<NodeId, NodeSpec2D> &node_data() const override
      {
        return node_data_;
      }
      const MapById<SubmapId, SubmapSpec2D> &submap_data() const override
      {
        return submap_data_;
      }
      const std::map<std::string, common::Rigid3d> &landmark_data()
          const override
      {
        return landmark_data_;
      }
      const common::MapByTime<common::ImuData> &imu_data() const override
      {
        return empty_imu_data_;
      }
      const common::MapByTime<common::OdometryData> &odometry_data()
          const override
      {
        return odometry_data_;
      }

      void AddFixedFramePoseData(
          int trajectory_id,
          const common::FixedFramePoseData &fixed_frame_pose_data);
      void SetTrajectoryData(
          int trajectory_id,
          const PoseGraphInterface::TrajectoryData &trajectory_data);
      const common::MapByTime<common::FixedFramePoseData> &fixed_frame_pose_data()
          const
      {
        return fixed_frame_pose_data_;
      }
      const std::map<int, PoseGraphInterface::TrajectoryData> &trajectory_data()
          const
      {
        return trajectory_data_;
      }

    private:
      std::unique_ptr<common::Rigid3d> InterpolateOdometry(
          int trajectory_id, common::Time time) const;
      // Computes the relative pose between two nodes based on odometry data.
      std::unique_ptr<common::Rigid3d> CalculateOdometryBetweenNodes(
          int trajectory_id, const NodeSpec2D &first_node_data,
          const NodeSpec2D &second_node_data) const;

      Config options_;
      MapById<NodeId, NodeSpec2D> node_data_;
      MapById<SubmapId, SubmapSpec2D> submap_data_;
      std::map<std::string, common::Rigid3d> landmark_data_;
      common::MapByTime<common::ImuData> empty_imu_data_;
      common::MapByTime<common::OdometryData> odometry_data_;
      common::MapByTime<common::FixedFramePoseData> fixed_frame_pose_data_;
      std::map<int, PoseGraphInterface::TrajectoryData> trajectory_data_;
    };

  } // namespace loop
} // namespace carto_slam
