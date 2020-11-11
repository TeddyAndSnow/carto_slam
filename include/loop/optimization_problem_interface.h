#pragma once

#include <map>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "common/time.h"
#include "estimator/id.h"
#include "loop/pose_graph_interface.h"
#include "common/basic_types.h"
#include "common/map_by_time.h"

namespace carto_slam
{
    namespace loop
    {

        // Implements the SPA loop closure method.
        template <typename NodeDataType, typename SubmapDataType,
                  typename RigidTransformType>
        class OptimizationProblemInterface
        {
        public:
            using Constraint = PoseGraphInterface::Constraint;
            using LandmarkNode = PoseGraphInterface::LandmarkNode;

            OptimizationProblemInterface() {}
            virtual ~OptimizationProblemInterface() {}

            OptimizationProblemInterface(const OptimizationProblemInterface &) = delete;
            OptimizationProblemInterface &operator=(const OptimizationProblemInterface &) =
                delete;

            virtual void AddImuData(int trajectory_id,
                                    const common::ImuData &imu_data) = 0;
            virtual void AddOdometryData(int trajectory_id,
                                         const common::OdometryData &odometry_data) = 0;
            virtual void AddTrajectoryNode(int trajectory_id,
                                           const NodeDataType &node_data) = 0;
            virtual void InsertTrajectoryNode(const NodeId &node_id,
                                              const NodeDataType &node_data) = 0;
            virtual void TrimTrajectoryNode(const NodeId &node_id) = 0;
            virtual void AddSubmap(int trajectory_id,
                                   const RigidTransformType &global_submap_pose) = 0;
            virtual void InsertSubmap(const SubmapId &submap_id,
                                      const RigidTransformType &global_submap_pose) = 0;
            virtual void TrimSubmap(const SubmapId &submap_id) = 0;
            virtual void SetMaxNumIterations(int32 max_num_iterations) = 0;

            // Optimizes the global poses.
            virtual void Solve(
                const std::vector<Constraint> &constraints,
                const std::map<int, PoseGraphInterface::TrajectoryState> &
                    trajectories_state,
                const std::map<std::string, LandmarkNode> &landmark_nodes) = 0;

            virtual const MapById<NodeId, NodeDataType> &node_data() const = 0;
            virtual const MapById<SubmapId, SubmapDataType> &submap_data() const = 0;
            virtual const std::map<std::string, common::Rigid3d> &landmark_data()
                const = 0;
            virtual const common::MapByTime<common::ImuData> &imu_data() const = 0;
            virtual const common::MapByTime<common::OdometryData> &odometry_data()
                const = 0;
        };

    } // namespace loop
} // namespace carto_slam
