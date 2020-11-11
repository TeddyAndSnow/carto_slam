#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

// #include "absl/container/flat_hash_map.h"
// #include "cartographer/common/lua_parameter_dictionary.h"
#include "estimator/id.h"
#include "loop/pose_graph_interface.h"
#include "loop/pose_graph_trimmer.h"
// #include "cartographer/mapping/proto/pose_graph.pb.h"
// #include "cartographer/mapping/proto/pose_graph_options.pb.h"
// #include "cartographer/mapping/proto/serialization.pb.h"
#include "estimator/submaps.h"
#include "estimator/trajectory_node.h"

#include "estimator/2d/submap_2d.h"
// #include "cartographer/sensor/fixed_frame_pose_data.h"
// #include "cartographer/sensor/imu_data.h"
// #include "cartographer/sensor/landmark_data.h"
#include "common/map_by_time.h"
#include "common/basic_types.h"
// #include "cartographer/sensor/odometry_data.h"

namespace carto_slam
{
    using namespace estimator;
    namespace loop
    {
        // proto::PoseGraphOptions CreatePoseGraphOptions(common::LuaParameterDictionary *const parameter_dictionary);

        class PoseGraph : public PoseGraphInterface
        {
        public:
            typedef std::shared_ptr<PoseGraph> Ptr;
            struct InitialTrajectoryPose
            {
                int to_trajectory_id;
                common::Rigid3d relative_pose;
                common::Time time;
            };

            PoseGraph() {}
            ~PoseGraph() override {}

            PoseGraph(const PoseGraph &) = delete;
            PoseGraph &operator=(const PoseGraph &) = delete;

            virtual NodeId AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data,
                                   int trajectory_id,
                                   const std::vector<std::shared_ptr<const Submap2D>> &insertion_submaps) = 0;

            // Inserts an IMU measurement.
            virtual void AddImuData(int trajectory_id,
                                    const common::ImuData &imu_data) = 0;

            // Inserts an odometry measurement.
            virtual void AddOdometryData(int trajectory_id,
                                         const common::OdometryData &odometry_data) = 0;

            // Inserts a fixed frame pose measurement.
            virtual void AddFixedFramePoseData(
                int trajectory_id,
                const common::FixedFramePoseData &fixed_frame_pose_data) = 0;

            // Inserts landmarks observations.
            virtual void AddLandmarkData(int trajectory_id,
                                         const common::LandmarkData &landmark_data) = 0;

            // Finishes the given trajectory.
            virtual void FinishTrajectory(int trajectory_id) = 0;

            // Freezes a trajectory. Poses in this trajectory will not be optimized.
            virtual void FreezeTrajectory(int trajectory_id) = 0;

            // Adds a 'submap' from a proto with the given 'global_pose' to the
            // appropriate trajectory.
            // virtual void AddSubmapFromProto(const transform::Rigid3d &global_pose,const proto::Submap &submap) = 0;

            // Adds a 'node' from a proto with the given 'global_pose' to the
            // appropriate trajectory.
            // virtual void AddNodeFromProto(const transform::Rigid3d &global_pose, const proto::Node &node) = 0;

            // Sets the trajectory data from a proto.
            // virtual void SetTrajectoryDataFromProto(const mapping::proto::TrajectoryData &data) = 0;

            // Adds information that 'node_id' was inserted into 'submap_id'. The submap
            // has to be deserialized first.
            virtual void AddNodeToSubmap(const NodeId &node_id, const SubmapId &submap_id) = 0;

            // Adds serialized constraints. The corresponding trajectory nodes and submaps
            // have to be deserialized before calling this function.
            virtual void AddSerializedConstraints(const std::vector<Constraint> &constraints) = 0;

            // Adds a 'trimmer'. It will be used after all data added before it has been
            // included in the pose graph.
            virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

            // Gets the current trajectory clusters.
            virtual std::vector<std::vector<int>> GetConnectedTrajectories() const = 0;

            // Returns the current optimized transform and submap itself for the given
            // 'submap_id'. Returns 'nullptr' for the 'submap' member if the submap does
            // not exist (anymore).
            virtual SubmapData GetSubmapData(const SubmapId &submap_id) const = 0;

            // proto::PoseGraph ToProto(bool include_unfinished_submaps) const override;

            // Returns the IMU data.
            virtual common::MapByTime<common::ImuData> GetImuData() const = 0;

            // Returns the odometry data.
            virtual common::MapByTime<common::OdometryData> GetOdometryData() const = 0;

            // Returns the fixed frame pose data.
            virtual common::MapByTime<common::FixedFramePoseData> GetFixedFramePoseData()
                const = 0;

            // Returns the landmark data.
            virtual std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
            GetLandmarkNodes() const = 0;

            // Sets a relative initial pose 'relative_pose' for 'from_trajectory_id' with
            // respect to 'to_trajectory_id' at time 'time'.
            virtual void SetInitialTrajectoryPose(int from_trajectory_id,
                                                  int to_trajectory_id,
                                                  const common::Rigid3d &pose,
                                                  const common::Time time) = 0;
        };

        // std::vector<PoseGraph::Constraint> FromProto(const ::google::protobuf::RepeatedPtrField<
        //                                              ::cartographer::mapping::proto::PoseGraph::Constraint> &
        //                                                  constraint_protos);
        // proto::PoseGraph::Constraint ToProto(const PoseGraph::Constraint &constraint);

    } // namespace loop
} // namespace carto_slam
