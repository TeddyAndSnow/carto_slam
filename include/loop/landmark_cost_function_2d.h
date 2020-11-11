#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "loop/cost_helpers.h"
#include "loop/optimization_problem_2d.h"
#include "loop/pose_graph_interface.h"
#include "common/rigid_transform.h"
#include "common/transform.h"
#include <ceres/ceres.h>
//#include <ceres/jet.h>

namespace carto_slam
{
    namespace loop
    {

        // Cost function measuring the weighted error between the observed pose given by
        // the landmark measurement and the linearly interpolated pose of embedded in 3D
        // space node poses.
        class LandmarkCostFunction2D
        {
        public:
            using LandmarkObservation =
                PoseGraphInterface::LandmarkNode::LandmarkObservation;

            static ceres::CostFunction *CreateAutoDiffCostFunction(
                const LandmarkObservation &observation, const NodeSpec2D &prev_node,
                const NodeSpec2D &next_node)
            {
                return new ceres::AutoDiffCostFunction<
                    LandmarkCostFunction2D, 6 /* residuals */,
                    3 /* previous node pose variables */, 3 /* next node pose variables */,
                    4 /* landmark rotation variables */,
                    3 /* landmark translation variables */>(
                    new LandmarkCostFunction2D(observation, prev_node, next_node));
            }

            template <typename T>
            bool operator()(const T *const prev_node_pose, const T *const next_node_pose,
                            const T *const landmark_rotation,
                            const T *const landmark_translation, T *const e) const
            {
                const std::tuple<std::array<T, 4>, std::array<T, 3>>
                    interpolated_rotation_and_translation = InterpolateNodes2D(
                        prev_node_pose, prev_node_gravity_alignment_, next_node_pose,
                        next_node_gravity_alignment_, interpolation_parameter_);
                const std::array<T, 6> error = ScaleError(
                    ComputeUnscaledError(
                        landmark_to_tracking_transform_,
                        std::get<0>(interpolated_rotation_and_translation).data(),
                        std::get<1>(interpolated_rotation_and_translation).data(),
                        landmark_rotation, landmark_translation),
                    translation_weight_, rotation_weight_);
                std::copy(std::begin(error), std::end(error), e);
                return true;
            }

        private:
            LandmarkCostFunction2D(const LandmarkObservation &observation,
                                   const NodeSpec2D &prev_node,
                                   const NodeSpec2D &next_node)
                : landmark_to_tracking_transform_(
                      observation.landmark_to_tracking_transform),
                  prev_node_gravity_alignment_(prev_node.gravity_alignment),
                  next_node_gravity_alignment_(next_node.gravity_alignment),
                  translation_weight_(observation.translation_weight),
                  rotation_weight_(observation.rotation_weight),
                  interpolation_parameter_(
                      common::ToSeconds(observation.time - prev_node.time) /
                      common::ToSeconds(next_node.time - prev_node.time)) {}

            const common::Rigid3d landmark_to_tracking_transform_;
            const Eigen::Quaterniond prev_node_gravity_alignment_;
            const Eigen::Quaterniond next_node_gravity_alignment_;
            const double translation_weight_;
            const double rotation_weight_;
            const double interpolation_parameter_;
        };

    } // namespace loop
} // namespace carto_slam
