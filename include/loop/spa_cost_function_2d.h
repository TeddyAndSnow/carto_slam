#pragma once

#include "loop/pose_graph_interface.h"
#include <ceres/ceres.h>

namespace carto_slam
{
    namespace loop
    {

        ceres::CostFunction *CreateAutoDiffSpaCostFunction(const PoseGraphInterface::Constraint::Pose &pose);

        ceres::CostFunction *CreateAnalyticalSpaCostFunction(const PoseGraphInterface::Constraint::Pose &pose);

    } // namespace loop
} // namespace carto_slam
