#pragma once

#include "estimator/2d/tsdf_2d.h"
#include "common/basic_types.h"
#include <ceres/ceres.h>

namespace carto_slam
{
    namespace estimator
    {

        // Creates a cost function for matching the 'point_cloud' in the 'grid' at
        // a 'pose'. The cost increases with the signed distance of the matched point
        // location in the 'grid'.
        ceres::CostFunction *CreateTSDFMatchCostFunction2D(
            const double scaling_factor, const common::PointCloud &point_cloud,
            const TSDF2D &grid);

    } // namespace estimator
} // namespace carto_slam
