#pragma once

#include "estimator/2d/grid_2d.h"
#include "common/basic_types.h"
#include <ceres/ceres.h>

namespace carto_slam
{
    namespace estimator
    {
        // Creates a cost function for matching the 'point_cloud' to the 'grid' with
        // a 'pose'. The cost increases with poorer correspondence of the grid and the
        // point observation (e.g. points falling into less occupied space).
        ceres::CostFunction *CreateOccupiedSpaceCostFunction2D(
            const double scaling_factor, const common::PointCloud &point_cloud,
            const Grid2D &grid);

    } // namespace estimator
} // namespace carto_slam
