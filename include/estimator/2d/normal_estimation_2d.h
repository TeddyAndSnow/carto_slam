#pragma once

#include <vector>

#include "common/basic_types.h"
#include "common/transform.h"
#include "common/config.h"

namespace carto_slam
{
    namespace estimator
    {

        // Estimates the normal for each 'return' in 'range_data'.
        // Assumes the angles in the range data returns are sorted with respect to
        // the orientation of the vector from 'origin' to 'return'.
        std::vector<float> EstimateNormals(
            const common::RangeData &range_data,
            const Config &normal_estimation_options);

    } // namespace estimator
} // namespace carto_slam
