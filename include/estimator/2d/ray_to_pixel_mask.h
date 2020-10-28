#pragma once

#include <vector>

#include "common/transform.h"

namespace carto_slam
{
    namespace estimator
    {

        // Compute all pixels that contain some part of the line segment connecting
        // 'scaled_begin' and 'scaled_end'. 'scaled_begin' and 'scaled_end' are scaled
        // by 'subpixel_scale'. 'scaled_begin' and 'scaled_end' are expected to be
        // greater than zero. Return values are in pixels and not scaled.
        std::vector<Eigen::Array2i> RayToPixelMask(const Eigen::Array2i &scaled_begin,
                                                   const Eigen::Array2i &scaled_end,
                                                   int subpixel_scale);

    } // namespace estimator
} // namespace carto_slam
