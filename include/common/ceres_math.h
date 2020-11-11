#pragma once

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace carto_slam
{
    namespace common
    {
        template <typename T>
        T atan2Ceres(const Eigen::Matrix<T, 2, 1> &vector)
        {
            return ceres::atan2(vector.y(), vector.x());
        }
    } // namespace common
} // namespace carto_slam