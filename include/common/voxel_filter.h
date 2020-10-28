#pragma once

#include <bitset>

#include "common/basic_types.h"
#include "common/config.h"

namespace carto_slam
{
    namespace common
    {
        PointCloud VoxelFilter(const PointCloud &point_cloud, const float resolution);
        TimedPointCloud VoxelFilter(const TimedPointCloud &timed_point_cloud,
                                    const float resolution);
        std::vector<common::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
            const std::vector<common::TimedPointCloudOriginData::RangeMeasurement> &
                range_measurements,
            const float resolution);

        PointCloud AdaptiveVoxelFilter(
            const PointCloud &point_cloud,
            const Config &options);
    } // namespace common
} // namespace carto_slam

