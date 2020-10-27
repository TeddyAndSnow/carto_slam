#pragma once

#include "rigid_transform.h"
#include "basic_types.h"

namespace carto_slam
{
    namespace common
    {

        template <class T>
        inline RangefinderPoint operator*(const common::Rigid3<T> &lhs,
                                          const RangefinderPoint &rhs)
        {
            RangefinderPoint result = rhs;
            result.position = lhs * rhs.position;
            return result;
        }

        template <class T>
        inline TimedRangefinderPoint operator*(const common::Rigid3<T> &lhs,
                                               const TimedRangefinderPoint &rhs)
        {
            TimedRangefinderPoint result = rhs;
            result.position = lhs * rhs.position;
            return result;
        }

        inline bool operator==(const RangefinderPoint &lhs,
                               const RangefinderPoint &rhs)
        {
            return lhs.position == rhs.position;
        }

        inline bool operator==(const TimedRangefinderPoint &lhs,
                               const TimedRangefinderPoint &rhs)
        {
            return lhs.position == rhs.position && lhs.time == rhs.time;
        }

        inline RangefinderPoint ToRangefinderPoint(
            const TimedRangefinderPoint &timed_rangefinder_point)
        {
            return {timed_rangefinder_point.position};
        }

        inline TimedRangefinderPoint ToTimedRangefinderPoint(
            const RangefinderPoint &rangefinder_point, const float time)
        {
            return {rangefinder_point.position, time};
        }

        // Transforms 'point_cloud' according to 'transform'.
        PointCloud TransformPointCloud(const PointCloud &point_cloud,
                                       const common::Rigid3f &transform);

        // Transforms 'point_cloud' according to 'transform'.
        TimedPointCloud TransformTimedPointCloud(const TimedPointCloud &point_cloud,
                                                 const common::Rigid3f &transform);

        // Returns a new point cloud without points that fall outside the region defined
        // by 'min_z' and 'max_z'.
        PointCloud CropPointCloud(const PointCloud &point_cloud, float min_z,
                                  float max_z);
    } // namespace common
} // namespace carto_slam