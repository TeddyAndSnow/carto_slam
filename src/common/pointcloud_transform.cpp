#include "common/pointcloud_transform.h"

namespace carto_slam
{
    namespace common
    {

        PointCloud TransformPointCloud(const PointCloud &point_cloud,
                                       const common::Rigid3f &transform)
        {
            PointCloud result;
            result.reserve(point_cloud.size());
            for (const RangefinderPoint &point : point_cloud)
            {
                result.emplace_back(transform * point);
            }
            return result;
        }

        TimedPointCloud TransformTimedPointCloud(const TimedPointCloud &point_cloud,
                                                 const common::Rigid3f &transform)
        {
            TimedPointCloud result;
            result.reserve(point_cloud.size());
            for (const TimedRangefinderPoint &point : point_cloud)
            {
                result.push_back(transform * point);
            }
            return result;
        }

        PointCloud CropPointCloud(const PointCloud &point_cloud, const float min_z,
                                  const float max_z)
        {
            PointCloud cropped_point_cloud;
            for (const RangefinderPoint &point : point_cloud)
            {
                if (min_z <= point.position.z() && point.position.z() <= max_z)
                {
                    cropped_point_cloud.push_back(point);
                }
            }
            return cropped_point_cloud;
        }
    } // namespace common
} // namespace carto_slam