#pragma once

#include <Eigen/Core>
#include "rigid_transform.h"
#include "time.h"

namespace carto_slam
{
    namespace common
    {

        // Stores 3D position of a point observed by a rangefinder sensor.
        struct RangefinderPoint
        {
            Eigen::Vector3f position;
        };
        // Stores 3D position of a point with its relative measurement time.
        // See point_cloud.h for more details.
        struct TimedRangefinderPoint
        {
            Eigen::Vector3f position;
            float time;
        };

        using PointCloud = std::vector<RangefinderPoint>;
        using TimedPointCloud = std::vector<TimedRangefinderPoint>;
        struct PointCloudWithIntensities
        {
            TimedPointCloud points;
            std::vector<float> intensities;
        };

        struct TimedPointCloudData
        {
            common::Time time;
            Eigen::Vector3f origin;
            TimedPointCloud ranges;
            // 'intensities' has to be same size as 'ranges', or empty.
            std::vector<float> intensities;
        };

        struct TimedPointCloudOriginData
        {
            struct RangeMeasurement
            {
                TimedRangefinderPoint point_time;
                float intensity;
                size_t origin_index;
            };
            common::Time time;
            std::vector<Eigen::Vector3f> origins;
            std::vector<RangeMeasurement> ranges;
        };

        struct ImuData
        {
            common::Time time;
            Eigen::Vector3d linear_acceleration;
            Eigen::Vector3d angular_velocity;
        };

        struct OdometryData
        {
            common::Time time;
            common::Rigid3d pose;
        };

    } // namespace common
} // namespace carto_slam