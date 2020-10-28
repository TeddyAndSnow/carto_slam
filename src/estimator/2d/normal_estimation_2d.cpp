
#include "estimator/2d/normal_estimation_2d.h"

namespace carto_slam
{
  namespace estimator
  {
    namespace
    {

      float NormalTo2DAngle(const Eigen::Vector3f &v)
      {
        return std::atan2(v[1], v[0]);
      }

      // Estimate the normal of an estimation_point as the arithmetic mean of the the
      // normals of the vectors from estimation_point to each point in the
      // sample_window.
      float EstimateNormal(const common::PointCloud &returns,
                           const size_t estimation_point_index,
                           const size_t sample_window_begin,
                           const size_t sample_window_end,
                           const Eigen::Vector3f &sensor_origin)
      {
        const Eigen::Vector3f &estimation_point =
            returns[estimation_point_index].position;
        if (sample_window_end - sample_window_begin < 2)
        {
          return NormalTo2DAngle(sensor_origin - estimation_point);
        }
        Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
        const Eigen::Vector3f &estimation_point_to_observation =
            sensor_origin - estimation_point;
        for (size_t sample_point_index = sample_window_begin;
             sample_point_index < sample_window_end; ++sample_point_index)
        {
          if (sample_point_index == estimation_point_index)
            continue;
          const Eigen::Vector3f &sample_point = returns[sample_point_index].position;
          const Eigen::Vector3f &tangent = estimation_point - sample_point;
          Eigen::Vector3f sample_normal = {-tangent[1], tangent[0], 0.f};
          constexpr float kMinNormalLength = 1e-6f;
          if (sample_normal.norm() < kMinNormalLength)
          {
            continue;
          }
          // Ensure sample_normal points towards 'sensor_origin'.
          if (sample_normal.dot(estimation_point_to_observation) < 0)
          {
            sample_normal = -sample_normal;
          }
          sample_normal.normalize();
          mean_normal += sample_normal;
        }
        return NormalTo2DAngle(mean_normal);
      }
    } // namespace

    // Estimates the normal for each 'return' in 'range_data'.
    // Assumes the angles in the range data returns are sorted with respect to
    // the orientation of the vector from 'origin' to 'return'.
    std::vector<float> EstimateNormals(
        const common::RangeData &range_data,
        const Config &normal_estimation_options)
    {
      std::vector<float> normals;
      normals.reserve(range_data.returns.size());
      const size_t max_num_samples = normal_estimation_options.num_normal_samples_;
      const float sample_radius = normal_estimation_options.sample_radius_;
      for (size_t current_point = 0; current_point < range_data.returns.size();
           ++current_point)
      {
        const Eigen::Vector3f &hit = range_data.returns[current_point].position;
        size_t sample_window_begin = current_point;
        for (; sample_window_begin > 0 &&
               current_point - sample_window_begin < max_num_samples / 2 &&
               (hit - range_data.returns[sample_window_begin - 1].position).norm() <
                   sample_radius;
             --sample_window_begin)
        {
        }
        size_t sample_window_end = current_point;
        for (;
             sample_window_end < range_data.returns.size() &&
             sample_window_end - current_point < ceil(max_num_samples / 2.0) + 1 &&
             (hit - range_data.returns[sample_window_end].position).norm() <
                 sample_radius;
             ++sample_window_end)
        {
        }
        const float normal_estimate =
            EstimateNormal(range_data.returns, current_point, sample_window_begin,
                           sample_window_end, range_data.origin);
        normals.push_back(normal_estimate);
      }
      return normals;
    }

  } // namespace estimator
} // namespace carto_slam
