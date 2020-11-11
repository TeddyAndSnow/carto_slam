
#include "estimator/range_data_collator.h"

#include <memory>
// #include "cartographer/mapping/local_slam_result_data.h"

namespace carto_slam
{
  namespace estimator
  {

    constexpr float RangeDataCollator::kDefaultIntensityValue;

    common::TimedPointCloudOriginData RangeDataCollator::AddRangeData(const std::string &sensor_id, common::TimedPointCloudData timed_point_cloud_data)
    {
      // CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
      timed_point_cloud_data.intensities.resize(timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);
      // TODO(gaschler): These two cases can probably be one.
      if (id_to_pending_data_.count(sensor_id) != 0)
      {
        current_start_ = current_end_;
        // When we have two messages of the same sensor, move forward the older of
        // the two (do not send out current).
        current_end_ = id_to_pending_data_.at(sensor_id).time;
        auto result = CropAndMerge();
        id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
        return result;
      }
      id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
      if (expected_sensor_ids_.size() != id_to_pending_data_.size())
      {
        return {};
      }
      current_start_ = current_end_;
      // We have messages from all sensors, move forward to oldest.
      common::Time oldest_timestamp = common::Time::max();
      for (const auto &pair : id_to_pending_data_)
      {
        oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
      }
      current_end_ = oldest_timestamp;
      return CropAndMerge();
    }

    common::TimedPointCloudOriginData RangeDataCollator::CropAndMerge()
    {
      common::TimedPointCloudOriginData result{current_end_, {}, {}};
      bool warned_for_dropped_points = false;
      for (auto it = id_to_pending_data_.begin();
           it != id_to_pending_data_.end();)
      {
        common::TimedPointCloudData &data = it->second;
        const common::TimedPointCloud &ranges = it->second.ranges;
        const std::vector<float> &intensities = it->second.intensities;

        auto overlap_begin = ranges.begin();
        while (overlap_begin < ranges.end() &&
               data.time + common::FromSeconds((*overlap_begin).time) <
                   current_start_)
        {
          ++overlap_begin;
        }
        auto overlap_end = overlap_begin;
        while (overlap_end < ranges.end() &&
               data.time + common::FromSeconds((*overlap_end).time) <=
                   current_end_)
        {
          ++overlap_end;
        }
        if (ranges.begin() < overlap_begin && !warned_for_dropped_points)
        {
          // LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin) << " earlier points.";
          warned_for_dropped_points = true;
        }

        // Copy overlapping range.
        if (overlap_begin < overlap_end)
        {
          std::size_t origin_index = result.origins.size();
          result.origins.push_back(data.origin);
          const float time_correction =
              static_cast<float>(common::ToSeconds(data.time - current_end_));
          auto intensities_overlap_it =
              intensities.begin() + (overlap_begin - ranges.begin());
          result.ranges.reserve(result.ranges.size() +
                                std::distance(overlap_begin, overlap_end));
          for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
               ++overlap_it, ++intensities_overlap_it)
          {
            common::TimedPointCloudOriginData::RangeMeasurement point{
                *overlap_it, *intensities_overlap_it, origin_index};
            // current_end_ + point_time[3]_after == in_timestamp +
            // point_time[3]_before
            point.point_time.time += time_correction;
            result.ranges.push_back(point);
          }
        }

        // Drop buffered points until overlap_end.
        if (overlap_end == ranges.end())
        {
          it = id_to_pending_data_.erase(it);
        }
        else if (overlap_end == ranges.begin())
        {
          ++it;
        }
        else
        {
          const auto intensities_overlap_end =
              intensities.begin() + (overlap_end - ranges.begin());
          data = common::TimedPointCloudData{
              data.time, data.origin,
              common::TimedPointCloud(overlap_end, ranges.end()),
              std::vector<float>(intensities_overlap_end, intensities.end())};
          ++it;
        }
      }

      std::sort(result.ranges.begin(), result.ranges.end(),
                [](const common::TimedPointCloudOriginData::RangeMeasurement &a,
                   const common::TimedPointCloudOriginData::RangeMeasurement &b) {
                  return a.point_time.time < b.point_time.time;
                });
      return result;
    }

  } // namespace estimator
} // namespace carto_slam
