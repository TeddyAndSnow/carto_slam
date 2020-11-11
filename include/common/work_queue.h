#pragma once

#include <chrono>
#include <deque>
#include <functional>

namespace carto_slam
{
  namespace common
  {

    struct WorkItem
    {
      enum class Result
      {
        kDoNotRunOptimization,
        kRunOptimization,
      };

      std::chrono::steady_clock::time_point time;
      std::function<Result()> task;
    };

    using WorkQueue = std::deque<WorkItem>;

  } // namespace common
} // namespace carto_slam
