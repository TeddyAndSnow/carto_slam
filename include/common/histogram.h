#pragma once

#include <string>
#include <vector>

#include "common/time.h"

namespace carto_slam
{
  namespace common
  {

    class Histogram
    {
    public:
      void Add(float value);
//      std::string ToString(int buckets) const;

    private:
      std::vector<float> values_;
    };

  } // namespace common
} // namespace carto_slam
