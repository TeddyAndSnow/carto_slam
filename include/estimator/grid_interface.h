#pragma once

#include <memory>
#include <utility>

namespace carto_slam
{
  namespace estimator
  {

    class GridInterface
    {
      // todo(kdaun) move mutual functions of Grid2D/3D here
    public:
      virtual ~GridInterface() {}
    };

  } // namespace estimator
} // namespace carto_slam
