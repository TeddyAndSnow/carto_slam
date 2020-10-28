#pragma once

#include <utility>
#include <vector>

#include "estimator/grid_interface.h"
#include "common/basic_types.h"

namespace carto_slam
{
    namespace estimator
    {
        class RangeDataInserterInterface
        {
        public:
            virtual ~RangeDataInserterInterface() {}

            // Inserts 'range_data' into 'grid'.
            virtual void Insert(const common::RangeData &range_data,
                                GridInterface *grid) const = 0;
        };

    } // namespace estimator
} // namespace carto_slam
