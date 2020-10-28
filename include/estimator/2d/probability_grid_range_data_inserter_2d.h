#pragma once

#include <utility>
#include <vector>

#include "common/time.h"
#include "common/basic_types.h"
#include "estimator/2d/probability_grid.h"
#include "estimator/2d/xy_index.h"
#include "estimator/range_data_inserter_interface.h"
#include "common/basic_types.h"
#include "common/config.h"

namespace carto_slam
{
    namespace estimator
    {
        class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface
        {
        public:
            explicit ProbabilityGridRangeDataInserter2D(const Config &options);

            ProbabilityGridRangeDataInserter2D(const ProbabilityGridRangeDataInserter2D &) = delete;
            ProbabilityGridRangeDataInserter2D &operator=(const ProbabilityGridRangeDataInserter2D &) = delete;

            // Inserts 'range_data' into 'probability_grid'.
            virtual void Insert(const common::RangeData &range_data, GridInterface *grid) const override;

        private:
            const Config options_;
            const std::vector<uint16> hit_table_;
            const std::vector<uint16> miss_table_;
        };

    } // namespace estimator
} // namespace carto_slam
