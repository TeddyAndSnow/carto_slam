#pragma once

#include "estimator/2d/tsdf_2d.h"
#include "estimator/range_data_inserter_interface.h"
#include "common/basic_types.h"
#include "common/config.h"

namespace carto_slam
{
    namespace estimator
    {

        class TSDFRangeDataInserter2D : public RangeDataInserterInterface
        {
        public:
            explicit TSDFRangeDataInserter2D(const Config &options);

            TSDFRangeDataInserter2D(const TSDFRangeDataInserter2D &) = delete;
            TSDFRangeDataInserter2D &operator=(const TSDFRangeDataInserter2D &) = delete;

            // Casts a ray from origin towards hit for each hit in range data.
            // If 'options.update_free_space' is 'true', all cells along the ray
            // until 'truncation_distance' behind hit are updated. Otherwise, only the
            // cells within 'truncation_distance' around hit are updated.
            virtual void Insert(const common::RangeData &range_data,
                                GridInterface *grid) const override;

        private:
            void InsertHit(const Config &options,
                           const Eigen::Vector2f &hit, const Eigen::Vector2f &origin,
                           float normal, TSDF2D *tsdf) const;
            void UpdateCell(const Eigen::Array2i &cell, float update_sdf,
                            float update_weight, TSDF2D *tsdf) const;
            const Config options_;
        };

    } // namespace estimator
} // namespace carto_slam
