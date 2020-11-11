#pragma once

#include <vector>

#include "common/time.h"
#include "estimator/2d/grid_2d.h"
#include "estimator/2d/map_limits.h"
#include "estimator/2d/tsd_value_converter.h"
#include "estimator/2d/xy_index.h"
#include "io/submap_painter.h"
namespace carto_slam
{
    using namespace common;
    namespace estimator
    {

        // Represents a 2D grid of truncated signed distances and weights.
        class TSDF2D : public Grid2D
        {
        public:
            TSDF2D(const MapLimits &limits, float truncation_distance, float max_weight,
                   ValueConversionTables *conversion_tables);

            void SetCell(const Eigen::Array2i &cell_index, const float tsd,
                         const float weight);
            GridType GetGridType() const override;
            float GetTSD(const Eigen::Array2i &cell_index) const;
            float GetWeight(const Eigen::Array2i &cell_index) const;
            std::pair<float, float> GetTSDAndWeight(
                const Eigen::Array2i &cell_index) const;

            void GrowLimits(const Eigen::Vector2f &point) override;
            std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
            bool CellIsUpdated(const Eigen::Array2i &cell_index) const;
            bool DrawToSubmapTexture(io::SubmapTextureData *const texture,
                                     common::Rigid3d local_pose) const override;

        private:
            ValueConversionTables *conversion_tables_;
            std::unique_ptr<TSDValueConverter> value_converter_;
            std::vector<uint16> weight_cells_; // Highest bit is update marker.
        };

    } // namespace estimator
} // namespace carto_slam
