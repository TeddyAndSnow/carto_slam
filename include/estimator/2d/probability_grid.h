#pragma once

#include <vector>
#include "common/time.h"
#include "estimator/2d/grid_2d.h"
#include "estimator/2d/map_limits.h"
#include "estimator/2d/xy_index.h"

namespace carto_slam
{
  namespace estimator
  {

    // Represents a 2D grid of probabilities.
    class ProbabilityGrid : public Grid2D
    {
    public:
      explicit ProbabilityGrid(const MapLimits &limits,
                               ValueConversionTables *conversion_tables);

      // Sets the probability of the cell at 'cell_index' to the given
      // 'probability'. Only allowed if the cell was unknown before.
      void SetProbability(const Eigen::Array2i &cell_index,
                          const float probability);

      // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
      // to the probability of the cell at 'cell_index' if the cell has not already
      // been updated. Multiple updates of the same cell will be ignored until
      // FinishUpdate() is called. Returns true if the cell was updated.
      //
      // If this is the first call to ApplyOdds() for the specified cell, its value
      // will be set to probability corresponding to 'odds'.
      bool ApplyLookupTable(const Eigen::Array2i &cell_index,
                            const std::vector<uint16> &table);

      GridType GetGridType() const override;

      // Returns the probability of the cell with 'cell_index'.
      float GetProbability(const Eigen::Array2i &cell_index) const;

      std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;

    private:
      ValueConversionTables *conversion_tables_;
    };

  } // namespace estimator
} // namespace carto_slam
