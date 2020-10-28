
#include "estimator/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "estimator/2d/xy_index.h"
#include "estimator/2d/ray_to_pixel_mask.h"
#include "estimator/probability_values.h"

namespace carto_slam
{
  namespace estimator
  {
    namespace
    {

      // Factor for subpixel accuracy of start and end point for ray casts.
      constexpr int kSubpixelScale = 1000;

      void GrowAsNeeded(const common::RangeData &range_data,
                        ProbabilityGrid *const probability_grid)
      {
        Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
        // Padding around bounding box to avoid numerical issues at cell boundaries.
        constexpr float kPadding = 1e-6f;
        for (const common::RangefinderPoint &hit : range_data.returns)
        {
          bounding_box.extend(hit.position.head<2>());
        }
        for (const common::RangefinderPoint &miss : range_data.misses)
        {
          bounding_box.extend(miss.position.head<2>());
        }
        probability_grid->GrowLimits(bounding_box.min() -
                                     kPadding * Eigen::Vector2f::Ones());
        probability_grid->GrowLimits(bounding_box.max() +
                                     kPadding * Eigen::Vector2f::Ones());
      }

      void CastRays(const common::RangeData &range_data,
                    const std::vector<uint16> &hit_table,
                    const std::vector<uint16> &miss_table,
                    const bool insert_free_space, ProbabilityGrid *probability_grid)
      {
        GrowAsNeeded(range_data, probability_grid);

        const MapLimits &limits = probability_grid->limits();
        const double superscaled_resolution = limits.resolution() / kSubpixelScale;
        const MapLimits superscaled_limits(
            superscaled_resolution, limits.max(),
            CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                       limits.cell_limits().num_y_cells * kSubpixelScale));
        const Eigen::Array2i begin =
            superscaled_limits.GetCellIndex(range_data.origin.head<2>());
        // Compute and add the end points.
        std::vector<Eigen::Array2i> ends;
        ends.reserve(range_data.returns.size());
        for (const common::RangefinderPoint &hit : range_data.returns)
        {
          ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
          probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
        }

        if (!insert_free_space)
        {
          return;
        }

        // Now add the misses.
        for (const Eigen::Array2i &end : ends)
        {
          std::vector<Eigen::Array2i> ray =
              RayToPixelMask(begin, end, kSubpixelScale);
          for (const Eigen::Array2i &cell_index : ray)
          {
            probability_grid->ApplyLookupTable(cell_index, miss_table);
          }
        }

        // Finally, compute and add empty rays based on misses in the range data.
        for (const common::RangefinderPoint &missing_echo : range_data.misses)
        {
          std::vector<Eigen::Array2i> ray = RayToPixelMask(
              begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
              kSubpixelScale);
          for (const Eigen::Array2i &cell_index : ray)
          {
            probability_grid->ApplyLookupTable(cell_index, miss_table);
          }
        }
      }
    } // namespace

    ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
        const Config &options)
        : options_(options),
          hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(options.hit_probability_))),
          miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(options.miss_probability_))) {}

    void ProbabilityGridRangeDataInserter2D::Insert(const common::RangeData &range_data, GridInterface *const grid) const
    {
      ProbabilityGrid *const probability_grid = static_cast<ProbabilityGrid *>(grid);
      // CHECK(probability_grid != nullptr);
      // By not finishing the update after hits are inserted, we give hits priority
      // (i.e. no hits will be ignored because of a miss in the same cell).
      CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space_, probability_grid);
      probability_grid->FinishUpdate();
    }

  } // namespace estimator
} // namespace carto_slam
