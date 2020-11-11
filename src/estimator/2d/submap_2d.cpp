
#include "estimator/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>

#include <Eigen/Geometry>
#include "common/time.h"
#include "estimator/2d/probability_grid_range_data_inserter_2d.h"
#include "estimator/2d/tsdf_range_data_inserter_2d.h"
#include "estimator/range_data_inserter_interface.h"

namespace carto_slam
{
  namespace estimator
  {

    Submap2D::Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid,
                       ValueConversionTables *conversion_tables)
        : Submap(common::Rigid3d::Translation(
              Eigen::Vector3d(origin.x(), origin.y(), 0.))),
          conversion_tables_(conversion_tables)
    {
      grid_ = std::move(grid);
    }

    void Submap2D::InsertRangeData(
        const common::RangeData &range_data,
        const RangeDataInserterInterface *range_data_inserter)
    {
      // CHECK(grid_);
      // CHECK(!insertion_finished());
      range_data_inserter->Insert(range_data, grid_.get());
      set_num_range_data(num_range_data() + 1);
    }

    void Submap2D::Finish()
    {
      // CHECK(grid_);
      // CHECK(!insertion_finished());
      grid_ = grid_->ComputeCroppedGrid();
      set_insertion_finished(true);
    }

    ActiveSubmaps2D::ActiveSubmaps2D(const Config &options)
        : options_(options), range_data_inserter_(CreateRangeDataInserter()) {}

    std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const
    {
      return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                          submaps_.end());
    }

    std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
        const common::RangeData &range_data)
    {
      if (submaps_.empty() ||
          submaps_.back()->num_range_data() == options_.num_range_data_)
      {
        AddSubmap(range_data.origin.head<2>());
      }
      for (auto &submap : submaps_)
      {
        submap->InsertRangeData(range_data, range_data_inserter_.get());
      }
      if (submaps_.front()->num_range_data() == 2 * options_.num_range_data_)
      {
        submaps_.front()->Finish();
      }
      return submaps();
    }

    std::unique_ptr<RangeDataInserterInterface>
    ActiveSubmaps2D::CreateRangeDataInserter()
    {
      if (options_.grid_options_2d_range_data_inserter_type_ == "PROBABILITY_GRID_INSERTER_2D")
      {
        return std::make_unique<ProbabilityGridRangeDataInserter2D>(options_);
      }
      else if (options_.grid_options_2d_range_data_inserter_type_ == "TSDF_INSERTER_2D")
      {
        return std::make_unique<TSDFRangeDataInserter2D>(options_);
      }
      else
      {
        // LOG(FATAL) << "Unknown RangeDataInserterType.";
      }
    }

    std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(const Eigen::Vector2f &origin)
    {
      const int kInitialSubmapSize = 100;
      float resolution = options_.grid_options_2d_resolution_;
      if (options_.grid_options_2d_grid_type_ == "PROBABILITY_GRID")
      {
        return std::make_unique<ProbabilityGrid>(
            MapLimits(resolution,
                      origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
                      CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
            &conversion_tables_);
      }
      else if (options_.grid_options_2d_grid_type_ == "TSDF")
      {
        return std::make_unique<TSDF2D>(
            MapLimits(resolution,
                      origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
                      CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
            options_.range_data_inserter_truncation_distance_,
            options_.range_data_inserter_maximum_weight_,
            &conversion_tables_);
      }
      else
      {
        // LOG(FATAL) << "Unknown GridType.";
      }
    }

    void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f &origin)
    {
      if (submaps_.size() >= 2)
      {
        // This will crop the finished Submap before inserting a new Submap to
        // reduce peak memory usage a bit.
        // CHECK(submaps_.front()->insertion_finished());
        submaps_.erase(submaps_.begin());
      }
      submaps_.push_back(std::make_unique<Submap2D>(
          origin,
          std::unique_ptr<Grid2D>(
              static_cast<Grid2D *>(CreateGrid(origin).release())),
          &conversion_tables_));
    }

  } // namespace estimator
} // namespace carto_slam
