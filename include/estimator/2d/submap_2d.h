#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include "estimator/2d/grid_2d.h"
#include "estimator/2d/map_limits.h"
#include "estimator/range_data_inserter_interface.h"
#include "estimator/submaps.h"
#include "estimator/value_conversion_tables.h"
#include "common/basic_types.h"
#include "common/rigid_transform.h"
#include "common/config.h"

namespace carto_slam
{
  namespace estimator
  {

    class Submap2D : public Submap
    {
    public:
      Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid,
               ValueConversionTables *conversion_tables);

      const Grid2D *grid() const { return grid_.get(); }

      // Insert 'range_data' into this submap using 'range_data_inserter'. The
      // submap must not be finished yet.
      void InsertRangeData(const common::RangeData &range_data,
                           const RangeDataInserterInterface *range_data_inserter);
      void Finish();

    private:
      std::unique_ptr<Grid2D> grid_;
      ValueConversionTables *conversion_tables_;
    };

    // The first active submap will be created on the insertion of the first range
    // data. Except during this initialization when no or only one single submap
    // exists, there are always two submaps into which range data is inserted: an
    // old submap that is used for matching, and a new one, which will be used for
    // matching next, that is being initialized.
    //
    // Once a certain number of range data have been inserted, the new submap is
    // considered initialized: the old submap is no longer changed, the "new" submap
    // is now the "old" submap and is used for scan-to-map matching. Moreover, a
    // "new" submap gets created. The "old" submap is forgotten by this object.
    class ActiveSubmaps2D
    {
    public:
      explicit ActiveSubmaps2D(const Config &options);

      ActiveSubmaps2D(const ActiveSubmaps2D &) = delete;
      ActiveSubmaps2D &operator=(const ActiveSubmaps2D &) = delete;

      // Inserts 'range_data' into the Submap collection.
      std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
          const common::RangeData &range_data);

      std::vector<std::shared_ptr<const Submap2D>> submaps() const;

    private:
      std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
      std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f &origin);
      void FinishSubmap();
      void AddSubmap(const Eigen::Vector2f &origin);

      const Config options_;
      std::vector<std::shared_ptr<Submap2D>> submaps_;
      std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
      ValueConversionTables conversion_tables_;
    };

  } // namespace estimator
} // namespace carto_slam
