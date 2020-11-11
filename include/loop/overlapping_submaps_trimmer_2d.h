#pragma once

#include "common/time.h"
#include "loop/pose_graph_trimmer.h"

namespace carto_slam
{
  namespace loop
  {

    // Trims submaps that have less than 'min_covered_cells_count' cells not
    // overlapped by at least 'fresh_submaps_count` submaps.
    class OverlappingSubmapsTrimmer2D : public PoseGraphTrimmer
    {
    public:
      OverlappingSubmapsTrimmer2D(uint16 fresh_submaps_count,
                                  double min_covered_area,
                                  uint16 min_added_submaps_count)
          : fresh_submaps_count_(fresh_submaps_count),
            min_covered_area_(min_covered_area),
            min_added_submaps_count_(min_added_submaps_count) {}
      ~OverlappingSubmapsTrimmer2D() override = default;

      void Trim(Trimmable *pose_graph) override;
      bool IsFinished() override { return finished_; }

    private:
      // Number of the most recent submaps to keep.
      const uint16 fresh_submaps_count_;
      // Minimum area of covered space to keep submap from trimming measured in m^2.
      const double min_covered_area_;
      // Number of added submaps before the trimmer is invoked.
      const uint16 min_added_submaps_count_;
      // Current finished submap count.
      uint16 current_submap_count_ = 0;

      bool finished_ = false;
    };

  } // namespace loop
} // namespace carto_slam
