#pragma once

#include <limits>

#include "common/time.h"
#include "common/rigid_transform.h"
#include "common/config.h"

namespace carto_slam {
namespace estimator {

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter(const Config& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const common::Rigid3d& pose);

 private:
  const Config options_;
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  common::Rigid3d last_pose_;
};

}  // namespace mapping
}  // namespace cartographer

