
#include "estimator/motion_filter.h"
#include "common/transform.h"

namespace carto_slam {
namespace estimator {

MotionFilter::MotionFilter(const Config& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const common::Time time,
                             const common::Rigid3d& pose) {
//   LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
    //   << "Motion filter reduced the number of nodes to "
    //   << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 &&
      time - last_time_ <= common::FromSeconds(options_.max_time_seconds_) &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters_ &&
      common::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians_) {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace mapping
}  // namespace cartographer
