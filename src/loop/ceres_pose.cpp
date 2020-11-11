
#include "loop/ceres_pose.h"

namespace carto_slam
{
  namespace loop
  {

    CeresPose::Data FromPose(const common::Rigid3d &pose)
    {
      return CeresPose::Data{{{pose.translation().x(), pose.translation().y(),
                               pose.translation().z()}},
                             {{pose.rotation().w(), pose.rotation().x(),
                               pose.rotation().y(), pose.rotation().z()}}};
    }

    CeresPose::CeresPose(
        const common::Rigid3d &pose,
        std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
        std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
        ceres::Problem *problem)
        : data_(std::make_shared<CeresPose::Data>(FromPose(pose)))
    {
      problem->AddParameterBlock(data_->translation.data(), 3,
                                 translation_parametrization.release());
      problem->AddParameterBlock(data_->rotation.data(), 4,
                                 rotation_parametrization.release());
    }

    const common::Rigid3d CeresPose::ToRigid() const
    {
      return common::Rigid3d::FromArrays(data_->rotation, data_->translation);
    }

  } // namespace loop
} // namespace carto_slam
