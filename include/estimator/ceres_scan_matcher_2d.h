#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>
// #include "cartographer/common/lua_parameter_dictionary.h"
#include "estimator/2d/grid_2d.h"
#include "common/config.h"

namespace carto_slam
{
  namespace estimator
  {

    // Align scans with an existing map using Ceres.
    class CeresScanMatcher2D
    {
    public:
      explicit CeresScanMatcher2D(const Config &options);
      virtual ~CeresScanMatcher2D();

      CeresScanMatcher2D(const CeresScanMatcher2D &) = delete;
      CeresScanMatcher2D &operator=(const CeresScanMatcher2D &) = delete;

      // Aligns 'point_cloud' within the 'grid' given an
      // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
      // 'summary'.
      void Match(const Eigen::Vector2d &target_translation,
                 const common::Rigid2d &initial_pose_estimate,
                 const common::PointCloud &point_cloud, const Grid2D &grid,
                 common::Rigid2d *pose_estimate,
                 ceres::Solver::Summary *summary) const;

    private:
      const Config options_;
      ceres::Solver::Options ceres_solver_options_;
    };

  } // namespace estimator
} // namespace carto_slam
