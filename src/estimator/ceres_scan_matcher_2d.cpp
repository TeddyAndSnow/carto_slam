
#include "estimator/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include <Eigen/Core>
#include "estimator/2d/grid_2d.h"
#include "estimator/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "estimator/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "estimator/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "estimator/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "common/transform.h"
#include <ceres/ceres.h>

namespace carto_slam
{
    namespace estimator
    {

        CeresScanMatcher2D::CeresScanMatcher2D(
            const Config &options)
            : options_(options)
        {
            ceres_solver_options_.use_nonmonotonic_steps = options_.use_nonmonotonic_steps_;
            ceres_solver_options_.max_num_iterations = options_.max_num_iterations_;
            ceres_solver_options_.num_threads = options_.num_threads_;
            ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
        }

        CeresScanMatcher2D::~CeresScanMatcher2D() {}

        void CeresScanMatcher2D::Match(const Eigen::Vector2d &target_translation,
                                       const common::Rigid2d &initial_pose_estimate,
                                       const common::PointCloud &point_cloud,
                                       const Grid2D &grid,
                                       common::Rigid2d *const pose_estimate,
                                       ceres::Solver::Summary *const summary) const
        {
            double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                             initial_pose_estimate.translation().y(),
                                             initial_pose_estimate.rotation().angle()};
            ceres::Problem problem;
            // CHECK_GT(options_.occupied_space_weight(), 0.);
            switch (grid.GetGridType())
            {
            case GridType::PROBABILITY_GRID:
                problem.AddResidualBlock(
                    CreateOccupiedSpaceCostFunction2D(options_.occupied_space_weight_ / std::sqrt(static_cast<double>(point_cloud.size())), point_cloud, grid), nullptr /* loss function */, ceres_pose_estimate);
                break;
            case GridType::TSDF:
                problem.AddResidualBlock(
                    CreateTSDFMatchCostFunction2D(options_.occupied_space_weight_ / std::sqrt(static_cast<double>(point_cloud.size())), point_cloud, static_cast<const TSDF2D &>(grid)), nullptr /* loss function */, ceres_pose_estimate);
                break;
            }
            // CHECK_GT(options_.translation_weight(), 0.);
            problem.AddResidualBlock(
                TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(options_.translation_weight_, target_translation), nullptr /* loss function */, ceres_pose_estimate);
            // CHECK_GT(options_.rotation_weight(), 0.);
            problem.AddResidualBlock(
                RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(options_.rotation_weight_, ceres_pose_estimate[2]), nullptr /* loss function */, ceres_pose_estimate);

            ceres::Solve(ceres_solver_options_, &problem, summary);

            *pose_estimate = common::Rigid2d(
                {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
        }

    } // namespace estimator
} // namespace carto_slam
