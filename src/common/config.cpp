#include "common/config.h"

#include <iostream>

using namespace carto_slam;

Config::Config() {

}

Config::~Config() {

}

std::string Config::get_config_filename() {
    return config_filepath_;
}

bool Config::set_config_filename(const std::string &config_filename) {
    this->node_ = YAML::LoadFile(config_filename);
    this->config_filepath_ = config_filename;
    if (this->node_.IsNull()) {
        std::cerr << "parameter file " << config_filename << " does not exist." << std::endl;
        return false;
    }

    laser_topic_ = "/rslidar_points";
    imu_topic_ = "/BMI088";
    is_imu_refine_ = true;

    occupied_space_weight_ = 1.0;
    translation_weight_ = 10.0;
    rotation_weight_ = 40.0;
    use_nonmonotonic_steps_ = false;
    max_num_iterations_ = 20;
    num_threads_ = 1;

    insert_free_space_ = true;
    hit_probability_ = 0.55;
    miss_probability_ = 0.49;

    num_normal_samples_ = 4;
    sample_radius_ = 0.5;

    truncation_distance_ = 0.3;
    maximum_weight_ = 10.0;
    update_free_space_ = false;
    project_sdf_distance_to_scan_normal_ = true;
    update_weight_range_exponent_ = 0;
    update_weight_angle_scan_normal_to_ray_kernel_bandwidth_ = 0.5;
    update_weight_distance_cell_to_hit_kernel_bandwidth_ = 0.5;

    num_range_data_ = 90;
    grid_options_2d_resolution_ = 0.05;
    grid_options_2d_grid_type_ = "PROBABILITY_GRID";
    grid_options_2d_range_data_inserter_type_ = "PROBABILITY_GRID_INSERTER_2D";
    range_data_inserter_truncation_distance_ = 0.3;
    range_data_inserter_maximum_weight_ = 10.0;

    max_time_seconds_ = 5.0;
    max_distance_meters_ = 0.2;
    max_angle_radians_ = 1.0;

    adaptive_voxel_filter_min_num_points_ = 200;
    adaptive_voxel_filter_max_length_ = 0.9;
    adaptive_voxel_filter_max_range_ = 50.0;

    min_z_ = -0.8;
    max_z_ = 2.0;
    voxel_filter_size_ = 0.025;
    use_online_correlative_scan_matching_ = false;
    use_imu_data_ = true;
    min_range_ = 0.0;
    max_range_ = 30.0;
    missing_data_ray_length_ = 5.0;
    num_accumulated_range_data_ = 1;

    imu_gravity_time_constant_ = 10.0;
    pose_queue_duration_ = 0.001;

    ///////////////////////////////////////////////////////////////////
    huber_scale_ = 1e1;
    odometry_translation_weight_ = 1e5;
    odometry_rotation_weight_ = 1e5;
    local_slam_pose_translation_weight_ = 1e5;
    local_slam_pose_rotation_weight_ = 1e5;
    fixed_frame_pose_translation_weight_ = 1e1;
    fixed_frame_pose_rotation_weight_ = 1e2;
    fixed_frame_pose_use_tolerant_loss_ = false;
    fixed_frame_pose_tolerant_loss_param_a_ = 1;
    fixed_frame_pose_tolerant_loss_param_b_ = 1;
    optimization_problem_use_nonmonotonic_steps_ = false;
    optimization_problem_max_num_iterations_ = 50;
    optimization_problem_num_threads_ = 7;
    log_solver_summary_ = false;

    fast_correlative_scan_matcher_branch_and_bound_depth_ = 7;
    fast_correlative_scan_matcher_linear_search_window_ = 7.;
    fast_correlative_scan_matcher_angular_search_window_ = 30.0 * M_PI / 180.0;

    constraint_builder_max_constraint_distance_ = 15.;
    constraint_builder_sampling_ratio_ = 0.3;
    constraint_builder_min_score_ = 0.55;
    constraint_builder_global_localization_min_score_ = 0.6;
    constraint_builder_log_matches_ = true;
    constraint_builder_loop_closure_translation_weight_ = 1.1e4;
    constraint_builder_loop_closure_rotation_weight_ = 1e5;

    pose_graph_global_sampling_ratio_ = 0.003;
    pose_graph_global_constraint_search_after_n_seconds_ = 10.;
    pose_graph_matcher_translation_weight_ = 5e2;
    pose_graph_matcher_rotation_weight_ = 1.6e3;
    pose_graph_optimize_every_n_nodes_ = 90;
    pose_graph_max_num_final_iterations_ = 200;
    optimization_problem_ceres_solver_options_max_num_iterations_ = 50;
    
    num_background_threads_ = 4;
}