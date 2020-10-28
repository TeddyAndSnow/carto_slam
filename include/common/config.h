/**
 * Created by Xiong Weicheng(wchxiong@126.com) on 10/26/20.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <Eigen/Dense>
#include <regex>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace carto_slam {
    /** \brief Config for parameters reading form .yaml file
     * 
    */
   class Config {
    public:
       typedef std::shared_ptr<Config> Ptr;
        Config();
        ~Config();
        bool set_config_filename(const std::string &config_filename);

        /** \brief Get value of key parameter
         * @tparam T type of value
         * @param key string of parameter
         * @return value of key parameter
         * */
        template <typename T>
        T get(const std::string &key) {
            auto result = this->node_[key];
            if (result.IsDefined()) {
                if (result.IsSequence()) {
                    return result.as<T>();
                }
                std::string val = result.as<std::string>();
                std::regex re(("^\\$\\((.*)\\)$"));
                std::cmatch cm;
                if (regex_match(val.c_str(), cm, re)) {
                    return get<T>(cm[1].str());
                } else {
                    return result.as<T>();
                }
            } else {
                return T();
            }
        }

        /** \brief Get value of key parameter
         * if key parameter is null, then set to defaultVal
         * @tparam T type of value
         * @param key string of parameter
         * @param defaultVal defalut value
         * @return value of key parameter
         * */
        template<typename T>
        T get(const std::string &key, const T defaultVal) {
            auto result = this->node_[key];
            if (result.IsDefined()) {
                if (result.IsSequence()) {
                    return result.as<T>();
                }
                std::string val = result.as<std::string>();
                std::regex re(("^\\$\\((.*)\\)$"));
                std::cmatch cm;
                if (regex_match(val.c_str(), cm, re)) {
                    return get<T>(cm[1].str());
                } else {
                    return result.as<T>();
                }
            } else {
                return defaultVal;
            }
        }

        /** \brief Get yaml filepath
         *
         * @return yaml filepath
         */
        std::string get_config_filename();

        /** \brief Make shared of Config
         *
         * @return shard of config
         */
        inline Ptr MakeShared () const { return Ptr (new Config (*this)); }

    public:

        std::string laser_topic_;
        std::string imu_topic_;
        bool is_imu_refine_;

        ///< Translation from laser frame to body frame
        Eigen::Matrix3d RBL;
        Eigen::Vector3d TBL;
        ///< Translation from imu frame to laser frame, laser^T_imu
        Eigen::Matrix3d RLI;
        Eigen::Vector3d TLI;

        double occupied_space_weight_;
        double translation_weight_;
        double rotation_weight_;

        bool use_nonmonotonic_steps_;
        int max_num_iterations_;
        int num_threads_;

        bool insert_free_space_; //probability_grid_range_data_inserter_2d.h
        float hit_probability_; //probability_grid_range_data_inserter_2d.h
        float miss_probability_; //probability_grid_range_data_inserter_2d.h

        size_t num_normal_samples_; //normal_estimation_2d.h
        float sample_radius_; //normal_estimation_2d.h

        double truncation_distance_; //tsdf_range_data_inserter_2d.h
        double maximum_weight_; //tsdf_range_data_inserter_2d.h
        bool update_free_space_; //tsdf_range_data_inserter_2d.h
        bool project_sdf_distance_to_scan_normal_;  //tsdf_range_data_inserter_2d.h
        int update_weight_range_exponent_; //tsdf_range_data_inserter_2d.h
        double update_weight_angle_scan_normal_to_ray_kernel_bandwidth_; //tsdf_range_data_inserter_2d.h
        double update_weight_distance_cell_to_hit_kernel_bandwidth_; //tsdf_range_data_inserter_2d.h

        int num_range_data_;//submap_2d.h
        float grid_options_2d_resolution_; //submap_2d.h
        std::string grid_options_2d_grid_type_; //submap_2d.h  PROBABILITY_GRID / TSDF
        std::string grid_options_2d_range_data_inserter_type_; // submap_2d.h  PROBABILITY_GRID_INSERTER_2D /  TSDF_INSERTER_2D
        float range_data_inserter_truncation_distance_;
        float range_data_inserter_maximum_weight_;

        double max_time_seconds_;  //motion_filter.h
        double max_distance_meters_; // motion_filter.h
        double max_angle_radians_; // motion_filter.h

        int adaptive_voxel_filter_min_num_points_; //voxel_filter.h
        float adaptive_voxel_filter_max_length_; //voxel_filter.h
        float adaptive_voxel_filter_max_range_; //voxel_filter.h

        float min_z_; //local_trajectory_builder_2d.h
        float max_z_;
        float voxel_filter_size_;
        bool use_online_correlative_scan_matching_;
        bool use_imu_data_;
        float min_range_;
        float max_range_;
        float missing_data_ray_length_;
        int num_accumulated_range_data_;

        float imu_gravity_time_constant_;
        double pose_queue_duration_;

    private:
        YAML::Node node_;
        std::string config_filepath_;    
   };
}