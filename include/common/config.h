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
    private:
        YAML::Node node_;
        std::string config_filepath_;    
   };
}