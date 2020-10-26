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


}