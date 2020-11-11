#include "app/mapping_ros.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_ros_node");
    ros::NodeHandle nh;

    MappingNodeRos mapping_ros(nh, "/home/zhihui/MyFiles/Projects/SLAM/carto_slam/config/config.yaml");
    mapping_ros.run();
    std::cout << "carto_slam" << std::endl;
    return 1;
}