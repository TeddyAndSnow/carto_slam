#pragma once

#include "common/config.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

class MappingNodeRos {
public:
    typedef std::shared_ptr<MappingNodeRos> Ptr;
    MappingNodeRos(ros::NodeHandle nh, std::string config_file);
    ~MappingNodeRos();

private:
    void LaserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Subscriber imu_sub_;

    std::string config_file_;
    carto_slam::Config config_;
};