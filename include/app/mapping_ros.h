#pragma once

#include "common/config.h"
#include "common/thread_pool.h"
#include "estimator/local_trajectory_builder_2d.h"
#include "loop/pose_graph_2d.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

class MappingNodeRos
{
public:
    typedef std::shared_ptr<MappingNodeRos> Ptr;
    MappingNodeRos(ros::NodeHandle nh, std::string config_file);
    ~MappingNodeRos();

    void run();

private:
    void LaserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher trajectory_node_list_publisher_;
    ros::Publisher scan_matched_point_cloud_publisher_;
    ros::Publisher occupancy_grid_publisher_;

    std::string config_file_;
    carto_slam::Config config_;

    int trajectory_id_;

    carto_slam::estimator::LocalTrajectoryBuilder2D::Ptr local_traj_builder_ptr_;
    // carto_slam::loop::PoseGraph2D::Ptr pose_graph_ptr;
    carto_slam::common::ThreadPool thread_pool_;
};