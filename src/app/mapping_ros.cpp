#include "app/mapping_ros.h"

MappingNodeRos::MappingNodeRos(ros::NodeHandle nh, std::string config_file) :
    nh_(nh),
    config_file_(config_file) {
    config_.set_config_filename(config_file_);
    laser_sub_ = nh_.subscribe(config_.laser_topic_, 10, &MappingNodeRos::LaserCallback, this);
    imu_sub_ = nh_.subscribe(config_.imu_topic_, 200, &MappingNodeRos::ImuCallback, this);
}

MappingNodeRos::~MappingNodeRos() {

}

void MappingNodeRos::LaserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    typedef pcl::PointXYZI PointT;
    typedef typename pcl::PointCloud<PointT> PointCloud;

    PointCloud raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
}

void MappingNodeRos::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    double time = msg->header.stamp.toSec();
    double dx = msg->linear_acceleration.x;
    double dy = msg->linear_acceleration.y;
    double dz = msg->linear_acceleration.z;
    double rx = msg->angular_velocity.x;
    double ry = msg->angular_velocity.y;
    double rz = msg->angular_velocity.z;
    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(rx, ry, rz);

    
}