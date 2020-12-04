#include "app/mapping_ros.h"
#include "common/time.h"
#include "common/pointcloud_transform.h"
#include "loop/pose_graph_2d.h"

#include <vector>
#include <map>

using namespace carto_slam;

common::Time FromRos(const ::ros::Time &time)
{
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return common::FromUniversal(
        (time.sec + common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
        (time.nsec + 50) / 100); // + 50 to get the rounding correct.
}

bool PointCloud2HasField(const sensor_msgs::PointCloud2 &pc2,
                         const std::string &field_name)
{
    for (const auto &field : pc2.fields)
    {
        if (field.name == field_name)
        {
            return true;
        }
    }
    return false;
}

std::tuple<::common::PointCloudWithIntensities, ::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2 &msg)
{
    common::PointCloudWithIntensities point_cloud;
    // We check for intensity field here to avoid run-time warnings if we pass in
    // a PointCloud2 without intensity.
    if (PointCloud2HasField(msg, "intensity1"))
    {
        if (PointCloud2HasField(msg, "time"))
        {
            pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto &point : pcl_point_cloud)
            {
                point_cloud.points.push_back({Eigen::Vector3f{point.x, point.y, point.z}, 0.0}); //point.time
                point_cloud.intensities.push_back(point.intensity);
            }
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto &point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
                point_cloud.intensities.push_back(point.intensity);
            }
        }
    }
    else
    {
        // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
        if (PointCloud2HasField(msg, "time"))
        {
            pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto &point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f}); //point.time
                point_cloud.intensities.push_back(1.0f);
            }
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto &point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
                point_cloud.intensities.push_back(1.0f);
            }
        }
    }
    common::Time timestamp = FromRos(msg.header.stamp);
    if (!point_cloud.points.empty())
    {
        const double duration = point_cloud.points.back().time;
        timestamp += common::FromSeconds(duration);
        for (auto &point : point_cloud.points)
        {
            point.time -= duration;
            CHECK_LE(point.time, 0.f)
                << "Encountered a point with a larger stamp than "
                   "the last point in the cloud.";
        }
    }
    return std::make_tuple(point_cloud, timestamp);
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3 &vector3)
{
    return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

std::unique_ptr<common::ImuData> ToImuData(const sensor_msgs::Imu::ConstPtr &msg)
{
    const common::Time time = FromRos(msg->header.stamp);

    Eigen::Vector3d t_b_i = Eigen::Vector3d(-0.016, 0.0, 0.075);
    Eigen::Matrix3d r_b_i;
    r_b_i << 0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0,
        0.0, 0.0, 1.0;
    Eigen::Quaterniond q_b_i(r_b_i);

    common::Rigid3d sensor_to_tracking(t_b_i, q_b_i);

    return std::make_unique<common::ImuData>(common::ImuData{
        time, sensor_to_tracking.rotation() * ToEigen(msg->linear_acceleration) * 9.8,
        sensor_to_tracking.rotation() * ToEigen(msg->angular_velocity)});
}

ros::Time ToRos(common::Time time)
{
    int64_t uts_timestamp = common::ToUniversal(time);
    int64_t ns_since_unix_epoch =
        (uts_timestamp -
         ::common::kUtsEpochOffsetFromUnixEpochInSeconds *
             10000000ll) *
        100ll;
    ::ros::Time ros_time;
    ros_time.fromNSec(ns_since_unix_epoch);
    return ros_time;
}

sensor_msgs::PointCloud2 PreparePointCloud2Message(const uint64_t timestamp, const std::string &frame_id, const int num_points)
{
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ToRos(common::FromUniversal(timestamp));
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = num_points;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = 16 * msg.width;
    msg.is_dense = true;
    msg.data.resize(16 * num_points);
    return msg;
}
constexpr float kPointCloudComponentFourMagic = 1.;
sensor_msgs::PointCloud2 ToPointCloud2Message(const uint64_t timestamp, const std::string &frame_id, const common::TimedPointCloud &point_cloud)
{
    auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
    for (const common::TimedRangefinderPoint &point : point_cloud)
    {
        stream.next(point.position.x());
        stream.next(point.position.y());
        stream.next(point.position.z());
        stream.next(kPointCloudComponentFourMagic);
    }
    return msg;
}

std::unique_ptr<io::SubmapTextureData> ToSubmapTexture(std::shared_ptr<const estimator::Submap2D> submap)
{
    std::unique_ptr<io::SubmapTextureData> texture = std::make_unique<io::SubmapTextureData>();
    submap->grid()->DrawToSubmapTexture(texture.get(), submap->local_pose());
    return std::move(texture);
}

std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const io::PaintSubmapSlicesResult &painted_slices,
    const double resolution, const std::string &frame_id,
    const ros::Time &time)
{
    auto occupancy_grid = std::make_unique<nav_msgs::OccupancyGrid>();

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height =
        cairo_image_surface_get_height(painted_slices.surface.get());

    occupancy_grid->header.stamp = time;
    occupancy_grid->header.frame_id = frame_id;
    occupancy_grid->info.map_load_time = time;
    occupancy_grid->info.resolution = resolution;
    occupancy_grid->info.width = width;
    occupancy_grid->info.height = height;
    occupancy_grid->info.origin.position.x =
        -painted_slices.origin.x() * resolution;
    occupancy_grid->info.origin.position.y =
        (-height + painted_slices.origin.y()) * resolution;
    occupancy_grid->info.origin.position.z = 0.;
    occupancy_grid->info.origin.orientation.w = 1.;
    occupancy_grid->info.origin.orientation.x = 0.;
    occupancy_grid->info.origin.orientation.y = 0.;
    occupancy_grid->info.origin.orientation.z = 0.;

    const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
        cairo_image_surface_get_data(painted_slices.surface.get()));
    occupancy_grid->data.reserve(width * height);
    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
                observed == 0
                    ? -1
                    : ::common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            occupancy_grid->data.push_back(value);
        }
    }

    return occupancy_grid;
}

MappingNodeRos::MappingNodeRos(ros::NodeHandle nh, std::string config_file) : nh_(nh),
                                                                              config_file_(config_file),
                                                                              thread_pool_(10)
{
    config_.set_config_filename(config_file_);
    laser_sub_ = nh_.subscribe(config_.laser_topic_, 10, &MappingNodeRos::LaserCallback, this);
    imu_sub_ = nh_.subscribe(config_.imu_topic_, 200, &MappingNodeRos::ImuCallback, this);

    trajectory_node_list_publisher_ = nh_.advertise<::visualization_msgs::MarkerArray>("/trajectory_node_list", 10);
    scan_matched_point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_matched_points2", 10);
    occupancy_grid_publisher_ = nh_.advertise<::nav_msgs::OccupancyGrid>("/map", 1, true /* latched */);

    trajectory_id_ = 1;
    std::vector<std::string> expected_range_sensor_ids;
    expected_range_sensor_ids.push_back(std::to_string(trajectory_id_));
    local_traj_builder_ptr_ = std::make_shared<carto_slam::estimator::LocalTrajectoryBuilder2D>(config_, expected_range_sensor_ids);
    pose_graph_ptr = std::make_shared<carto_slam::loop::PoseGraph2D>(config_, std::make_unique<carto_slam::loop::OptimizationProblem2D>(config_), &thread_pool_);
}

MappingNodeRos::~MappingNodeRos()
{
}

void MappingNodeRos::run()
{
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        //TODO:

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MappingNodeRos::LaserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

    std::cout << "================================================================" << std::endl;
    common::PointCloudWithIntensities point_cloud;
    common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);

    const float min_height = 0.0;
    const float max_height = 0.1;
    const float min_range = 0.8;
    const float max_range = 100.0;

    common::PointCloudWithIntensities filter_point_cloud;
    for (int i = 0; i < point_cloud.points.size(); i++)
    {
        if (point_cloud.points[i].position.z() >= min_height && point_cloud.points[i].position.z() <= max_height)
        {

            float plane_distance = sqrt(point_cloud.points[i].position.x() * point_cloud.points[i].position.x() +
                                        point_cloud.points[i].position.y() * point_cloud.points[i].position.y());
            if (plane_distance < min_range || plane_distance > max_range)
            {
                continue;
            }
            filter_point_cloud.points.push_back(point_cloud.points[i]);
            filter_point_cloud.intensities.push_back(point_cloud.intensities[i]);
        }
    }

    Eigen::Vector3d t_b_l = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Quaterniond q_b_l = Eigen::Quaterniond(Eigen::AngleAxisd(0.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                                                  Eigen::AngleAxisd(0.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(0.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));

    common::Rigid3d sensor_to_tracking(t_b_l, q_b_l);

    auto transform_point_cloud = common::TimedPointCloudData{time, sensor_to_tracking.translation().cast<float>(),
                                                             common::TransformTimedPointCloud(filter_point_cloud.points, sensor_to_tracking.cast<float>())};

    // std::cout << "Add Range Data" << std::endl;
    /// local trajectory
    auto matching_result = local_traj_builder_ptr_->AddRangeData(config_.laser_topic_, transform_point_cloud);
    if (matching_result == nullptr)
    {
        std::cout << "matching_result is null" << std::endl;
        return;
    }
    // std::cout << matching_result->local_pose.translation().x() << " " << matching_result->local_pose.translation().y() << " " << matching_result->local_pose.translation().z() << std::endl;

    using namespace carto_slam::estimator;
    // std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> insertion_result;
    // if (matching_result->insertion_result != nullptr)
    // {
    //     // std::cout << "insertion submaps size: " << matching_result->insertion_result->insertion_submaps.size() << std::endl;
    //     auto node_id = pose_graph_ptr->AddNode(matching_result->insertion_result->constant_data,
    //                                            trajectory_id_,
    //                                            matching_result->insertion_result->insertion_submaps);
    //     // insertion_result = std::make_unique<LocalTrajectoryBuilder2D::InsertionResult>(LocalTrajectoryBuilder2D::InsertionResult{
    //     //     matching_result->insertion_result->constant_data,
    //     //     std::vector<std::shared_ptr<const Submap2D>>(
    //     //         matching_result->insertion_result->insertion_submaps.begin(),
    //     //         matching_result->insertion_result->insertion_submaps.end())});
    // }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /// publish trajectory
    static std::vector<Eigen::Vector3d> trajectory_vector;
    trajectory_vector.push_back(matching_result->local_pose.translation().cast<double>());
    if (trajectory_vector.size() > 1)
    {
        visualization_msgs::MarkerArray trajctory_marker;
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path";
        path_marker.id = 1;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;

        path_marker.color = color;
        path_marker.scale.x = 0.01;
        path_marker.scale.y = 0.01;

        for (auto tp : trajectory_vector)
        {
            geometry_msgs::Point point;
            // point.x = ptr->at(i).ptr->at(j)[0];
            // point.y = ptr->at(i).ptr->at(j)[1];
            point.x = tp.x();
            point.y = tp.y();
            point.z = 0.0;
            path_marker.points.push_back(point);
            // LOG(INFO) << "vis x: " << point.x << " y: " << point.y << std::endl;
        }
        trajctory_marker.markers.push_back(path_marker);
        trajectory_node_list_publisher_.publish(trajctory_marker);
    }

    /// publish scan
    // std::cout << "size: " << transform_point_cloud.ranges.size() << std::endl;
    scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(common::ToUniversal(matching_result->time), "map",
                                                                     common::TransformTimedPointCloud(transform_point_cloud.ranges, matching_result->local_pose.cast<float>())));

    /// publish submap
    if (matching_result->insertion_result == nullptr)
    {
        // std::cout << "matching_result's insertion_result is null" << std::endl;
        return;
    }

    std::map<estimator::SubmapId, io::SubmapSlice> submap_slices_;
    estimator::SubmapId id(1, 1);
    io::SubmapSlice &submap_slice = submap_slices_[id];
    std::unique_ptr<io::SubmapTextures> fetched_textures = std::make_unique<::io::SubmapTextures>();

    std::cout << "submap size: " << matching_result->insertion_result->insertion_submaps.size() << std::endl;

    // std::cout << "local pose: (xyz) " << 
    // matching_result->local_pose.translation().x() << " " << 
    // matching_result->local_pose.translation().y() << " " << 
    // matching_result->local_pose.translation().z() << " (xyzw)" << 
    // matching_result->local_pose.rotation().x() << " " <<
    // matching_result->local_pose.rotation().y() << " " <<
    // matching_result->local_pose.rotation().z() << " " << 
    // matching_result->local_pose.rotation().w() << std::endl;

    // std::cout << "constant_data: (xyz) " << 
    // matching_result->insertion_result->constant_data->local_pose.translation().x() << " " << 
    // matching_result->insertion_result->constant_data->local_pose.translation().y() << " " << 
    // matching_result->insertion_result->constant_data->local_pose.translation().z() << " (xyzw)" << 
    // matching_result->insertion_result->constant_data->local_pose.rotation().x() << " " <<
    // matching_result->insertion_result->constant_data->local_pose.rotation().y() << " " <<
    // matching_result->insertion_result->constant_data->local_pose.rotation().z() << " " << 
    // matching_result->insertion_result->constant_data->local_pose.rotation().w() << std::endl;

    std::vector<common::Rigid3d> submap_local_pose;
    for (auto submap : matching_result->insertion_result->insertion_submaps)
    {
        // if (submap->insertion_finished()) {
        //     std::cout << "insertion finished" << std::endl;
        // } else {
        //     std::cout << "insertion not finished" << std::endl;
        // }

        // std::cout << "xyz: " << submap->local_pose().translation().x() << " " << 
        // submap->local_pose().translation().y() << " " << submap->local_pose().translation().z() << std::endl;

        submap_local_pose.emplace_back(submap->local_pose());
        auto texture = ToSubmapTexture(submap);
        const std::string compressed_cells(texture->cells.begin(), texture->cells.end());
        fetched_textures->textures.emplace_back(io::SubmapTexture{
            io::UnpackTextureData(compressed_cells, texture->width, texture->height),
            texture->width, texture->height, texture->resolution,
            texture->slice_pose});
    }

    // std::cout << "fetched_texture_size: " << fetched_textures->textures.size() << std::endl;
    submap_slice.version = fetched_textures->version;
    submap_slice.pose = submap_local_pose.back();  // n-1

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    // const auto fetched_texture = fetched_textures->textures.begin();
    const auto fetched_texture = fetched_textures->textures.end() - 1; // n-1
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);

    std::cout << "slice_pose: " << submap_slices_[id].slice_pose.translation().x() << " " << submap_slices_[id].slice_pose.translation().y() << " " << submap_slices_[id].slice_pose.translation().z() << std::endl;
    std::cout << "pose: " << submap_slices_[id].pose.translation().x() << " " << submap_slices_[id].pose.translation().y() << " " << submap_slices_[id].pose.translation().z() << std::endl;

    auto painted_slices = io::PaintSubmapSlices(submap_slices_, 0.05);
    std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(painted_slices, 0.05, "map", ros::Time::now());
    occupancy_grid_publisher_.publish(*msg_ptr);
}

void MappingNodeRos::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    auto imu_data_ptr = ToImuData(msg);
    if (imu_data_ptr != nullptr)
    {
        auto tranform_imu = common::ImuData{imu_data_ptr->time, imu_data_ptr->linear_acceleration,
                                            imu_data_ptr->angular_velocity};
        // std::cout << "Add Imu Data" << std::endl;
        /// local trajectory
        local_traj_builder_ptr_->AddImuData(tranform_imu);

        // pose_graph_ptr->AddImuData(trajectory_id_, *imu_data_ptr);
    }
}