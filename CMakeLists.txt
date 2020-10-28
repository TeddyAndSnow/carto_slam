cmake_minimum_required(VERSION 3.2)
project(carto_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

set(3rdParty_DIR "/home/zhihui/MyFiles/Projects/LIBS")
set(CMAKE_PREFIX_PATH "${3rdParty_DIR}/3rdparty;${3rdParty_DIR}/3rdparty/share;${3rdParty_DIR}/3rdparty/lib/cmake;${CMAKE_PREFIX_PATH}")

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

include_directories(include)

set(CMAKE_PREFIX_PATH "/opt/ros/kinetic;${CMAKE_PREFIX_PATH}")
find_package(catkin COMPONENTS rosbag roscpp std_msgs nav_msgs sensor_msgs cv_bridge)

if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})

    add_executable(carto_slam_node
        src/app/mapping_ros.cpp
        src/app/mapping_main.cpp

        src/common/config.cpp
        src/common/pointcloud_transform.cpp
        src/common/rigid_transform.cpp
        src/common/time.cpp
        src/common/voxel_filter.cpp

        src/estimator/2d/grid_2d.cpp
        src/estimator/2d/normal_estimation_2d.cpp
        src/estimator/2d/probability_grid_range_data_inserter_2d.cpp
        src/estimator/2d/probability_grid.cpp
        src/estimator/2d/ray_to_pixel_mask.cpp
        src/estimator/2d/submap_2d.cpp
        src/estimator/2d/tsd_value_converter.cpp
        src/estimator/2d/tsdf_2d.cpp
        src/estimator/2d/tsdf_range_data_inserter_2d.cpp

        src/estimator/2d/scan_matching/occupied_space_cost_function_2d.cpp
        src/estimator/2d/scan_matching/tsdf_match_cost_function_2d.cpp

        src/estimator/ceres_scan_matcher_2d.cpp
        src/estimator/imu_tracker.cpp
        src/estimator/local_trajectory_builder_2d.cpp
        src/estimator/motion_filter.cpp
        src/estimator/pose_extrapolator.cpp
        src/estimator/probability_values.cpp
        src/estimator/range_data_collator.cpp
        src/estimator/value_conversion_tables.cpp
        )
    
    target_link_libraries(carto_slam_node
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
    )
endif (catkin_FOUND)