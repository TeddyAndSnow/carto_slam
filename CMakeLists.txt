cmake_minimum_required(VERSION 3.2)
project(carto_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

#if ( CMAKE_SYSTEM_NAME MATCHES "Windows" )
#    set( my_std_pre "/std:" )
#else()
#    set( my_std_pre "-std=" )
#endif()
#
#set( basic_cxx17 "c++17" )
#set( str_cxx17 "${my_std_pre}${basic_cxx17}" )
#
#include( CheckCXXCompilerFlag )
#check_cxx_compiler_flag( "${str_cxx17}" _cpp_17_flag_supported )
#if ( _cpp_17_flag_supported )
#    set( CMAKE_CXX_STANDARD 17 )
# endif()
#
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

set(3rdParty_DIR "/home/zhihui/MyFiles/Projects/LIBS")
set(CMAKE_PREFIX_PATH "${3rdParty_DIR}/3rdparty;${3rdParty_DIR}/3rdparty/share;${3rdParty_DIR}/3rdparty/lib/cmake;${CMAKE_PREFIX_PATH}")

# find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(absl REQUIRED)
# find_package(PCL REQUIRED)
# find_package(cairo REQUIRED)
include(FindPkgConfig)
if (NOT WIN32)
  PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)
else()
  find_library(CAIRO_LIBRARIES cairo)
endif()

include_directories(
    ${CERES_INCLUDE_DIRS}
    ${sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

include_directories(include)

add_compile_options(-fPIC)
add_library(carto_slam STATIC
    # src/common/compressed_point_cloud.cpp
    src/common/config.cpp
    src/common/fixed_ratio_sampler.cpp
    src/common/histogram.cpp
    src/common/pointcloud_transform.cpp
    src/common/rigid_transform.cpp
    src/common/task.cpp
    src/common/thread_pool.cpp
    src/common/time.cpp
    src/common/voxel_filter.cpp

    src/io/color.cpp
    src/io/file_writer.cpp
    src/io/image.cpp
    src/io/submap_painter.cpp

    src/estimator/2d/grid_2d.cpp
    src/estimator/2d/normal_estimation_2d.cpp
    src/estimator/2d/probability_grid_range_data_inserter_2d.cpp
    src/estimator/2d/probability_grid.cpp
    src/estimator/2d/ray_to_pixel_mask.cpp
    src/estimator/2d/submap_2d.cpp
    src/estimator/2d/tsd_value_converter.cpp
    src/estimator/2d/tsdf_2d.cpp
    src/estimator/2d/tsdf_range_data_inserter_2d.cpp

    src/estimator/2d/scan_matching/correlative_scan_matcher_2d.cpp
    src/estimator/2d/scan_matching/fast_correlative_scan_matcher_2d.cpp
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

    src/loop/ceres_pose.cpp
    src/loop/connected_components.cpp
    src/loop/constraint_builder_2d.cpp
    src/loop/constraint_builder.cpp
    src/loop/optimization_problem_2d.cpp
    src/loop/optimization_problem_options.cpp
    src/loop/overlapping_submaps_trimmer_2d.cpp
    src/loop/pose_graph_2d.cpp
    src/loop/pose_graph_trimmer.cpp
    src/loop/pose_graph.cpp
    src/loop/spa_cost_function_2d.cpp
    src/loop/trajectory_connectivity_state.cpp
)

target_link_libraries(carto_slam
    # ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${CAIRO_LIBRARIES}
    absl::algorithm
    absl::base
    absl::debugging
    absl::flat_hash_map
    absl::memory
    absl::meta
    absl::numeric
    absl::str_format
    absl::strings
    absl::synchronization
    absl::time
    absl::utility
    absl::strings_internal
    absl::str_format_internal
)

message(STATUS "absl: " absl::strings)

set(CMAKE_PREFIX_PATH "/opt/ros/kinetic;${CMAKE_PREFIX_PATH}")
find_package(catkin COMPONENTS rosbag roscpp std_msgs nav_msgs sensor_msgs cv_bridge visualization_msgs)

if (catkin_FOUND)
    find_package(PCL REQUIRED)
    include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

    add_executable(carto_slam_node
        src/app/mapping_ros.cpp
        src/app/mapping_main.cpp
        )
    
    target_link_libraries(carto_slam_node
        carto_slam
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
    )
endif (catkin_FOUND)