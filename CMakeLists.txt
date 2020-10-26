cmake_minimum_required(VERSION 3.2)
project(carto_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

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
        src/common/rigid_transform.cpp
        src/common/time.cpp
        src/estimator/imu_tracker.cpp
        #src/estimator/pose_extrapolator.cpp
        )
    
    target_link_libraries(carto_slam_node
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
    )
endif (catkin_FOUND)