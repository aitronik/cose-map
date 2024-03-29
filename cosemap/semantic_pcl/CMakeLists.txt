cmake_minimum_required(VERSION 3.8)
project(semantic_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O0)
endif()

set (CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(ros_utils REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost REQUIRED)
find_package(cosemap_tools REQUIRED)
find_package(cosemap_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


include_directories(
  include
)

add_executable(semantic_pcl_node      

                src/run_semantic_pcl_node.cpp
                src/semantic_pcl_node/semantic_pcl.cpp
                src/semantic_pcl_node/semantic_pcl_node.cpp
               )


ament_target_dependencies(semantic_pcl_node
    rclcpp
    # ros_utils
    std_msgs
    sensor_msgs
    geometry_msgs
    OpenCV
    cv_bridge
    message_filters
    Boost
    cosemap_tools
    cosemap_msgs
  )

install(TARGETS      semantic_pcl_node 
        DESTINATION  lib/${PROJECT_NAME})

ament_export_dependencies(PCL)

ament_package()
