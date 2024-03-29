cmake_minimum_required(VERSION 3.8)
project(semantic_database)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(ros_utils REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(cosemap_msgs)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(semantic_pcl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(yaml-cpp REQUIRED)
find_package(cosemap_tools REQUIRED)

include_directories(
    include
    "/usr/include/mariadb"
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIR}
    
)
message(${PCL_INCLUDE_DIRS})

# to add mariadbcpp library to the project
add_library(mariadbcpp STATIC IMPORTED)
set_property(TARGET mariadbcpp PROPERTY IMPORTED_LOCATION /usr/lib/libmariadbcpp.so) #path to libmariadbcpp.so

add_executable(semantic_database_node      
        src/semantic_database_node/run_semantic_database_node.cpp
        src/semantic_database_node/semantic_database_node.cpp
        )

add_executable(map_visualizer_node      
               src/map_visualizer_node/run_map_visualizer_node.cpp
               src/map_visualizer_node/map_visualizer_node.cpp
)

ament_target_dependencies(semantic_database_node
  rclcpp
  # ros_utils
  OCTOMAP
  cosemap_msgs
  std_msgs
  octomap_msgs
  octomap_ros
  sensor_msgs
  visualization_msgs
  cosemap_tools
  PCL
)

ament_target_dependencies(map_visualizer_node
  rclcpp
  # ros_utils
  cosemap_msgs
  std_msgs
  octomap_msgs
  visualization_msgs
  cosemap_tools
)

#to link libraries
target_link_libraries(semantic_database_node mariadbcpp ${PLC_LIBRARIES})


ament_export_dependencies(PCL)

install(TARGETS      semantic_database_node 
        DESTINATION  lib/${PROJECT_NAME})

install(TARGETS      map_visualizer_node 
        DESTINATION  lib/${PROJECT_NAME})


        
ament_package()
