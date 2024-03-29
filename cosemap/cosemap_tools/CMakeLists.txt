cmake_minimum_required(VERSION 3.8)
project(cosemap_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O0 -fPIC)
endif()

set (CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
  include
)


# to add mariadbcpp library to the project
add_library(mariadbcpp STATIC IMPORTED)
set_property(TARGET mariadbcpp PROPERTY IMPORTED_LOCATION /usr/lib/libmariadbcpp.so) #path to libmariadbcpp.so

add_library(semantic_utils  src/semantic_utils/semantic_utils.cpp)
add_library(inference_utils src/inference_utils/inference_utils.cpp)
add_library(map_loader src/map_loader/map_loader.cpp)
add_library(visualizer  src/visualizer/database_visualizer.cpp)
add_library(sqlHelper src/sql_helper/sql_helper.cpp)
add_library(base_node src/base_node/base_node.cpp)

                    
target_link_libraries(sqlHelper mariadbcpp)
target_link_libraries(semantic_utils ${PCL_LIBRARIES})
target_link_libraries(inference_utils ${OpenCV_LIBS})
target_link_libraries(map_loader octomap)
# ros libs have to be linked with ament_target_dependencies
ament_target_dependencies(base_node rclcpp)
ament_target_dependencies(visualizer visualization_msgs)
target_link_libraries(visualizer semantic_utils)

ament_export_targets(semantic_utilsTargets HAS_LIBRARY_TARGET)
ament_export_targets(inference_utilsTargets HAS_LIBRARY_TARGET)
ament_export_targets(map_loaderTargets HAS_LIBRARY_TARGET)
ament_export_targets(visualizerTargets HAS_LIBRARY_TARGET)
ament_export_targets(sqlHelperTargets HAS_LIBRARY_TARGET)
ament_export_targets(base_nodeTargets HAS_LIBRARY_TARGET)

ament_export_dependencies(PCL)
ament_export_dependencies(OpenCV)
ament_export_dependencies(octomap)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(rclcpp)

install(
    DIRECTORY include/
    DESTINATION include/
)

install(
    TARGETS semantic_utils
    EXPORT semantic_utilsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS inference_utils
    EXPORT inference_utilsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS map_loader
    EXPORT map_loaderTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS visualizer
    EXPORT visualizerTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS sqlHelper
    EXPORT sqlHelperTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS base_node
    EXPORT base_nodeTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

set(cosemap_tools_INCLUDE_DIRS 
        ${CMAKE_INSTALL_PREFIX}/include/)

ament_package()