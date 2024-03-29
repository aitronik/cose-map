cmake_minimum_required(VERSION 3.8)
project(cosemap_launch)

find_package(ament_cmake REQUIRED)

        
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY labels
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY utils
        DESTINATION share/${PROJECT_NAME})

ament_package()