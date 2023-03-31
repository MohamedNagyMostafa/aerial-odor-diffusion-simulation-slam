cmake_minimum_required(VERSION 2.8.3)
project(particle_shooter)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++17)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES particle_shooter
        DEPENDS
        roscpp
        gazebo_ros
)



link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

# For particle_shooter_plugin plugin
add_library(particle_shooter_plugin src/particle_shooter_plugin.cpp include/particle_shooter_plugin.h)
target_link_libraries(particle_shooter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



install(TARGETS particle_shooter_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        INCLUDES DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )