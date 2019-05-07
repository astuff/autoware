cmake_minimum_required(VERSION 2.8.3)
project(gpsins_localizer)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps roscpp roslib nodelet tf2 tf2_ros novatel_gps_msgs sensor_msgs geometry_msgs roslint)
find_package(catkin REQUIRED ${catkin_deps})

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
)

roslint_cpp()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# nodelet
add_library(${PROJECT_NAME}_nodelet
    src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    libGeographic.so
)

# node
add_executable(${PROJECT_NAME}_node
    src/gpsins_localizer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

##########
## TEST ##
##########

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    # ROS tests
    add_rostest_gtest(test_${PROJECT_NAME}_node
        test/rostest_launch.test
        test/rostest_integration_test.cpp
    )
    target_link_libraries(test_${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
    )
endif(CATKIN_ENABLE_TESTING)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
