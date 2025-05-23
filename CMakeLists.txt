cmake_minimum_required(VERSION 3.0.2)
project(ekalibr)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "RELEASE")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        rosbag
        roscpp
        rospy
        std_msgs
        cv_bridge
        sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        PropheseeEvent.msg
        PropheseeEventArray.msg

        DVSEvent.msg
        DVSEventArray.msg

        SbgImuData.msg
        SbgImuStatus.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ekalibr
        #  CATKIN_DEPENDS geometry_msgs message_generation rosbag roscpp rospy std_msgs
        #  DEPENDS system_lib
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(magic_enum REQUIRED)
# magic_enum works in range of [MAGIC_ENUM_RANGE_MIN, MAGIC_ENUM_RANGE_MAX]
add_definitions(-DMAGIC_ENUM_RANGE_MIN=0)
# 1024 = 2^10
add_definitions(-DMAGIC_ENUM_RANGE_MAX=1023)

find_package(yaml-cpp REQUIRED)

find_package(Ceres REQUIRED)

set(tiny-viewer_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj/thirdparty/tiny-viewer-install/lib/cmake/tiny-viewer)
find_package(tiny-viewer)

set(ctraj_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj-install/lib/cmake/ctraj)
find_package(ctraj)

set(opengv_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opengv-install/lib/cmake/opengv-1.0)
find_package(opengv)

set(veta_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/veta-install/lib/cmake/veta)
find_package(veta)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/config CONFIG_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/util UTIL_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/sensor SENSOR_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/core CORE_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/viewer VIEWER_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/factor FACTOR_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/calib CALIB_SRC_FILES)

###################
# libekalibr_util #
###################
add_library(
        ${PROJECT_NAME}_util SHARED
        # source files
        ${UTIL_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_util PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${PROJECT_NAME}_util PUBLIC
        ${catkin_LIBRARIES}
        tiny-viewer
        veta
)

#####################
# libekalibr_config #
#####################
add_library(
        ${PROJECT_NAME}_config SHARED
        # source files
        ${CONFIG_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_config PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${PROJECT_NAME}_config PUBLIC
        ${catkin_LIBRARIES}
        # thirdparty
        ${YAML_CPP_LIBRARIES}
        veta
)


#####################
# libekalibr_sensor #
#####################
add_library(
        ${PROJECT_NAME}_sensor SHARED
        # source files
        ${SENSOR_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_sensor PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${PROJECT_NAME}_sensor PUBLIC
        ${catkin_LIBRARIES}
        # thirdparty
        tiny-viewer
        veta
        ctraj
)
###################
# libekalibr_core #
###################
add_library(
        ${PROJECT_NAME}_core SHARED
        # source files
        ${CORE_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_core PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # thirdparty
        ${CERES_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME}_core PUBLIC
        ${catkin_LIBRARIES}
        # thirdparty
        tiny-viewer
        opengv
        ${CERES_LIBRARIES}
        veta
        ctraj
)
#####################
# libikalibr_viewer #
#####################
add_library(
        ${PROJECT_NAME}_viewer SHARED
        # source files
        ${VIEWER_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_viewer PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${PROJECT_NAME}_viewer PUBLIC
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_sensor
        # thirdparty
        tiny-viewer
        veta
)
#####################
# libekalibr_factor #
#####################
add_library(
        ${PROJECT_NAME}_factor SHARED
        # source files
        ${FACTOR_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_factor PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${PROJECT_NAME}_factor PUBLIC
        ${catkin_LIBRARIES}
        # thirdparty
        ctraj
        veta
        opengv
)
###################
# libekalibr_calib #
###################
add_library(
        ${PROJECT_NAME}_calib SHARED
        # source files
        ${CALIB_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_calib PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # thirdparty
        ${CERES_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME}_calib PUBLIC
        ${catkin_LIBRARIES}

        ${PROJECT_NAME}_config
        ${PROJECT_NAME}_util
        ${PROJECT_NAME}_sensor
        ${PROJECT_NAME}_viewer
        ${PROJECT_NAME}_factor
        ${PROJECT_NAME}_core
        # thirdparty
        tiny-viewer
        opengv
        ${CERES_LIBRARIES}
        veta
        ctraj
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#        # include
#        ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ekalibr.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
        ${PROJECT_NAME}_prog
        exe/main.cpp
)
target_include_directories(
        ${PROJECT_NAME}_prog PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Specify libraries to link a library or executable target against
target_link_libraries(
        ${PROJECT_NAME}_prog
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_calib
)

add_executable(
        ${PROJECT_NAME}_undistortion
        exe/undistortion.cpp
)
target_include_directories(
        ${PROJECT_NAME}_undistortion PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
## Specify libraries to link a library or executable target against
target_link_libraries(
        ${PROJECT_NAME}_undistortion
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_calib
)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ekalibr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
