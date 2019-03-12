cmake_minimum_required(VERSION 2.8.3)
project(agile_apps)

add_definitions(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospack

  agile_driver
  agile_foundation
)

# Declare a catkin package
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  DEPENDS
    roscpp
)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_ROOT            src)
# set(SRC_EXAMPLES        examples/src)
set(SRC_APP             ${SRC_ROOT}/apps)

## Declare a C++ executable
add_executable(pd_apps
  ${SRC_APP}/pd_wrapper.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(pd_apps
  ${catkin_LIBRARIES}
  agile_foundation
  agile_driver_sys_robot
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS pd_apps
                # ${PROJECT_NAME}_test_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir examples/launch examples/config launch config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
