cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  traj_utils
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

find_library(OSQP_LIBRARY NAMES osqp)
find_path(OSQP_INCLUDE_DIR NAMES osqp/osqp.h)
if(NOT OSQP_LIBRARY OR NOT OSQP_INCLUDE_DIR)
  message(FATAL_ERROR "OSQP 0.6.x was not found (expected osqp/osqp.h and libosqp).")
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OSQP_INCLUDE_DIR}
  include/${PROJECT_NAME}
  include
  src
)

add_library(px4ctrl_ommpc
  src/ommpc_controller.cpp
)
add_dependencies(px4ctrl_ommpc ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4ctrl_ommpc
  ${catkin_LIBRARIES}
  ${OSQP_LIBRARY}
)

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(px4ctrl_node
  px4ctrl_ommpc
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(ommpc_controller_test
    test/ommpc_controller_test.cpp
    src/PX4CtrlParam.cpp
    src/controller.cpp
    src/input.cpp
  )
  if(TARGET ommpc_controller_test)
    target_link_libraries(ommpc_controller_test
      px4ctrl_ommpc
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_nosetests(test/test_polytraj_publisher.py)
endif()

catkin_install_python(PROGRAMS
  thrust_calibrate_scrips/thrust_calibrate.py
  scripts/publish_polytraj.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
