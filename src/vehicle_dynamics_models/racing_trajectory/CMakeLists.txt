cmake_minimum_required(VERSION 3.8)
project(racing_trajectory)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set ROS_DISTRO macros
if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "Environment variable ROS_DISTRO is not defined. Have you sourced your ROS workspace?")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
  add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "galactic")
  add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()

# Require that dependencies from package.xml be available.
find_package(casadi REQUIRED)
find_package(TBB REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/racing_trajectory.cpp
  src/racing_trajectory_map.cpp
  src/trajectory_kd_tree.cpp
  src/safe_set.cpp
  src/ros_trajectory_visualizer.cpp
)

set(${PROJECT_NAME}_HEADER
  include/racing_trajectory/racing_trajectory.hpp
  include/racing_trajectory/racing_trajectory_map.hpp
  include/racing_trajectory/trajectory_kd_tree.hpp
  include/racing_trajectory/safe_set.hpp
  include/racing_trajectory/ros_trajectory_visualizer.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

target_link_libraries(${PROJECT_NAME} casadi Eigen3::Eigen TBB::tbb)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_racing_trajectory.cpp)
  set(TEST_TRAJECTORY_EXE test_trajectory)
  ament_add_gtest(${TEST_TRAJECTORY_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_TRAJECTORY_EXE} ${PROJECT_NAME})
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  test_data
)
