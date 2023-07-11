cmake_minimum_required(VERSION 3.8)
project(ekf_state_estimator)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set ROS_DISTRO macros
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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/ekf_state_estimator.cpp
  src/ros_param_loader.cpp
)

set(${PROJECT_NAME}_HEADER
  include/ekf_state_estimator/ekf_state_estimator.hpp
  include/ekf_state_estimator/ekf_state_estimator_config.hpp
  include/ekf_state_estimator/ros_param_loader.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

target_link_libraries(${PROJECT_NAME} casadi)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_ekf_state_estimator.cpp)
  set(TEST_MPC_EXE test_ekf_state_estimator)
  ament_add_gtest(${TEST_MPC_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_MPC_EXE} ${PROJECT_NAME})
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  param
  test_data
)
