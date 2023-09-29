cmake_minimum_required(VERSION 3.8)
project(base_vehicle_model)

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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/base_vehicle_model.cpp
  src/ros_param_loader.cpp
)

set(${PROJECT_NAME}_HEADER
  include/base_vehicle_model/base_vehicle_model.hpp
  include/base_vehicle_model/base_vehicle_model_config.hpp
  include/base_vehicle_model/base_vehicle_model_state.hpp
  include/base_vehicle_model/ros_param_loader.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

target_link_libraries(${PROJECT_NAME} casadi)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_base_vehicle_model.cpp)
  set(TEST_BVM_EXE test_bvm)
  ament_add_gtest(${TEST_BVM_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_BVM_EXE} ${PROJECT_NAME})
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  param
)
