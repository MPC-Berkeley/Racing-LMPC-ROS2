cmake_minimum_required(VERSION 3.8)
project(vanilla_controller)

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
  src/vanilla_controller.cpp
  src/ros_param_loader.cpp
  src/vanilla_controller_node.cpp
)

set(${PROJECT_NAME}_HEADER
  include/vanilla_controller/vanilla_controller.hpp
  include/vanilla_controller/vanilla_controller_config.hpp
  include/vanilla_controller/ros_param_loader.hpp
  include/vanilla_controller/vanilla_controller_node.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

target_link_libraries(${PROJECT_NAME} casadi)

ament_auto_add_executable(${PROJECT_NAME}_node_exe
  ${${PROJECT_NAME}_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_vanilla_controller.cpp)
  set(TEST_VANILLA_EXE test_vanilla_controller)
  ament_add_gtest(${TEST_VANILLA_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_VANILLA_EXE} ${PROJECT_NAME})
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  param
  launch
)
