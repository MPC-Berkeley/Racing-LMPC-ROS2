cmake_minimum_required(VERSION 3.8)
project(lmpc_utils)

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
find_package(ament_cmake_auto REQUIRED)
find_package(casadi REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/lookup.cpp
  src/logging.cpp
  src/primitives.cpp
  src/utils.cpp
  src/pid_controller.cpp
)

set(${PROJECT_NAME}_HEADER
  include/lmpc_utils/ros_param_helper.hpp
  include/lmpc_utils/lookup.hpp
  include/lmpc_utils/utils.hpp
  include/lmpc_utils/logging.hpp
  include/lmpc_utils/primitives.hpp
  include/lmpc_utils/casadi_primitives.hpp
  include/lmpc_utils/cycle_profiler.hpp
  include/lmpc_utils/pid_controller.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

target_link_libraries(${PROJECT_NAME}
  casadi
)

# build python utils
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_lmpc_utils.cpp)
  set(TEST_LMPC_UTILS_EXE test_lmpc_utils)
  ament_add_gtest(${TEST_LMPC_UTILS_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_LMPC_UTILS_EXE} ${PROJECT_NAME})
endif()

# Create & install ament package.
ament_auto_package()
