cmake_minimum_required(VERSION 3.5)
project(mpclab_msgs)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ParametricPoseMsg.msg"
    "msg/ParametricVelocityMsg.msg"
    "msg/BodyAngularAccelerationMsg.msg"
    "msg/BodyAngularVelocityMsg.msg"
    "msg/BodyLinearAccelerationMsg.msg"
    "msg/BodyLinearVelocityMsg.msg"
    "msg/OrientationEulerMsg.msg"
    "msg/OrientationQuaternionMsg.msg"
    "msg/PositionMsg.msg"
    "msg/VehicleStateMsg.msg"
    "msg/EncoderMsg.msg"
    "msg/PredictionMsg.msg"
    "msg/TrackLookaheadMsg.msg"
    "msg/VehicleActuationMsg.msg"
    "msg/ControllerStatusMsg.msg"
    "msg/DriveStateMsg.msg"
    "msg/TimingMsg.msg"
    DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
