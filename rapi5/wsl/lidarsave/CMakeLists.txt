################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(lidarsave)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows(MSVC) 환경에서 M_PI 같은 수학 상수 사용을 가능하게 하는 매크로
if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()



################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

################################################################################
# Build
################################################################################
include_directories(include)


# add_executable(lidarsave src/lidarsave.cpp)
add_executable(
  lidarsave
  src/lidarsave.cpp
)
ament_target_dependencies(lidarsave
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  OpenCV
)


install(
  TARGETS lidarsave
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(OpenCV)
ament_export_include_directories(include)
ament_package()
