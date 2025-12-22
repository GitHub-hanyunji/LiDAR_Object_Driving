################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sllidar_ros2)

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

# sllidar_sdk가 위치한 상대 경로 지정
set(SLLIDAR_SDK_PATH "./sdk/")

# SLLIDAR SDK의 모든 C++ 소스 파일을 자동으로 수집
FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

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
include_directories(
  include
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

# 실행파일 노드 생성 (sllidar_node)
add_executable(sllidar_node	src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})


# target_link_libraries(sllidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  OpenCV
)

# add_executable(sllidar_client src/sllidar_client.cpp)
add_executable(
  sllidar_client
  src/main.cpp
  src/sllidar_client.cpp
)
ament_target_dependencies(sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  OpenCV
)
################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS sllidar_node sllidar_client
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
