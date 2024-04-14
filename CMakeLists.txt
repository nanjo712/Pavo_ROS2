cmake_minimum_required(VERSION 3.8)
project(pavo_ros2)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(
    PAVO_SDK STATIC
    src/pavo_sdk/data_filters.cpp
    src/pavo_sdk/pavo_driver.cpp
    src/pavo_sdk/utils.cpp
)
target_link_libraries(PAVO_SDK PRIVATE Boost::system Boost::thread)
target_include_directories(PAVO_SDK PRIVATE include)

add_executable(pavo_scan_node src/pavo_scan_node.cpp)
ament_target_dependencies(pavo_scan_node rclcpp sensor_msgs)
target_link_libraries(pavo_scan_node PAVO_SDK)
target_include_directories(pavo_scan_node PRIVATE include)


ament_package()
