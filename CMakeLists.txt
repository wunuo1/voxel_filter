cmake_minimum_required(VERSION 3.5)
project(point_cloud_processing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(point_cloud_sparsification src/point_cloud_sparsification.cpp)

ament_target_dependencies(point_cloud_sparsification
    rclcpp
    sensor_msgs
    PCL
    pcl_conversions
)
target_link_libraries(point_cloud_sparsification
  ${PCL_LIBRARIES}
)

install(TARGETS
    point_cloud_sparsification
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
