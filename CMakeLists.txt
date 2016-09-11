cmake_minimum_required(VERSION 2.8.3)
project(imu_serial_node)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  serial
  std_msgs
  std_srvs
  tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_serial_node src/mpu6050_serial_to_imu_node.cpp)
target_link_libraries(imu_serial_node   ${catkin_LIBRARIES} )
