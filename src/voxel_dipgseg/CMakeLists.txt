cmake_minimum_required(VERSION 3.5)
project(dipgseg)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.54 REQUIRED)

include_directories(
  ${ament_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(demo_node src/demo.cpp)
ament_target_dependencies(demo_node
  rclcpp
  sensor_msgs
)
target_link_libraries(demo_node
  ${ament_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(TARGETS demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 launch
 DESTINATION share/${PROJECT_NAME}
)


ament_package()