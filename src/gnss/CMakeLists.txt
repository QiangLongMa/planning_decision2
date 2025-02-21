cmake_minimum_required(VERSION 3.5)
project(gnss)

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  "msg/Imu.msg"
  "msg/Ins.msg"
  "msg/Message.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/config 
                    ${PROJECT_SOURCE_DIR}/parser 
                    ${PROJECT_SOURCE_DIR}/stream
                    ${PROJECT_SOURCE_DIR}/util)

add_library(${PROJECT_NAME}_lib SHARED 
            ${PROJECT_SOURCE_DIR}/parser/data_parser.cc 
            ${PROJECT_SOURCE_DIR}/parser/novatel_parser.cc
            ${PROJECT_SOURCE_DIR}/stream/raw_stream.cc 
            ${PROJECT_SOURCE_DIR}/stream/serial_stream.cc)     

add_executable(gnss_node gnss_node.cc)
target_link_libraries(gnss_node ${PROJECT_NAME}_lib)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs)
ament_target_dependencies(gnss_node rclcpp std_msgs)
rosidl_target_interfaces(gnss_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(gnss_sync_node gnss_sync_node.cc)
ament_target_dependencies(gnss_sync_node rclcpp std_msgs message_filters)
rosidl_target_interfaces(gnss_sync_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gnss_node
  gnss_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
