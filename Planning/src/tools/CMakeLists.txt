cmake_minimum_required(VERSION 3.16)
project(tools)


# 收集 tools 下的所有源文件
file(GLOB TOOLS_SRCS "*.cpp")

set(TARGET tools)

# 构建动态库 ${TARGET}
add_library(${TARGET} SHARED ${TOOLS_SRCS})

# 如果 tools 需要额外包含头文件路径，可设置，如：
target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
target_include_directories(${TARGET} PUBLIC ${EIGEN3_INCLUDE_DIRS} )
target_include_directories(${TARGET} PUBLIC ${OpenCV_INCLUDE_DIRS} )





# 链接依赖（如果工具库需要依赖其它库，可以在这里添加）
ament_target_dependencies(${TARGET}
  rclcpp
  Eigen3
  std_msgs
  visualization_msgs
  ros2_path_interfaces
  OpenMP
  yaml-cpp
  OpenCV
)
# 如果 OSQP 未被 ament_target_dependencies 完全链接，可以显示链接导入目标：
target_link_libraries(${TARGET} osqp::osqp)

# 设置编译选项
target_compile_options(${TARGET} PRIVATE ${COMPILE_OPTIONS})

# 安装动态库
install(TARGETS ${TARGET}
  DESTINATION lib/${PROJECT_NAME})