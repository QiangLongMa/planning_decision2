cmake_minimum_required(VERSION 3.16)
project(senario)

# 收集 senario 下的所有源文件
file(GLOB SENARIO_SRCS "*.cpp")

set(TARGET senario)


# 构建动态库 senario_lib
add_library(${TARGET} SHARED ${SENARIO_SRCS})

# 设置 senario 动态库的头文件搜索路径
target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# 添加eigen库的头文件
target_include_directories(${TARGET} PUBLIC ${EIGEN3_INCLUDE_DIR})
# 添加tools的头文件搜索路径
target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../tools)

# 链接 senario 库所需依赖
ament_target_dependencies(${TARGET}
  rclcpp
  Eigen3
  std_msgs
  visualization_msgs
  ros2_path_interfaces
  OpenMP
  osqp
  yaml-cpp
  OpenCV
)

# 链接库依赖
target_link_libraries(${TARGET} tools)

# 设置编译选项
target_compile_options(${TARGET} PRIVATE ${COMPILE_OPTIONS})

# 安装动态库
install(TARGETS ${TARGET}
  DESTINATION lib/${PROJECT_NAME})