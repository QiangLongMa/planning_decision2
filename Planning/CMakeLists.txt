cmake_minimum_required(VERSION 3.16)
project(planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 设置编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros2_path_interfaces REQUIRED)
find_package(OpenMP REQUIRED)
find_package(osqp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)


# 设置依赖
set(DEPENDS
  "rclcpp"
  "std_msgs"
  "visualization_msgs"
  "ros2_path_interfaces"
)

# 将./src下的源文件编译成可执行文件
file(GLOB ALL_SRCS "src/*.cpp")

# 获取tools目录下的所有 .cpp 文件，并追加到 ALL_SRCS
file(GLOB TOOL_SRCS "src/tools/*.cpp")
list(APPEND ALL_SRCS ${TOOL_SRCS})

# 设置目标名称
set(TARGET planning_main_node)

# 添加可执行文件
add_executable(${TARGET} ${ALL_SRCS})

# 包含Eigen3头文件
target_include_directories(${TARGET} PRIVATE ${EIGEN3_INCLUDE_DIR})

# 包含OpenCV头文件
target_include_directories(${TARGET} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(${TARGET} PRIVATE ${osqp_INCLUDE_DIRS} )
target_include_directories(${TARGET} PRIVATE ${YAML_CPP_INCLUDE_DIR})


# 链接库文件
target_link_libraries(${TARGET} ${OpenCV_LIBS})
target_link_libraries(${TARGET}  osqp::osqp)
target_link_libraries(${TARGET} yaml-cpp)

# 设置编译选项
target_compile_options(${TARGET} PRIVATE ${COMPILE_OPTIONS})

# 链接库
ament_target_dependencies(${TARGET} ${DEPENDS})

# 安装可执行文件
install(TARGETS ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
