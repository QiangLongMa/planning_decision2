cmake_minimum_required(VERSION 3.16)
project(text_global)

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

# 调试信息
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
# 禁用优化
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(COMPILE_OPTIONS -Wall -Wextra -Wpedantic -O0)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


# 设置依赖
set(DEPENDS
  "rclcpp"
  "std_msgs"
  "visualization_msgs"
)

# 构建主程序（假设 main.cpp 在 src 目录下）
file(GLOB MAIN_SRCS "src/*.cpp")

set(TARGET text_global_node)
add_executable(${TARGET} ${MAIN_SRCS})

# 包含Eigen3头文件
target_include_directories(${TARGET} PRIVATE ${EIGEN3_INCLUDE_DIR})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 链接库
ament_target_dependencies(${TARGET} ${DEPENDS})

# 安装可执行文件
install(TARGETS ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
