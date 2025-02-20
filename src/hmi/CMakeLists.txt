cmake_minimum_required(VERSION 3.5)
project(hmi)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost 1.65.1 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
# find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(target_bbox_msgs REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}) 
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/*.cpp)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  include/*.h *.hpp)
#file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp src/runcenterpoint.cpp)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOUCE_HPP ${PRO_RESOURCE_DIR})
include_directories(
	include/
	${CMAKE_CURRENT_BINARY_DIR}
  # ${target_bbox_msgs_INCLUDE_DIRS}
  # ${visualization_msgs_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}
    ${PRO_SOURCE_DIR}
    ${PRO_INCLUDE_DIR}
    ${PRO_UI_DIR}
    ${QT_MOC_HPP}
    ${QT_RESOUCE_HPP}
  )
ament_target_dependencies(${PROJECT_NAME} rclcpp  Boost  std_msgs  sensor_msgs) #target_bbox_msgs

target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${OpenCV_LIBS}) # ${PCL_LIBRARIES}




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

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
