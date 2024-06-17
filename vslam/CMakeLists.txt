cmake_minimum_required(VERSION 3.20)
project(cuda_vslam_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wnon-virtual-dtor -pedantic)

  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -DNDEBUG)
  elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(
      -O0 -ggdb -fno-omit-frame-pointer -fsanitize=undefined)
    add_link_options(-fsanitize=undefined)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(dependencies
  rclcpp
  foxglove_msgs
  sensor_msgs
  cv_bridge
  message_filters)

find_package(OpenCV REQUIRED)

add_library(vslam_lib
  src/vslam.cpp
  src/tracking/feature_extractor.cpp)
target_include_directories(vslam_lib
  PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(vslam_lib ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

add_executable(vslam_node src/vslam_node.cpp)
target_link_libraries(vslam_node
  vslam_lib
  ${OpenCV_LIBS})

add_executable(tum_bag_generator src/tools/tum_bag_generator.cpp)
target_include_directories(tum_bag_generator
  PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tum_bag_generator ${OpenCV_LIBS})
ament_target_dependencies(tum_bag_generator
  rclcpp
  sensor_msgs
  rosbag2_cpp
  cv_bridge)

add_executable(feature_plot src/tools/feature_plot.cpp)
target_include_directories(feature_plot
  PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(feature_plot
  vslam_lib
  ${OpenCV_LIBS})

install(TARGETS
  vslam_node
  tum_bag_generator
  feature_plot
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
