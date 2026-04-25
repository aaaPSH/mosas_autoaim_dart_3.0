cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(detect_base)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



ament_auto_add_library(${PROJECT_NAME} SHARED 
  src/GreenDotDetectNode.cpp
  src/GreenDotDetect.cpp
  
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}       
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
)



rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "detect_base::GreenDotDetectNode"
  EXECUTABLE green_dot_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)