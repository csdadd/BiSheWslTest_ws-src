cmake_minimum_required(VERSION 3.8)
project(qt5_nav2_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(yaml-cpp REQUIRED)

# Qt5 MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(include)

# Executable
add_executable(nav2_view
  src/main.cpp
  src/nav2_view_widget.cpp
  include/qt5_nav2_display/nav2_view_widget.hpp
)

target_link_libraries(nav2_view
  Qt5::Widgets
  Qt5::Gui
  Qt5::Core
  yaml-cpp
)

ament_target_dependencies(nav2_view
  rclcpp
  ament_index_cpp
  nav_msgs
  geometry_msgs
)

install(TARGETS nav2_view
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
