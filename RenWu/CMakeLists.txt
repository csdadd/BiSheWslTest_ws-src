cmake_minimum_required(VERSION 3.5)

project(RenWu VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 修复 GCC 11 与 Qt6 的兼容性问题
add_compile_options(-Wno-deprecated-declarations)

if(CMAKE_VERSION VERSION_LESS "3.16")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

find_package(ament_cmake REQUIRED)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        basethread.h
        basethread.cpp
        roscontextmanager.h
        roscontextmanager.cpp
        robotstatusthread.h
        robotstatusthread.cpp
        navstatusthread.h
        navstatusthread.cpp
        systemmonitorthread.h
        systemmonitorthread.cpp
        logthread.h
        logthread.cpp
        threadsafequeue.h
)

add_executable(RenWu
    ${PROJECT_SOURCES}
)

target_link_libraries(RenWu PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
)

ament_target_dependencies(RenWu PUBLIC
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    diagnostic_msgs
    action_msgs
    nav2_msgs
    rcl_interfaces
)

set_target_properties(RenWu PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
