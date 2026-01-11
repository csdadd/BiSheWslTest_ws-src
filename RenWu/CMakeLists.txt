cmake_minimum_required(VERSION 3.5)

project(renwu VERSION 0.1 LANGUAGES CXX)

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
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets Concurrent Sql Test REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Concurrent Sql Test)

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
        infothread.h
        infothread.cpp
        logthread.h
        logthread.cpp
        logtablemodel.h
        logtablemodel.cpp
        logfilterproxymodel.h
        logfilterproxymodel.cpp
        logstorageengine.h
        logstorageengine.cpp
        logquerytask.h
        logquerytask.cpp
        threadsafequeue.h
        mapconverter.h
        mapconverter.cpp
        mapthread.h
        mapthread.cpp
        mapwidget.h
        mapwidget.cpp
        maploader.h
        maploader.cpp
        mapcache.h
        mapcache.cpp
)

add_executable(renwu
    ${PROJECT_SOURCES}
)

target_link_libraries(renwu PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    Qt${QT_VERSION_MAJOR}::Sql
)

ament_target_dependencies(renwu PUBLIC
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

set_target_properties(renwu PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(BUILD_TESTING)
    enable_testing()

    set(TEST_SOURCES
        test/main.cpp
        test/testmapconverter.cpp
        test/testmapconverter.h
        test/testmapwidget.cpp
        test/testmapwidget.h
        test/testintegration.cpp
        test/testintegration.h
        test/testsystem.cpp
        test/testsystem.h
        test/testmaploader.cpp
        test/testmaploader.h
        basethread.h
        basethread.cpp
        roscontextmanager.h
        roscontextmanager.cpp
        mapconverter.h
        mapconverter.cpp
        mapthread.h
        mapthread.cpp
        mapwidget.h
        mapwidget.cpp
        maploader.h
        maploader.cpp
        mapcache.h
        mapcache.cpp
    )

    add_executable(renwu_tests ${TEST_SOURCES})

    target_link_libraries(renwu_tests PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Test
    )

    ament_target_dependencies(renwu_tests PUBLIC
        rclcpp
        nav_msgs
        diagnostic_msgs
        sensor_msgs
    )

    target_include_directories(renwu_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    add_test(NAME renwu_tests COMMAND renwu_tests)
endif()

ament_package()
