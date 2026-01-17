cmake_minimum_required(VERSION 3.5)

project(renwu VERSION 0.1 LANGUAGES CXX)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_VERSION VERSION_LESS "3.16")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Concurrent Sql Test)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
        mapmarker.h
        mapmarker.cpp
        statusindicator.h
        statusindicator.cpp
        navigationactionclient.h
        navigationactionclient.cpp
        pathvisualizer.h
        pathvisualizer.cpp
        user.h
        userstorageengine.h
        userstorageengine.cpp
        userauthmanager.h
        userauthmanager.cpp
        logindialog.h
        logindialog.cpp
        logindialog.ui
        usermanagementdialog.h
        usermanagementdialog.cpp
        usermanagementdialog.ui
        changepassworddialog.h
        changepassworddialog.cpp
        changepassworddialog.ui
        adduserdialog.h
        adduserdialog.cpp
        adduserdialog.ui
)

add_executable(renwu
    ${PROJECT_SOURCES}
)

target_link_libraries(renwu PRIVATE
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Sql
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
    tf2
    tf2_ros
    tf2_geometry_msgs
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
        test/testnavigationactionclient.cpp
        test/testnavigationactionclient.h
        test/testpathvisualizer.cpp
        test/testpathvisualizer.h
        test/testnavigationintegration.cpp
        test/testnavigationintegration.h
        test/testuser.cpp
        test/testuser.h
        test/testuserstorageengine.cpp
        test/testuserstorageengine.h
        test/testuserauthmanager.cpp
        test/testuserauthmanager.h
        test/testlogindialog.cpp
        test/testlogindialog.h
        test/testusermanagementdialog.cpp
        test/testusermanagementdialog.h
        test/testlogstorageengine.cpp
        test/testlogstorageengine.h
        test/testlogtablemodel.cpp
        test/testlogtablemodel.h
        test/testlogfilterproxymodel.cpp
        test/testlogfilterproxymodel.h
        test/testlogquerytask.cpp
        test/testlogquerytask.h
        test/testlogthread.cpp
        test/testlogthread.h
        test/testbasethread.cpp
        test/testbasethread.h
        test/testroscontextmanager.cpp
        test/testroscontextmanager.h
        test/testthreadsafequeue.cpp
        test/testthreadsafequeue.h
        test/testrobotstatusthread.cpp
        test/testrobotstatusthread.h
        test/testnavstatusthread.cpp
        test/testnavstatusthread.h
        test/testsystemmonitorthread.cpp
        test/testsystemmonitorthread.h
        test/testinfothread.cpp
        test/testinfothread.h
        test/testmapmarker.cpp
        test/testmapmarker.h
        test/teststatusindicator.cpp
        test/teststatusindicator.h
        test/testmapcache.cpp
        test/testmapcache.h
        test/testmapthread.cpp
        test/testmapthread.h
        test/testmainwindow.cpp
        test/testmainwindow.h
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
        mapmarker.h
        mapmarker.cpp
        statusindicator.h
        statusindicator.cpp
        navigationactionclient.h
        navigationactionclient.cpp
        pathvisualizer.h
        pathvisualizer.cpp
        user.h
        userstorageengine.h
        userstorageengine.cpp
        userauthmanager.h
        userauthmanager.cpp
        logindialog.h
        logindialog.cpp
        logindialog.ui
        usermanagementdialog.h
        usermanagementdialog.cpp
        usermanagementdialog.ui
        changepassworddialog.h
        changepassworddialog.cpp
        changepassworddialog.ui
        adduserdialog.h
        adduserdialog.cpp
        adduserdialog.ui
    )

    add_executable(renwu_tests ${TEST_SOURCES})

    target_link_libraries(renwu_tests PRIVATE
        Qt5::Widgets
        Qt5::Test
        Qt5::Sql
        Qt5::Concurrent
    )

    ament_target_dependencies(renwu_tests PUBLIC
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
        tf2
        tf2_ros
        tf2_geometry_msgs
    )

    target_include_directories(renwu_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    add_test(NAME renwu_tests COMMAND renwu_tests)
endif()

install(
  TARGETS
  renwu
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
