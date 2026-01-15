ament_add_gtest(test_recoveries
  test_recoveries.cpp
)

ament_target_dependencies(test_recoveries
  ${dependencies}
)

# ThreePointTurn 单元测试
# 添加 plugins 目录到 include 路径，因为 three_point_turn.hpp 在 plugins/ 目录下
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../plugins)

ament_add_gtest(test_three_point_turn
  test_three_point_turn.cpp
)

# 链接到 ThreePointTurn 库
target_link_libraries(test_three_point_turn
  nav2_three_point_turn_recovery
)

ament_target_dependencies(test_three_point_turn
  ${dependencies}
  ackermann_msgs
)
