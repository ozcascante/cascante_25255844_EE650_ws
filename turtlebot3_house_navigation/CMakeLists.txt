cmake_minimum_required(VERSION 3.5)
project(turtlebot3_house_navigation)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(gazebo_msgs REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    plansys2_msgs
    nav2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser   
    gazebo_msgs
    std_msgs
    lifecycle_msgs
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(patrol_action_node src/patrol_action_node.cpp)
ament_target_dependencies(patrol_action_node ${dependencies})

add_executable(patrolling_controller_node src/patrolling_controller_node.cpp)
ament_target_dependencies(patrolling_controller_node ${dependencies})

add_executable(problem_generator_node src/problem_generator_node.cpp)
ament_target_dependencies(problem_generator_node ${dependencies})

add_executable(turn_off_light_action_node src/turn_off_light_action_node.cpp)
ament_target_dependencies(turn_off_light_action_node ${dependencies})

install(DIRECTORY launch pddl worlds params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  patrol_action_node
  patrolling_controller_node
  problem_generator_node
  turn_off_light_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
