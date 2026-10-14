cmake_minimum_required(VERSION 3.18)
project(auto_mode_manager LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoModeStatus.msg" "msg/SafetyState.msg"
  DEPENDENCIES builtin_interfaces)
rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
option(BUILD_RUNTIME_NODES "Build manager runtime nodes" ON)
if(BUILD_RUNTIME_NODES)
  add_executable(auto_mode_manager_node src/auto_mode_manager_node.cpp)
  ament_target_dependencies(auto_mode_manager_node rclcpp)
  target_link_libraries(auto_mode_manager_node "${cpp_typesupport}")
  install(TARGETS auto_mode_manager_node DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING AND BUILD_RUNTIME_NODES)
  find_package(ament_cmake_pytest REQUIRED)
  install(PROGRAMS test/mock_mainboard_fixture.py DESTINATION lib/${PROJECT_NAME})
  ament_add_pytest_test(manager_foundation_test test/manager_foundation_test.py
    ENV MANAGER_EXECUTABLE=$<TARGET_FILE:auto_mode_manager_node>
    TIMEOUT 30)
  ament_add_pytest_test(communication_launch_test test/communication_launch_test.py
    ENV TEST_LAUNCH_FILE=${CMAKE_CURRENT_SOURCE_DIR}/launch/integration_test.launch.py
        FIXTURE_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mock_mainboard_fixture.py
    TIMEOUT 30)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
