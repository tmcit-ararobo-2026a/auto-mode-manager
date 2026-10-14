cmake_minimum_required(VERSION 3.18)
project(auto_mode_manager LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoModeStatus.msg" "msg/SafetyState.msg"
  "msg/ExecutionPath.msg" "msg/SimulationState.msg"
  DEPENDENCIES builtin_interfaces nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
option(BUILD_RUNTIME_NODES "Build manager runtime nodes" ON)
if(BUILD_RUNTIME_NODES)
  install(PROGRAMS scripts/auto_mode_manager_runtime.py scripts/navigation_safety_monitor.py
    DESTINATION lib/${PROJECT_NAME})
  install(FILES scripts/contract_core.py scripts/runtime_common.py scripts/tracking_controller.py DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/auto_mode_manager_runtime.py DESTINATION lib/${PROJECT_NAME}
    RENAME auto_mode_manager_node)
endif()
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING AND BUILD_RUNTIME_NODES AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/manager_foundation_test.py")
  find_package(ament_cmake_pytest REQUIRED)
  install(PROGRAMS test/mock_mainboard_fixture.py test/simulation_fixture.py DESTINATION lib/${PROJECT_NAME})
  ament_add_pytest_test(manager_foundation_test test/manager_foundation_test.py
    ENV MANAGER_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/auto_mode_manager_node
    TIMEOUT 30)
  ament_add_pytest_test(communication_launch_test test/communication_launch_test.py
    ENV TEST_LAUNCH_FILE=${CMAKE_CURRENT_SOURCE_DIR}/launch/integration_test.launch.py
        FIXTURE_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mock_mainboard_fixture.py
    TIMEOUT 30)
endif()
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING AND BUILD_RUNTIME_NODES AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/manager_foundation_test.py")
  ament_add_pytest_test(tracking_controller_test test/tracking_controller_test.py TIMEOUT 10)
  ament_add_pytest_test(simulation_integration_test test/simulation_integration_test.py
    ENV TEST_LAUNCH_FILE=${CMAKE_CURRENT_SOURCE_DIR}/launch/integration_test.launch.py TIMEOUT 90)
  ament_add_pytest_test(clock_fault_test test/clock_fault_test.py
    ENV TEST_LAUNCH_FILE=${CMAKE_CURRENT_SOURCE_DIR}/launch/integration_test.launch.py TIMEOUT 45)
  ament_add_pytest_test(safety_geometry_test test/safety_geometry_test.py TIMEOUT 10)
  ament_add_pytest_test(async_event_test test/async_event_test.py TIMEOUT 10)
  ament_add_pytest_test(contract_core_test test/contract_core_test.py TIMEOUT 10)
endif()

ament_package()
