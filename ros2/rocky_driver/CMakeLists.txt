cmake_minimum_required(VERSION 3.16)
project(rocky_driver)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(rocky_system SHARED src/rocky_system.cpp)
target_include_directories(rocky_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rocky_system
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface rocky_driver.xml)

install(TARGETS rocky_system LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/hw_bridge_node.py DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(rocky_system)
ament_package()
