cmake_minimum_required(VERSION 3.8)
project(bms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)

# 2. Build Core Protocol Library (Pure C++)
add_library(bms_protocol_lib SHARED src/bms_protocol.cpp)
target_include_directories(bms_protocol_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Build ROS 2 Component
add_library(bms_component SHARED src/bms_driver.cpp)
target_compile_definitions(bms_component PRIVATE "BMS_COMPONENT_BUILD")
ament_target_dependencies(bms_component rclcpp rclcpp_components sensor_msgs std_srvs)
target_link_libraries(bms_component bms_protocol_lib)

# 4. Register ROS 2 Component
rclcpp_components_register_nodes(bms_component "TwsBmsNode")

# 5. Build Standalone ROS 2 Node
add_executable(bms_node src/bms_driver.cpp)
ament_target_dependencies(bms_node rclcpp rclcpp_components sensor_msgs std_srvs)
target_link_libraries(bms_node bms_protocol_lib)

# 6. Build Python Bindings
pybind11_add_module(tws_bms_api src/pybind_module.cpp)
target_link_libraries(tws_bms_api PRIVATE bms_protocol_lib)

# 7. Installation rules
install(TARGETS bms_protocol_lib bms_component bms_node tws_bms_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
