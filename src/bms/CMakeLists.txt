cmake_minimum_required(VERSION 3.8)
project(bms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖，增加 rclcpp_components
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) # 新增
find_package(sensor_msgs REQUIRED)

include_directories(include)

# 2. 生成共享库 (Shared Library)，这是组件的核心
# 我们把 src/bms.cpp 编译成库，供组件容器加载
add_library(bms_component SHARED src/bms.cpp)
target_compile_definitions(bms_component PRIVATE "BMS_COMPONENT_BUILD")
ament_target_dependencies(bms_component rclcpp rclcpp_components sensor_msgs)

# 3. 注册组件 (这一步会生成插件索引)
rclcpp_components_register_nodes(bms_component "TwsBmsNode")

# 4. 生成独立可执行文件 (Standalone Executable)
# 这样你依然可以用 ros2 run bms bms_node 启动
add_executable(bms_node src/bms.cpp)
ament_target_dependencies(bms_node rclcpp rclcpp_components sensor_msgs)

# 5. 安装规则
install(TARGETS bms_component bms_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()