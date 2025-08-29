cmake_minimum_required(VERSION 3.8)
project(pid_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 声明消息文件
set(msg_files
  "msg/Command.msg"
)

# 配置生成接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs  # ← Command.msg 中若用到了 std_msgs 下的类型需保留
)

# 如果其他包要依赖本包的接口
ament_export_dependencies(rosidl_default_runtime)

ament_package()
