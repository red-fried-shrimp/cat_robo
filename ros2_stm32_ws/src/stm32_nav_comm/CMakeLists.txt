cmake_minimum_required(VERSION 3.8)                          # 指定CMake最低版本要求为3.8
project(stm32_nav_comm)                                     # 定义项目名称为stm32_nav_comm

# 如果编译器是GNU C++或Clang，添加编译警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)             # 启用所有警告、额外警告、严格语法检查
endif()

set(CMAKE_CXX_STANDARD 14)                                  # 设置C++标准为C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)                         # 强制要求使用指定的C++标准

# 查找并加载ROS2相关依赖包
find_package(ament_cmake REQUIRED)                          # ROS2构建系统核心包
find_package(rclcpp REQUIRED)                               # ROS2 C++客户端库
find_package(geometry_msgs REQUIRED)                        # 几何消息类型包（Twist等）
find_package(nav_msgs REQUIRED)                             # 导航消息类型包（Odometry等）
find_package(std_msgs REQUIRED)                             # 标准消息类型包（UInt8等）
find_package(tf2 REQUIRED)                                  # TF2核心库
find_package(tf2_ros REQUIRED)                              # TF2 ROS接口库
find_package(serial REQUIRED)                               # 串口通信库

# 添加头文件包含目录
include_directories(
  include                                                   # 包含项目根目录下的include文件夹
)

# 构建可执行文件，指定源文件
add_executable(stm32_comm_node                              # 可执行文件名称为stm32_comm_node
  src/stm32_comm_node.cpp                                   # 源文件路径
)

# 为可执行文件链接依赖包
ament_target_dependencies(stm32_comm_node
  rclcpp                                                    # 链接rclcpp库
  geometry_msgs                                             # 链接geometry_msgs库
  nav_msgs                                                  # 链接nav_msgs库
  std_msgs                                                  # 链接std_msgs库
  tf2                                                       # 链接tf2库
  tf2_ros                                                   # 链接tf2_ros库
  serial                                                    # 链接serial库
)

# 安装可执行文件到指定目录
install(TARGETS
  stm32_comm_node                                           # 要安装的可执行文件
  DESTINATION lib/${PROJECT_NAME}                           # 安装路径：lib/项目名
)

# 安装头文件到系统include目录
install(DIRECTORY include/                                  # 要安装的头文件目录
  DESTINATION include/                                      # 安装目标路径
)

# 安装launch文件夹到share目录
install(DIRECTORY launch                                    # 要安装的launch目录
  DESTINATION share/${PROJECT_NAME}                         # 安装路径：share/项目名
)

# 生成ROS2包相关的配置文件
ament_package()

