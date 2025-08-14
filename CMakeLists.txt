# 文件名: CMakeLists.txt
# 文件作用: CMake构建配置文件，定义pose_measurement包的编译规则和依赖关系
#
# 主要功能需求:
# 1. 配置C++17编译标准（现代C++特性支持）
# 2. 查找和链接ROS相关包（roscpp、pcl_ros等）
# 3. 查找和配置PCL点云库（版本1.10+）
# 4. 查找和配置Eigen3数学库
# 5. 构建动态链接库和可执行文件
# 6. 设置正确的包含目录和链接关系
#
# 构建目标:
# - libpose_measurement.so：核心算法库
# - pose_measurement_node：主要可执行程序
#
# 依赖关系:
# - ROS核心包：roscpp、rospy、std_msgs等
# - PCL点云库：1.10版本以上，包含多个组件
# - Eigen3：线性代数数学库
# - 标准C++17库

# 设置CMake最低版本要求
# 3.0.2是ROS Melodic/Noetic常用的CMake版本
cmake_minimum_required(VERSION 3.0.2)

# 定义项目名称
# 这个名称将用于生成库文件和可执行文件
project(pose_measurement)

## 编译器配置
# 设置C++17标准，支持现代C++特性
add_compile_options(-std=c++17)

## ROS包依赖查找
# find_package查找catkin和所有必需的ROS包
find_package(catkin REQUIRED COMPONENTS
  roscpp                    # ROS C++客户端库，核心功能
  rospy                     # ROS Python客户端库，用于脚本
  std_msgs                  # ROS标准消息类型
  sensor_msgs               # 传感器消息类型（点云、图像等）
  geometry_msgs             # 几何消息类型（位置、姿态等）
  pcl_conversions           # PCL与ROS消息的转换工具
  pcl_ros                   # PCL在ROS中的集成包
  tf2                       # 变换库2.0（坐标系变换）
  tf2_ros                   # tf2的ROS接口
  tf2_geometry_msgs         # tf2几何消息支持
)

## 系统库依赖查找
# PCL点云库配置
# 要求1.10版本以上，包含核心处理组件
find_package(PCL 1.10 REQUIRED COMPONENTS 
  common                    # 基础数据结构和通用功能
  io                        # 文件输入输出（PCD、PLY等）
  features                  # 特征提取（FPFH、SHOT等）
  filters                   # 滤波器（体素网格、统计滤波等）
  registration              # 配准算法（ICP、RANSAC等）
  segmentation              # 分割算法（平面分割、聚类等）
)

# Eigen3数学库配置
# 用于矩阵运算、线性代数计算
find_package(Eigen3 REQUIRED)

## catkin包配置
# 定义包的公共接口和依赖关系
catkin_package(
  INCLUDE_DIRS include      # 公开的头文件目录
  LIBRARIES ${PROJECT_NAME} # 生成的库文件
  CATKIN_DEPENDS            # ROS包依赖（运行时需要）
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    pcl_conversions 
    pcl_ros 
    tf2 
    tf2_ros 
    tf2_geometry_msgs
  DEPENDS                   # 系统库依赖
    PCL 
    EIGEN3
)

## 头文件包含目录配置
# 设置编译时查找头文件的路径
include_directories(
  include                   # 项目本身的头文件目录
  ${catkin_INCLUDE_DIRS}    # catkin包的头文件目录
  ${PCL_INCLUDE_DIRS}       # PCL库的头文件目录
  ${EIGEN3_INCLUDE_DIRS}    # Eigen3库的头文件目录
)

## 构建动态链接库
# 创建pose_measurement库，包含核心算法实现
add_library(${PROJECT_NAME}
  src/pose_estimator.cpp              # 姿态估计器实现
  src/measurement_calculator.cpp      # 几何测量计算器实现
  src/point_cloud_processor.cpp       # 点云预处理器实现
)

## 库链接配置
# 将依赖库链接到我们的库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}       # ROS相关库
  ${PCL_LIBRARIES}          # PCL点云处理库
)

## 构建可执行文件
# 创建主要的ROS节点可执行文件
add_executable(pose_measurement_node 
  src/pose_measurement_node.cpp      # 主节点程序
)

## 可执行文件链接配置
# 链接库文件和依赖库到可执行文件
target_link_libraries(pose_measurement_node
  ${PROJECT_NAME}           # 我们自己的库
  ${catkin_LIBRARIES}       # ROS相关库
  ${PCL_LIBRARIES}          # PCL点云处理库
)

# ===== 可选的高级配置 =====

## 编译器优化标志（可选）
# 可以添加这些标志来优化性能：
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

## PCL预编译定义（可选）
# 禁用PCL的一些警告和过时功能：
# add_definitions(${PCL_DEFINITIONS})

## 调试配置（可选）
# 在Debug模式下添加调试符号：
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

## 安装配置（可选）
# 如果需要install目标，可以添加：
# install(TARGETS ${PROJECT_NAME} pose_measurement_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

## 测试配置（可选）
# 如果有单元测试，可以添加：
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(${PROJECT_NAME}_test test/test_pose_measurement.test test/test_pose_measurement.cpp)
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
# endif()

# ===== 构建说明 =====
#
# 编译步骤:
# 1. cd ~/catkin_ws
# 2. catkin_make
# 或者使用catkin build:
# 1. cd ~/catkin_ws
# 2. catkin build pose_measurement
#
# 清理构建:
# catkin_make clean
# 或者:
# rm -rf build/ devel/
#
# 依赖检查:
# rosdep install --from-paths src --ignore-src -r -y
#
# 常见问题:
# 1. PCL版本不兼容：确保安装PCL 1.10+
# 2. Eigen3找不到：sudo apt-get install libeigen3-dev
# 3. 编译错误：检查C++17支持
# 4. 链接错误：确保所有依赖库正确安装