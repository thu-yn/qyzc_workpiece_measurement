# Pose Measurement System

一个基于点云配准的3D物体姿态估计和测量系统，适用于不规则金属件的几何参数测量。

## 功能特性

- **3D姿态估计**：通过点云配准计算物体的6自由度姿态
- **几何测量**：输出长度、宽度、高度、体积、表面积等几何参数
- **多种配准算法**：支持PCA对齐、FPFH+RANSAC粗配准和Point-to-Plane ICP精配准
- **质量评估**：提供配准质量指标（fitness score, RMSE）
- **数据输出**：支持终端显示和CSV文件保存
- **双模式处理**：基础模式（快速）和扩展模式（精确）

## 系统要求

- Ubuntu 16.04/18.04/20.04
- ROS Melodic/Noetic
- PCL 1.8+
- Eigen3

## 安装依赖

```bash
# 安装PCL和相关依赖
sudo apt-get update
sudo apt-get install libpcl-dev pcl-tools
sudo apt-get install libeigen3-dev
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions
sudo apt-get install ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-geometry-msgs

# 检查PCL版本 (应该是1.8+)
pcl_viewer --version
```

## 编译

```bash
# 创建工作空间（如果还没有）
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆或复制项目到src目录
# git clone <your_repo> 或者直接复制文件夹

# 编译
cd ~/catkin_ws
catkin_make

# 设置环境变量
source devel/setup.bash
```

## 代码目录

pose_measurement/
├── CMakeLists.txt                           # 构建配置文件
├── package.xml                              # ROS包描述文件
├── README.md                                # 项目说明文档
│
├── include/pose_measurement/                # 头文件目录
│   ├── pose_estimator.h                     # 核心姿态估计算法头文件
│   ├── measurement_calculator.h             # 几何测量计算头文件
│   └── point_cloud_processor.h              # 点云预处理头文件
│
├── src/                                     # 源代码目录
│   ├── pose_measurement_node.cpp            # 主节点程序
│   ├── pose_estimator.cpp                   # 姿态估计算法实现
│   ├── measurement_calculator.cpp           # 几何测量计算实现
│   └── point_cloud_processor.cpp            # 点云预处理实现
│
├── launch/                                  # 启动文件目录
│   └── pose_measurement.launch              # 主启动文件
│
├── scripts/                                 # 脚本目录
│   └── test_dataset.py                      # 批量测试脚本
│
├── data/                                    # 数据目录 (需要手动创建)
│   ├── reference.pcd                        # 标准姿态点云 (用户提供)
│   ├── target_001.pcd                       # 待测点云1 (用户提供)
│   ├── target_002.pcd                       # 待测点云2 (用户提供)
│   └── ...                                  # 更多测试数据
│
└── results/                                 # 结果输出目录 (需要手动创建)
    ├── measurements.csv                      # 默认CSV输出文件
    ├── result_target_001.csv                # 单独结果文件
    ├── merged_results.csv                   # 合并的批量结果
    └── ...                                  # 其他结果文件

## 使用方法

### 1. 准备数据

创建数据目录并放入PCD文件：

```bash
mkdir -p ~/catkin_ws/src/pose_measurement/data
mkdir -p ~/catkin_ws/src/pose_measurement/results

# 将你的标准姿态点云重命名为 reference.pcd
# 将待测点云放入 data/ 目录
```

### 2. 基础模式（默认）

基础模式提供快速的点云配准和基本几何测量：

```bash
# 处理单个目标点云
roslaunch pose_measurement pose_measurement.launch \
    reference_pcd:=/path/to/reference.pcd \
    target_pcd:=/path/to/target.pcd \
    output_csv:=/path/to/output.csv
```

### 3. 扩展模式（高精度）

扩展模式包括高级点云处理、平面移除、聚类分割和详细几何测量：

```bash
# 启用扩展模式
roslaunch pose_measurement pose_measurement.launch \
    reference_pcd:=/path/to/reference.pcd \
    target_pcd:=/path/to/target.pcd \
    enable_advanced:=true \
    enable_plane_removal:=true \
    enable_clustering:=true
```

### 4. 交互模式

```bash
# 交互模式（可连续处理多个文件）
roslaunch pose_measurement pose_measurement.launch \
    reference_pcd:=/path/to/reference.pcd
    
# 然后在终端中输入目标PCD文件路径
```

### 5. 批量处理

```bash
# 批量处理数据集
python3 scripts/test_dataset.py /path/to/dataset/ /path/to/reference.pcd --merge
```

## 参数配置

### 基础参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `voxel_size` | 0.005 | 下采样体素大小（米） |
| `icp_max_iterations` | 100 | ICP最大迭代次数 |
| `icp_transformation_epsilon` | 1e-6 | ICP收敛阈值 |

### 扩展模式参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_advanced` | false | 启用扩展处理模式 |
| `enable_plane_removal` | true | 启用平面移除 |
| `enable_clustering` | true | 启用聚类分割 |

### 参数调优示例

```bash
# 小物体高精度处理
roslaunch pose_measurement pose_measurement.launch \
    reference_pcd:=small_object_ref.pcd \
    target_pcd:=small_object_tgt.pcd \
    enable_advanced:=true \
    voxel_size:=0.001 \
    icp_max_iter:=200

# 大物体快速处理
roslaunch pose_measurement pose_measurement.launch \
    reference_pcd:=large_object_ref.pcd \
    target_pcd:=large_object_tgt.pcd \
    voxel_size:=0.01 \
    icp_max_iter:=50
```

## 输出说明

### 终端输出

```
=== Measurement Results ===
File: target_001.pcd
Processing time: 2.145 seconds

--- Pose Information ---
Translation (x, y, z): 0.012500, -0.008300, 0.045200 [m]
Euler Angles (roll, pitch, yaw): 2.3°, -1.8°, 15.7°

--- Geometric Measurements ---
Length: 0.125600 [m]
Width:  0.089400 [m]
Height: 0.034200 [m]
Volume: 0.000478 [m³]           # 扩展模式新增
Surface Area: 0.021567 [m²]     # 扩展模式新增
Key Angles: 12.5°, 45.2°, 78.9°

--- Registration Quality ---
Fitness Score: 0.000123
RMSE: 0.002456 [m]

--- Transformation Matrix ---
0.965926  -0.258819   0.000000   0.012500
0.258819   0.965926   0.000000  -0.008300
0.000000   0.000000   1.000000   0.045200
0.000000   0.000000   0.000000   1.000000
```

### CSV输出

| 列名 | 说明 | 单位 | 模式 |
|------|------|------|------|
| `filename` | 文件名 | - | 全部 |
| `processing_time` | 处理时间 | 秒 | 全部 |
| `tx, ty, tz` | 平移量 | 米 | 全部 |
| `roll, pitch, yaw` | 欧拉角 | 度 | 全部 |
| `length, width, height` | 几何尺寸 | 米 | 全部 |
| `volume` | 体积 | 米³ | 扩展模式更精确 |
| `surface_area` | 表面积 | 米² | 扩展模式更精确 |
| `fitness, rmse` | 配准质量指标 | - | 全部 |

## 算法流程

1. **预处理**：
   - 基础模式：去噪声 + 下采样
   - 扩展模式：高级去噪 + 平面移除 + 聚类分割
2. **粗配准**：
   - 首先尝试PCA主轴对齐
   - 失败则使用FPFH特征+RANSAC
3. **精配准**：Point-to-Plane ICP优化
4. **测量计算**：
   - 基础模式：长宽高 + 简单体积估算
   - 扩展模式：详细几何特征 + 精确体积表面积
5. **质量评估**：计算配准精度指标

## 数据集使用建议

当使用多视角数据集时：

1. **选择参考视角**：选择最完整、噪声最少的视角作为`reference.pcd`
2. **视角作为姿态**：将不同视角视为不同姿态进行测试
3. **模式选择**：
   - 开发调试阶段：使用基础模式快速迭代
   - 最终精确测量：使用扩展模式获得高精度结果
4. **批量处理**：使用脚本批量处理多个视角

## 性能对比

| 特性 | 基础模式 | 扩展模式 |
|------|----------|----------|
| 处理速度 | 快 | 较慢 |
| 测量精度 | 基础 | 高精度 |
| 背景干扰抗性 | 一般 | 强（自动移除平面） |
| 多物体处理 | 无 | 自动选择最大物体 |
| 体积测量 | 包络盒估算 | 凸包精确计算 |
| 表面积测量 | 包络盒估算 | 三角网格精确计算 |

## 故障排除

### 常见问题

1. **编译错误**：
   ```bash
   # 安装缺失依赖
   sudo apt-get install libpcl-dev libpcl-all-dev libeigen3-dev
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **配准失败**：
   - 检查两个点云是否为同一物体
   - 尝试扩展模式：`enable_advanced:=true`
   - 调整下采样参数：`voxel_size:=0.01`
   - 增加ICP迭代次数：`icp_max_iter:=200`

3. **精度不足**：
   - 使用扩展模式获得更高精度
   - 减小体素大小：`voxel_size:=0.001`
   - 提高ICP收敛精度：`icp_epsilon:=1e-8`

4. **处理速度慢**：
   - 使用基础模式：`enable_advanced:=false`
   - 增大体素大小：`voxel_size:=0.01`
   - 减少ICP迭代次数：`icp_max_iter:=50`

## 扩展建议

- 添加更多配准算法（如Go-ICP）
- 集成深度学习特征提取
- 支持多物体检测和分割
- 添加实时可视化界面
- GPU加速处理

## 许可证

MIT License