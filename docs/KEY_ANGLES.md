# key_angles数组参考文档

本文档详细说明了本系统中`key_angles`数组在不同处理模式下的含义和用途。

## 📋 概述

`key_angles`数组是姿态测量系统的核心输出之一，包含了多种角度测量信息。根据系统运行模式的不同，该数组的长度和内容会有所差异：

- **基础模式**：8个角度值（索引0-7）
- **扩展模式**：20个角度值（索引0-19）

所有角度值均以**弧度**为单位存储，在CSV输出时会转换为**度数**。

---

## 🔧 基础模式（Basic Mode）

**条件**：`enable_advanced_measurements = false`

**数组长度**：8个元素

**数据来源**：`calculateWorkpieceRelativeAngles`函数

### 角度定义

| 索引 | 含义 | 计算方法 | 备注 |
|------|------|----------|------|
| `key_angles[0]` | Roll绝对值偏差 | `abs(euler_angles.x())` | 目标点云相对参考点云绕X轴的旋转偏差 |
| `key_angles[1]` | Pitch绝对值偏差 | `abs(euler_angles.y())` | 目标点云相对参考点云绕Y轴的旋转偏差 |
| `key_angles[2]` | Yaw绝对值偏差 | `abs(euler_angles.z())` | 目标点云相对参考点云绕Z轴的旋转偏差 |
| `key_angles[3]` | 总旋转偏差 | `abs(angle_axis.angle())` | 轴角表示的等效单轴旋转角度 |
| `key_angles[4]` | 旋转轴与X轴夹角 | `acos(abs(rotation_axis.x()))` | 旋转轴方向与X轴的夹角 |
| `key_angles[5]` | 旋转轴与Y轴夹角 | `acos(abs(rotation_axis.y()))` | 旋转轴方向与Y轴的夹角 |
| `key_angles[6]` | 旋转轴与Z轴夹角 | `acos(abs(rotation_axis.z()))` | 旋转轴方向与Z轴的夹角 |
| `key_angles[7]` | 综合偏差指标 | `sqrt(roll² + pitch² + yaw²)` | 三个欧拉角的欧几里得范数 |

### 应用场景

基础模式适用于：
- 快速姿态评估
- 实时质量检测
- 简单的偏差分析
- 批量处理场景

---

## 🚀 扩展模式（Advanced Mode）

**条件**：`enable_advanced_measurements = true`

**数组长度**：20个元素

**数据来源**：结合了几何特征分析和姿态偏差计算

### 角度定义

#### 📊 几何特征角度（索引0-11）

**来源**：`MeasurementCalculator::calculateCharacteristicAngles`

**主轴与坐标轴夹角：**

| 索引 | 含义 | 计算对象 |
|------|------|----------|
| `key_angles[0]` | 主轴1与X轴偏差 | 第一主轴（最大特征向量）与X轴夹角 |
| `key_angles[1]` | 主轴1与Y轴偏差 | 第一主轴与Y轴夹角 |
| `key_angles[2]` | 主轴1与Z轴偏差 | 第一主轴与Z轴夹角 |
| `key_angles[3]` | 主轴2与X轴偏差 | 第二主轴（中等特征向量）与X轴夹角 |
| `key_angles[4]` | 主轴2与Y轴偏差 | 第二主轴与Y轴夹角 |
| `key_angles[5]` | 主轴2与Z轴偏差 | 第二主轴与Z轴夹角 |
| `key_angles[6]` | 主轴3与X轴偏差 | 第三主轴（最小特征向量）与X轴夹角 |
| `key_angles[7]` | 主轴3与Y轴偏差 | 第三主轴与Y轴夹角 |
| `key_angles[8]` | 主轴3与Z轴偏差 | 第三主轴与Z轴夹角 |

**主轴之间的夹角：**

| 索引 | 含义 | 计算方法 |
|------|------|----------|
| `key_angles[9]` | 主轴1与主轴2偏差 | `acos(abs(axis1.dot(axis2)))` |
| `key_angles[10]` | 主轴1与主轴3偏差 | `acos(abs(axis1.dot(axis3)))` |
| `key_angles[11]` | 主轴2与主轴3偏差 | `acos(abs(axis2.dot(axis3)))` |

#### 📊 姿态偏差角度（索引12-19）

**来源**：`calculateWorkpieceRelativeAngles`函数

| 索引 | 含义 | 计算方法 | 备注 |
|------|------|----------|------|
| `key_angles[12]` | Roll绝对值偏差 | `abs(euler_angles.x())` | 与基础模式相同 |
| `key_angles[13]` | Pitch绝对值偏差 | `abs(euler_angles.y())` | 与基础模式相同 |
| `key_angles[14]` | Yaw绝对值偏差 | `abs(euler_angles.z())` | 与基础模式相同 |
| `key_angles[15]` | 总旋转偏差 | `abs(angle_axis.angle())` | 与基础模式相同 |
| `key_angles[16]` | 旋转轴与X轴夹角 | `acos(abs(rotation_axis.x()))` | 与基础模式相同 |
| `key_angles[17]` | 旋转轴与Y轴夹角 | `acos(abs(rotation_axis.y()))` | 与基础模式相同 |
| `key_angles[18]` | 旋转轴与Z轴夹角 | `acos(abs(rotation_axis.z()))` | 与基础模式相同 |
| `key_angles[19]` | 综合偏差指标 | `sqrt(roll² + pitch² + yaw²)` | 与基础模式相同 |

### 应用场景

扩展模式适用于：
- 详细的几何分析
- 复杂形状物体的测量
- 科研和开发场景
- 需要完整特征描述的应用

---

## 📝 CSV输出对应关系

在CSV文件中，根据不同模式会选择性输出`key_angles`的部分内容：

### 基础模式CSV输出

```cpp
// 对应 key_angles[0-7]
if (data.key_angles.size() >= 8) {
    file << data.key_angles[0] * 180.0 / M_PI << ","   // Roll偏差
         << data.key_angles[1] * 180.0 / M_PI << ","   // Pitch偏差
         << data.key_angles[2] * 180.0 / M_PI << ","   // Yaw偏差
         << data.key_angles[3] * 180.0 / M_PI << ","   // 总旋转偏差
         << data.key_angles[7] * 180.0 / M_PI << ",";  // 综合偏差
}
```

### 扩展模式CSV输出

```cpp
// 对应 key_angles[12-19]
if (enable_advanced_processing_ && data.key_angles.size() >= 20) {
    file << data.key_angles[12] * 180.0 / M_PI << ","  // Roll偏差
         << data.key_angles[13] * 180.0 / M_PI << ","  // Pitch偏差
         << data.key_angles[14] * 180.0 / M_PI << ","  // Yaw偏差
         << data.key_angles[15] * 180.0 / M_PI << ","  // 总旋转偏差
         << data.key_angles[19] * 180.0 / M_PI << ","; // 综合偏差
}
```

---

## 🔍 重要说明

### 单位转换

- **存储单位**：弧度（rad）
- **显示单位**：度（°）
- **转换公式**：`度 = 弧度 × 180 / π`

### 角度含义

1. **偏差角度**（索引12-14, 0-2）：表示目标物体相对参考物体的姿态差异
2. **几何角度**（索引0-11）：描述物体几何形状的空间方向特征
3. **旋转轴角度**（索引16-18, 4-6）：描述等效旋转轴的空间方向
4. **综合指标**（索引19, 7）：用于整体质量评估的单一数值

### 应用建议

- **质量检测**：主要关注偏差角度（roll, pitch, yaw）和综合指标
- **形状分析**：使用几何角度分析物体的主轴方向
- **旋转分析**：通过旋转轴角度理解变换的本质
- **自动化控制**：根据偏差角度为机械臂提供纠正参数

### 数值范围

- **角度范围**：0° - 180°（使用绝对值）
- **特殊值**：90°表示垂直关系
- **质量阈值**：通常小于5°为高精度，小于30°为可接受

---

## 📚 相关文档

- [pose_estimator.h](../include/pose_measurement/pose_estimator.h) - 姿态估计器接口定义
- [measurement_calculator.h](../include/pose_measurement/measurement_calculator.h) - 几何测量计算器接口
- [pose_measurement_node.cpp](../src/pose_measurement_node.cpp) - CSV输出实现
- [README.md](../README.md) - 项目完整文档

---

## 📞 技术支持

如有问题或建议，请联系thu.yangnan@outlook.com。