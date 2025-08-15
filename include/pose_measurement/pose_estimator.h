/*
* 文件名: pose_estimator.h
* 文件作用: 姿态估计器头文件，定义了用于3D物体姿态估计和几何测量的核心类
* 
* 主要功能需求:
* 1. 实现多层次点云配准算法（粗配准 + 精配准）
* 2. 支持多种配准策略（PCA主轴对齐、FPFH特征匹配、ICP优化）
* 3. 集成扩展处理模块（高级点云处理和几何测量）
* 4. 提供双模式处理（基础模式和扩展模式）
* 5. 计算6自由度姿态和详细几何参数
* 6. 提供配准质量评估（适配度和RMSE）
* 7. 生成变换后点云用于可视化对比（新增功能）
* 
* 设计模式: 
* - 组合模式：集成PointCloudProcessor和MeasurementCalculator
* - 策略模式：支持多种配准算法和处理模式
* - 模板方法：固定的处理流程，可配置的具体步骤
* 
* 适用场景: 工业零件检测、质量控制、机器人视觉、3D重建
* 依赖库: PCL点云库、Eigen数学库、ROS系统
*/

#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

// ===== ROS和PCL核心库 =====
#include <ros/ros.h>                           // ROS核心功能和日志系统
#include <pcl/point_cloud.h>                   // PCL点云数据结构
#include <pcl/point_types.h>                   // PCL点类型定义（PointXYZ等）

// ===== PCL配准算法库 =====
#include <pcl/registration/icp.h>              // ICP迭代最近点算法
#include <pcl/registration/sample_consensus_prerejective.h>  // RANSAC预筛选配准

// ===== PCL特征提取库 =====
#include <pcl/features/fpfh.h>                 // FPFH快速点特征直方图
#include <pcl/features/normal_3d.h>            // 3D法向量估计

// ===== PCL滤波和处理库 =====
#include <pcl/filters/voxel_grid.h>            // 体素网格下采样滤波器
#include <pcl/filters/statistical_outlier_removal.h>  // 统计异常值移除滤波器

// ===== PCL几何计算库 =====
#include <pcl/common/pca.h>                    // 主成分分析（PCA）
#include <pcl/common/transforms.h>             // 点云变换操作
#include <pcl/io/pcd_io.h>                     // PCD文件读写操作
#include <pcl/search/kdtree.h>                 // KD树搜索结构

// ===== 数学和标准库 =====
#include <Eigen/Dense>                         // Eigen线性代数库
#include <cmath>                              // 数学函数库
#include <algorithm>                          // STL算法库
#include <limits>                             // 数值极限定义

// ===== 扩展功能模块 =====
#include "pose_measurement/point_cloud_processor.h"    // 点云预处理器
#include "pose_measurement/measurement_calculator.h"   // 几何测量计算器

/**
* @brief 测量数据结构体
* 存储完整的姿态估计和几何测量结果，是系统的主要输出数据结构
*/
struct MeasurementData {
    // ===== 姿态信息 =====
    Eigen::Matrix4f transformation;           // 4x4变换矩阵（旋转+平移）
    Eigen::Vector3f translation;              // 平移向量（x,y,z）[米]
    Eigen::Vector3f euler_angles;             // 欧拉角（roll, pitch, yaw）[弧度]
    
    // ===== 基础几何测量 =====
    float length, width, height;              // 长、宽、高（按大小排序）[米]
    float volume, surface_area;               // 体积[立方米]和表面积[平方米]
    std::vector<float> key_angles;            // 关键角度集合[弧度]
    
    // ===== 扩展几何特征 =====
    pose_measurement::GeometricFeatures geometric_features;  // 详细几何特征（扩展模式）
    
    // ===== 配准质量指标 =====
    float registration_fitness;               // 配准适配度（越小越好）
    float registration_rmse;                  // 配准均方根误差[米]（越小越好）
};

/**
* @brief 姿态估计器主类
* 
* 核心功能：
* 1. 点云配准：通过配准目标点云到参考点云来估计姿态
* 2. 几何测量：计算物体的各种几何参数
* 3. 质量评估：评估配准和测量的可靠性
* 4. 变换点云生成：生成配准后的点云用于可视化对比
* 
* 工作流程：
* 1. 加载参考点云并预处理
* 2. 预处理目标点云
* 3. 粗配准（PCA或FPFH）
* 4. 精配准（ICP）
* 5. 计算几何测量
* 6. 生成变换后点云（可选）
* 7. 评估结果质量
* 
* 设计特点：
* - 双模式处理：基础模式（快速）和扩展模式（精确）
* - 智能配准：多种算法自动选择
* - 参数可配置：支持运行时调整
* - 鲁棒性：完善的错误处理和回退机制
* - 可视化支持：生成变换后点云用于对比分析
*/
class PoseEstimator {
public:
    // ===== 构造和析构函数 =====
    
    /**
    * @brief 构造函数
    * 初始化所有参数为工业级默认值，创建处理模块实例
    */
    PoseEstimator();
    
    /**
    * @brief 析构函数
    * 清理资源，释放内存
    */
    ~PoseEstimator();
    
    // ===== 主要处理功能 =====
    
    /**
    * @brief 加载参考点云
    * @param pcd_path PCD文件路径
    * @return bool 成功返回true，失败返回false
    * 
    * 功能说明：
    * 1. 从PCD文件加载参考点云（标准姿态）
    * 2. 预处理参考点云（去噪、下采样等）
    * 3. 预计算PCA特征（用于后续快速配准）
    * 4. 缓存处理结果以提高后续性能
    * 
    * 注意事项：
    * - 参考点云应该是物体的标准姿态
    * - 预处理参数会影响后续所有配准精度
    * - 该函数只需调用一次，结果会被缓存
    */
    bool loadReferenceCloud(const std::string& pcd_path);
    
    /**
    * @brief 处理目标点云
    * @param target_cloud 待处理的目标点云
    * @return MeasurementData 包含姿态和测量信息的完整结果
    * 
    * 功能说明：
    * 这是系统的主要接口函数，完成完整的处理流程：
    * 1. 预处理目标点云
    * 2. 执行粗配准获得初始估计
    * 3. 执行精配准优化结果
    * 4. 计算几何测量参数
    * 5. 评估配准质量
    * 
    * 算法流程：
    * 预处理 → 粗配准 → 精配准 → 测量计算 → 质量评估
    * 
    * 性能优化：
    * - 预处理结果可复用
    * - 配准算法智能选择
    * - 并行处理（OpenMP）
    */
    MeasurementData processTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
    
    // ===== 变换点云功能（新增）=====
    
    /**
    * @brief 生成变换后的点云
    * @param source_cloud 源点云（通常是预处理后的目标点云）
    * @param transformation 变换矩阵
    * @return 变换后的点云
    * 
    * 功能说明：
    * 1. 应用变换矩阵到源点云
    * 2. 生成与参考点云在同一坐标系下的点云
    * 3. 便于可视化对比和质量评估
    * 
    * 使用场景：
    * - 配准结果可视化
    * - 质量评估和调试
    * - 生成对比数据
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr generateTransformedCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        const Eigen::Matrix4f& transformation);
    
    /**
    * @brief 保存变换后的点云到PCD文件
    * @param transformed_cloud 变换后的点云
    * @param file_path 保存路径
    * @return bool 成功返回true，失败返回false
    * 
    * 功能说明：
    * 1. 将变换后的点云保存为PCD格式
    * 2. 支持用户自定义文件名和路径
    * 3. 提供完整的错误处理
    * 
    * 文件命名建议：
    * - transformed_[原文件名]_[时间戳].pcd
    * - aligned_[目标名称].pcd
    * - result_[处理批次].pcd
    */
    bool saveTransformedCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
        const std::string& file_path);
    
    // ===== 配置接口 =====
    
    /**
    * @brief 设置体素大小
    * @param size 体素大小[米]，影响下采样精度和处理速度
    * 
    * 参数选择指导：
    * - 小物体(< 10cm)：0.001-0.003m（1-3mm）
    * - 中等物体(10-50cm)：0.003-0.008m（3-8mm）
    * - 大物体(> 50cm)：0.008-0.02m（8-20mm）
    */
    void setVoxelSize(float size) { voxel_size_ = size; }
    
    /**
    * @brief 设置ICP最大迭代次数
    * @param iter 迭代次数，影响配准精度和处理时间
    * 
    * 推荐设置：
    * - 快速模式：50-100次
    * - 精确模式：100-200次
    * - 高精度模式：200-500次
    */
    void setICPMaxIterations(int iter) { icp_max_iterations_ = iter; }
    
    /**
    * @brief 设置ICP变换收敛阈值
    * @param eps 收敛阈值，越小越精确但可能影响收敛速度
    * 
    * 推荐设置：
    * - 快速模式：1e-4 到 1e-5
    * - 精确模式：1e-6 到 1e-7
    * - 高精度模式：1e-7 到 1e-8
    */
    void setICPTransformationEpsilon(float eps) { icp_transformation_epsilon_ = eps; }
    
    /**
    * @brief 启用/禁用扩展测量功能
    * @param enable true启用扩展模式，false使用基础模式
    * 
    * 模式对比：
    * 基础模式：快速处理，基本几何测量
    * 扩展模式：完整处理流水线，精确几何测量，平面移除，聚类分割
    */
    void setEnableAdvancedMeasurements(bool enable) { enable_advanced_measurements_ = enable; }
    
    /**
    * @brief 启用/禁用下采样
    * @param enable true启用下采样，false保持原始分辨率
    * 
    * 使用建议：
    * - 大点云：启用下采样提高处理速度
    * - 高精度需求：禁用下采样保持完整细节
    * - 实时应用：启用下采样平衡速度和精度
    */
    void setEnableDownsampling(bool enable) { enable_downsampling_ = enable; }
    
    /**
    * @brief 设置扩展处理参数
    * @param params 处理参数结构体
    * 
    * 包含的参数：
    * - 滤波参数：体素大小、统计滤波、半径滤波
    * - 分割参数：平面检测、聚类分割
    * - 开关控制：各功能模块的启用/禁用
    */
    void setProcessingParameters(const pose_measurement::ProcessingParameters& params);
    
    // ===== 状态查询接口 =====
    
    /**
    * @brief 获取下采样开关状态
    * @return bool 当前下采样开关状态
    */
    bool getEnableDownsampling() const { return enable_downsampling_; }
    
private:
    // ===== 私有处理函数 =====
    
    /**
    * @brief 预处理点云
    * @param cloud 输入点云
    * @return 预处理后的点云
    * 
    * 基础模式处理步骤：
    * 1. 统计异常值移除
    * 2. 体素网格下采样（可选）
    * 
    * 扩展模式由PointCloudProcessor处理：
    * 1. 多层次去噪
    * 2. 下采样
    * 3. 平面移除
    * 4. 聚类分割
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // ===== 配准算法实现 =====
    
    /**
    * @brief 粗配准主函数
    * @param source 源点云（目标点云）
    * @param target 目标点云（参考点云）
    * @return 粗配准变换矩阵
    * 
    * 智能配准策略：
    * 1. 首先尝试PCA主轴对齐（快速）
    * 2. 评估PCA结果质量
    * 3. 如果PCA失败，使用FPFH特征匹配（鲁棒）
    * 
    * 质量评估标准：
    * - 质心距离 < 阈值：PCA成功
    * - 质心距离 >= 阈值：切换到FPFH
    */
    Eigen::Matrix4f coarseRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    /**
    * @brief 精配准主函数
    * @param source 源点云
    * @param target 目标点云
    * @param initial_guess 初始变换估计（来自粗配准）
    * @return 精配准优化后的变换矩阵
    * 
    * ICP算法流程：
    * 1. 应用初始变换
    * 2. 迭代优化：找对应点→计算变换→更新点云
    * 3. 检查收敛条件
    * 4. 返回最终变换
    * 
    * 优化特性：
    * - Point-to-Plane ICP（精度更高）
    * - 自适应对应距离阈值
    * - 多重收敛条件检查
    */
    Eigen::Matrix4f fineRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target,
        const Eigen::Matrix4f& initial_guess);
    
    /**
    * @brief PCA主轴对齐
    * @param source 源点云
    * @param target 目标点云
    * @return PCA对齐变换矩阵
    * 
    * 算法原理：
    * 1. 计算两个点云的主轴（PCA）
    * 2. 对齐主轴方向（旋转）
    * 3. 对齐质心位置（平移）
    * 
    * 适用场景：
    * - 形状规整的物体
    * - 主轴方向明显的物体
    * - 需要快速初始对齐的场景
    * 
    * 优点：计算快速，无需特征计算
    * 缺点：对称物体可能对齐错误
    */
    Eigen::Matrix4f pcaAlignment(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    /**
    * @brief FPFH特征匹配对齐
    * @param source 源点云
    * @param target 目标点云
    * @return FPFH配准变换矩阵
    * 
    * 算法流程：
    * 1. 计算点云法向量
    * 2. 计算FPFH特征描述子
    * 3. 特征匹配和RANSAC筛选
    * 4. 估计变换矩阵
    * 
    * 适用场景：
    * - 复杂形状物体
    * - PCA对齐失败的情况
    * - 需要鲁棒配准的场景
    * 
    * 优点：对形状和姿态变化鲁棒
    * 缺点：计算量大，需要足够的特征点
    */
    Eigen::Matrix4f fpfhAlignment(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    // ===== 测量和分析函数 =====
    
    /**
    * @brief 计算测量数据
    * @param cloud 配准后的点云
    * @param transformation 配准变换矩阵
    * @return 完整的测量数据结构
    * 
    * 计算内容：
    * 1. 姿态信息：平移、旋转、欧拉角
    * 2. 基础几何：长宽高、体积、表面积
    * 3. 扩展几何：详细特征（扩展模式）
    * 4. 质量评估：适配度、RMSE
    * 
    * 双模式支持：
    * - 基础模式：快速估算
    * - 扩展模式：精确计算
    */
    MeasurementData calculateMeasurements(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const Eigen::Matrix4f& transformation);
    
    // ===== 工具函数 =====
    
    /**
    * @brief 旋转矩阵转欧拉角
    * @param R 3x3旋转矩阵
    * @return 欧拉角向量（roll, pitch, yaw）
    * 
    * 转换约定：
    * - 使用ZYX欧拉角顺序
    * - 处理奇点情况
    * - 角度范围：[-π, π]
    */
    Eigen::Vector3f rotationMatrixToEulerAngles(const Eigen::Matrix3f& R);
    
    /**
    * @brief 计算主轴
    * @param cloud 输入点云
    * @param eigenvectors 输出主轴方向矩阵
    * @param eigenvalues 输出特征值向量
    * 
    * PCA分析：
    * 1. 计算协方差矩阵
    * 2. 特征值分解
    * 3. 按特征值大小排序
    * 4. 标准化方向向量
    */
    void computePrincipalAxes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        Eigen::Matrix3f& eigenvectors,
        Eigen::Vector3f& eigenvalues);
    
    // ===== 私有成员变量 =====
    
    // --- 点云数据 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;           // 原始参考点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_processed_; // 预处理后的参考点云
    
    // --- 扩展处理模块 ---
    std::shared_ptr<pose_measurement::PointCloudProcessor> cloud_processor_;        // 点云处理器（扩展模式）
    std::shared_ptr<pose_measurement::MeasurementCalculator> measurement_calculator_; // 测量计算器（扩展模式）
    bool enable_advanced_measurements_;     // 扩展模式开关
    bool enable_downsampling_;              // 下采样开关
    
    // --- 预计算的参考点云特征 ---
    Eigen::Matrix3f ref_eigenvectors_;      // 参考点云主轴方向矩阵
    Eigen::Vector3f ref_eigenvalues_;       // 参考点云特征值
    Eigen::Vector3f ref_centroid_;          // 参考点云质心
    
    // --- 算法参数 ---
    float voxel_size_;                      // 体素大小[米]（影响下采样精度）
    int icp_max_iterations_;                // ICP最大迭代次数
    float icp_transformation_epsilon_;      // ICP变换收敛阈值
    float fpfh_radius_;                     // FPFH特征计算半径[米]
    
    // --- PCL对象（性能优化：避免重复创建） ---
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;                          // 体素网格滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;        // 统计异常值滤波器
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;  // 法向量估计器
};

#endif // POSE_ESTIMATOR_H

/*
=== 文件分析总结 ===

1. 架构设计：
   - 采用组合模式集成多个专业模块
   - 支持双模式处理（基础/扩展）
   - 提供完整的配置接口
   - 独立的变换点云功能

2. 算法特点：
   - 智能配准策略：PCA → FPFH → ICP
   - 多重质量检查和回退机制
   - 性能优化：预计算、对象复用

3. 工业应用优化：
   - 参数可调：适应不同物体和精度需求
   - 鲁棒性：完善的错误处理
   - 扩展性：模块化设计，易于添加新算法
   - 可视化支持：独立的变换点云生成功能

4. 变换点云功能特点：
   - 职责分离：不与测量数据混合
   - 按需生成：用户控制是否需要
   - 灵活保存：支持自定义路径和文件名
   - 完整的错误处理

5. 使用建议：
   - 开发阶段：使用基础模式快速迭代
   - 生产环境：使用扩展模式获得最佳精度
   - 调试分析：启用变换点云功能
   - 实时应用：启用下采样平衡速度和精度

6. 关键参数调优：
   - 体素大小：根据物体尺寸选择
   - ICP参数：根据精度要求调整
   - 模式选择：根据应用场景选择
*/