/*
* 文件名: pose_estimator.h
* 文件作用: 姿态估计器头文件，定义了基于点云配准的6自由度姿态估计系统
* 
* 主要功能需求:
* 1. 3D物体姿态估计（6自由度：3个平移 + 3个旋转）
* 2. 多层次配准算法（PCA粗对齐 + FPFH特征匹配 + ICP精配准）
* 3. 几何测量计算（长宽高、体积、表面积等）
* 4. 配准质量评估（适配度、RMSE等指标）
* 5. 扩展测量功能集成（高级几何特征计算）
* 
* 系统架构:
* - 分层配准策略：粗配准 → 精配准，确保鲁棒性和精度
* - 模块化设计：集成点云处理器和测量计算器
* - 灵活配置：支持基础模式和扩展模式
* 
* 适用场景: 工业质检、机器人抓取、物体识别、3D测量
* 核心算法: PCA对齐、FPFH+RANSAC、Point-to-Plane ICP
* 依赖库: PCL点云库、Eigen数学库、ROS
*/

#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>                              // ICP精配准算法
#include <pcl/registration/sample_consensus_prerejective.h>    // RANSAC预筛选配准
#include <pcl/features/fpfh.h>                                 // FPFH特征描述子
#include <pcl/features/normal_3d.h>                            // 法向量估算
#include <pcl/filters/voxel_grid.h>                            // 体素网格滤波
#include <pcl/filters/statistical_outlier_removal.h>           // 统计异常值移除
#include <pcl/common/pca.h>                                    // PCA主成分分析
#include <pcl/common/transforms.h>                             // 点云变换
#include <pcl/io/pcd_io.h>                                     // PCD文件IO
#include <pcl/search/kdtree.h>                                 // KD树搜索
#include <Eigen/Dense>                                         // Eigen数学库
#include <cmath>                                               // 数学函数
#include <algorithm>                                           // STL算法
#include <limits>                                              // 数值极限

// 包含扩展功能模块
#include "pose_measurement/point_cloud_processor.h"
#include "pose_measurement/measurement_calculator.h"

/**
* @brief 测量数据结构体
* 包含姿态估计和几何测量的完整结果
*/
struct MeasurementData {
    // ===== 姿态信息 =====
    Eigen::Matrix4f transformation;    // 4x4变换矩阵（包含旋转和平移）
    Eigen::Vector3f translation;       // 平移向量（x, y, z）单位：米
    Eigen::Vector3f euler_angles;      // 欧拉角（roll, pitch, yaw）单位：弧度
    
    // ===== 基础几何测量 =====
    float length, width, height;       // 长、宽、高（米）
    float volume, surface_area;        // 体积（立方米）、表面积（平方米）
    std::vector<float> key_angles;     // 关键角度集合（弧度）
    
    // ===== 扩展几何特征 =====
    pose_measurement::GeometricFeatures geometric_features;  // 详细几何特征
    
    // ===== 质量评估指标 =====
    float registration_fitness;        // 配准适配度（越小越好）
    float registration_rmse;           // 均方根误差（米）
};

/**
* @brief 姿态估计器类
* 核心功能：通过点云配准实现6自由度姿态估计和几何测量
*/
class PoseEstimator {
public:
    /**
    * @brief 构造函数
    * 初始化所有参数和PCL对象
    */
    PoseEstimator();
    
    /**
    * @brief 析构函数
    */
    ~PoseEstimator();
    
    // ===== 主要处理接口 =====
    
    /**
    * @brief 加载参考点云
    * @param pcd_path PCD文件路径
    * @return bool 加载是否成功
    * 
    * 功能说明：
    * 1. 加载标准姿态的参考点云
    * 2. 预处理参考点云（去噪、下采样等）
    * 3. 计算参考点云的PCA主轴和质心
    * 4. 为后续配准做准备
    */
    bool loadReferenceCloud(const std::string& pcd_path);
    
    /**
    * @brief 处理目标点云
    * @param target_cloud 待测量的目标点云
    * @return MeasurementData 完整的测量结果
    * 
    * 处理流程：
    * 1. 预处理目标点云
    * 2. 粗配准（PCA对齐或FPFH+RANSAC）
    * 3. 精配准（ICP优化）
    * 4. 计算几何测量
    * 5. 评估配准质量
    */
    MeasurementData processTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
    
    // ===== 配置接口 =====
    
    /**
    * @brief 设置体素大小
    * @param size 体素大小（米）
    * 影响下采样精度和配准速度
    */
    void setVoxelSize(float size) { voxel_size_ = size; }
    
    /**
    * @brief 设置ICP最大迭代次数
    * @param iter 迭代次数
    * 影响配准精度和计算时间
    */
    void setICPMaxIterations(int iter) { icp_max_iterations_ = iter; }
    
    /**
    * @brief 设置ICP收敛阈值
    * @param eps 收敛阈值
    * 控制ICP算法的收敛精度
    */
    void setICPTransformationEpsilon(float eps) { icp_transformation_epsilon_ = eps; }
    
    /**
    * @brief 启用/禁用高级测量功能
    * @param enable 是否启用
    * true: 使用高精度几何测量算法
    * false: 使用基础测量算法（更快）
    */
    void setEnableAdvancedMeasurements(bool enable) { enable_advanced_measurements_ = enable; }
    
    /**
    * @brief 设置处理参数
    * @param params 点云处理参数
    * 配置扩展处理模块的参数
    */
    void setProcessingParameters(const pose_measurement::ProcessingParameters& params);
    
private:
    // ===== 点云预处理 =====
    
    /**
    * @brief 基础点云预处理
    * @param cloud 输入点云
    * @return 预处理后的点云
    * 
    * 基础处理流程：
    * 1. 统计异常值移除
    * 2. 体素网格下采样
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // ===== 配准算法 =====
    
    /**
    * @brief 粗配准
    * @param source 源点云（目标点云）
    * @param target 目标点云（参考点云）
    * @return 4x4变换矩阵
    * 
    * 策略：
    * 1. 首先尝试PCA主轴对齐
    * 2. 失败则使用FPFH特征+RANSAC
    */
    Eigen::Matrix4f coarseRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    /**
    * @brief 精配准
    * @param source 源点云
    * @param target 目标点云
    * @param initial_guess 初始变换估计
    * @return 优化后的变换矩阵
    * 
    * 使用Point-to-Plane ICP算法进行精确配准
    */
    Eigen::Matrix4f fineRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target,
        const Eigen::Matrix4f& initial_guess);
    
    // ===== 具体配准方法 =====
    
    /**
    * @brief PCA主轴对齐
    * @param source 源点云
    * @param target 目标点云
    * @return 变换矩阵
    * 
    * 算法原理：
    * 1. 计算两个点云的主轴方向
    * 2. 对齐主轴方向
    * 3. 对齐质心位置
    * 适用于形状规整的物体
    */
    Eigen::Matrix4f pcaAlignment(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    /**
    * @brief FPFH特征匹配对齐
    * @param source 源点云
    * @param target 目标点云
    * @return 变换矩阵
    * 
    * 算法流程：
    * 1. 计算法向量
    * 2. 计算FPFH特征描述子
    * 3. RANSAC匹配寻找最佳变换
    * 适用于复杂形状的物体
    */
    Eigen::Matrix4f fpfhAlignment(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    // ===== 测量计算 =====
    
    /**
    * @brief 计算几何测量
    * @param cloud 配准后的点云
    * @param transformation 变换矩阵
    * @return 完整的测量数据
    * 
    * 根据enable_advanced_measurements_决定使用基础或高级算法
    */
    MeasurementData calculateMeasurements(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const Eigen::Matrix4f& transformation);
    
    // ===== 工具函数 =====
    
    /**
    * @brief 旋转矩阵转欧拉角
    * @param R 3x3旋转矩阵
    * @return 欧拉角（roll, pitch, yaw）
    * 使用ZYX顺序的欧拉角转换
    */
    Eigen::Vector3f rotationMatrixToEulerAngles(const Eigen::Matrix3f& R);
    
    /**
    * @brief 计算主轴
    * @param cloud 输入点云
    * @param eigenvectors 输出特征向量矩阵
    * @param eigenvalues 输出特征值向量
    * 
    * 使用PCA计算点云的主轴方向和对应的特征值
    */
    void computePrincipalAxes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        Eigen::Matrix3f& eigenvectors,
        Eigen::Vector3f& eigenvalues);
    
    // ===== 成员变量 =====
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;           // 原始参考点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_processed_; // 预处理后的参考点云
    
    // 扩展处理模块
    std::shared_ptr<pose_measurement::PointCloudProcessor> cloud_processor_;        // 点云处理器
    std::shared_ptr<pose_measurement::MeasurementCalculator> measurement_calculator_; // 测量计算器
    bool enable_advanced_measurements_;  // 是否启用高级测量
    
    // 参考点云的PCA数据（预计算以提高性能）
    Eigen::Matrix3f ref_eigenvectors_;   // 参考点云主轴方向
    Eigen::Vector3f ref_eigenvalues_;    // 参考点云特征值
    Eigen::Vector3f ref_centroid_;       // 参考点云质心
    
    // ===== 算法参数 =====
    float voxel_size_;                   // 体素大小（米）
    int icp_max_iterations_;             // ICP最大迭代次数
    float icp_transformation_epsilon_;   // ICP收敛阈值
    float fpfh_radius_;                  // FPFH特征半径
    
    // ===== PCL对象（预初始化以提高性能）=====
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;                               // 体素网格滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;             // 异常值移除器
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;       // 法向量估算器
};

#endif // POSE_ESTIMATOR_H