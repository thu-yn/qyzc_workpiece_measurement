/*
* 文件名: point_cloud_processor.h
* 文件作用: 点云预处理器头文件，定义了用于点云滤波、分割和聚类的高级处理流水线
* 
* 主要功能需求:
* 1. 点云去噪和异常值移除（统计学方法 + 半径方法）
* 2. 点云下采样（体素网格滤波）减少数据量
* 3. 平面检测和移除（去除背景平面如桌面、地面）
* 4. 欧几里得聚类分割（分离多个物体）
* 5. 目标物体提取（选择最大聚类作为目标）
* 6. 灵活的参数配置系统
* 
* 设计模式: 
* - 流水线模式：多个处理步骤串联
* - 策略模式：可配置的处理参数
* - 单一职责原则：专门负责点云预处理
* 
* 适用场景: 工业物体检测、机器人视觉、质量检测
* 依赖库: PCL点云库（滤波、分割、聚类模块）
*/

#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>                    // 体素网格下采样
#include <pcl/filters/statistical_outlier_removal.h>   // 统计异常值移除
#include <pcl/filters/radius_outlier_removal.h>        // 半径异常值移除
#include <pcl/filters/passthrough.h>                   // 直通滤波器
#include <pcl/segmentation/sac_segmentation.h>         // RANSAC分割
#include <pcl/segmentation/extract_clusters.h>         // 欧几里得聚类

namespace pose_measurement {

/**
* @brief 点云处理参数配置结构体
* 集中管理所有处理步骤的参数，便于调优和配置
*/
struct ProcessingParameters {
    // ===== 滤波参数 =====
    float voxel_size = 0.005f;                    // 体素大小（米），控制下采样精度
    int statistical_k = 50;                       // 统计滤波邻居点数量
    float statistical_stddev = 1.0f;              // 统计滤波标准差倍数
    float radius_outlier_radius = 0.01f;          // 半径滤波搜索半径（米）
    int radius_outlier_min_neighbors = 10;        // 半径内最少邻居点数量
    
    // ===== 分割参数 =====
    bool enable_plane_removal = true;             // 是否启用平面移除
    float plane_distance_threshold = 0.01f;       // 平面距离阈值（米）
    int plane_max_iterations = 1000;              // RANSAC最大迭代次数
    
    // ===== 聚类参数 =====
    bool enable_clustering = true;                // 是否启用聚类分割
    float cluster_tolerance = 0.02f;              // 聚类容差（米）
    int cluster_min_size = 100;                   // 最小聚类点数
    int cluster_max_size = 100000;                // 最大聚类点数
};

/**
* @brief 点云处理器类
* 提供完整的点云预处理流水线，从原始点云到清洁的目标物体点云
*/
class PointCloudProcessor {
public:
    /**
    * @brief 构造函数
    * 初始化所有PCL滤波器和分割器对象
    */
    PointCloudProcessor();
    
    /**
    * @brief 析构函数
    */
    ~PointCloudProcessor();
    
    // ===== 主要处理流水线 =====
    
    /**
    * @brief 完整点云处理流水线
    * @param input_cloud 输入的原始点云
    * @return 处理后的清洁点云
    * 
    * 处理流程：
    * 1. 去除异常值（统计学 + 半径方法）
    * 2. 体素网格下采样
    * 3. 移除平面（可选）
    * 4. 聚类分割并提取最大聚类（可选）
    * 
    * 设计思想：每个步骤都可以通过参数控制开启/关闭
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    
    // ===== 单独的处理步骤 =====
    
    /**
    * @brief 移除异常值
    * @param cloud 输入点云
    * @return 去噪后的点云
    * 
    * 两步去噪策略：
    * 1. 统计异常值移除：基于邻域点数的统计分析
    * 2. 半径异常值移除：基于半径内邻居点数量
    * 双重过滤确保更好的去噪效果
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
    * @brief 点云下采样
    * @param cloud 输入点云
    * @return 下采样后的点云
    * 
    * 使用体素网格方法：
    * - 将空间划分为规则体素网格
    * - 每个体素内的所有点用质心代替
    * - 既减少数据量又保持形状特征
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
    * @brief 移除平面
    * @param cloud 输入点云
    * @return 移除平面后的点云
    * 
    * 算法流程：
    * 1. 使用RANSAC算法检测最大平面
    * 2. 判断平面是否足够大（占点云10%以上）
    * 3. 移除平面点，保留物体点
    * 4. 迭代直到没有大平面或点数过少
    * 
    * 应用场景：去除桌面、地面等背景平面
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr removePlanes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
    * @brief 提取聚类
    * @param cloud 输入点云
    * @return 聚类数组，每个聚类是一个点云
    * 
    * 欧几里得聚类算法：
    * 1. 基于点间距离进行聚类
    * 2. 距离小于阈值的点归为同一聚类
    * 3. 过滤掉过小或过大的聚类
    * 
    * 用途：分离场景中的多个物体
    */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
    * @brief 获取最大聚类
    * @param cloud 输入点云
    * @return 最大聚类的点云
    * 
    * 假设：在工业检测场景中，最大的聚类通常就是目标物体
    * 这样可以自动过滤掉小的噪声聚类和背景物体
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLargestCluster(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // ===== 配置接口 =====
    
    /**
    * @brief 设置处理参数
    * @param params 参数结构体
    * 
    * 支持运行时动态调整所有处理参数
    */
    void setParameters(const ProcessingParameters& params) { params_ = params; }
    
    /**
    * @brief 获取当前参数
    * @return 当前的参数配置
    */
    ProcessingParameters getParameters() const { return params_; }
    
    // ===== 工具函数 =====
    
    /**
    * @brief 打印点云信息
    * @param cloud 要分析的点云
    * @param name 点云名称（用于日志）
    * 
    * 输出信息包括：
    * - 点云大小
    * - 边界框范围
    * - 质心位置
    * 用于调试和监控处理效果
    */
    static void printCloudInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name = "Cloud");
    
private:
    // ===== 私有成员变量 =====
    ProcessingParameters params_;  // 处理参数配置
    
    // ===== PCL滤波器对象 =====
    // 注意：这些对象在构造函数中初始化，可重复使用以提高性能
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;                          // 体素网格滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_filter_;    // 统计异常值滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter_;              // 半径异常值滤波器
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter_;                  // 直通滤波器（备用）
    
    // ===== PCL分割器对象 =====
    pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation_;              // 平面分割器
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction_;   // 聚类提取器
};

} // namespace pose_measurement

#endif // POINT_CLOUD_PROCESSOR_H