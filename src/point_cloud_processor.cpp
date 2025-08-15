/*
* 文件名: point_cloud_processor.cpp
* 文件作用: 点云预处理器的具体实现，提供鲁棒的工业级点云处理算法
* 
* 主要功能需求:
* 1. 实现多层次的点云去噪算法（统计学 + 半径双重过滤）
* 2. 实现高效的点云下采样算法（体素网格方法）
* 3. 实现智能的平面检测和移除算法（RANSAC + 迭代策略）
* 4. 实现欧几里得聚类算法（基于距离的物体分离）
* 5. 提供完整的错误处理和性能监控
* 
* 算法特点:
* - 采用流水线处理模式，每步都有质量控制
* - 智能的阈值判断，避免过度处理
* - 详细的日志记录，便于调试和优化
* - 鲁棒的异常处理，确保系统稳定性
* 
* 适用场景: 工业质检、机器人抓取、3D重建预处理
* 依赖库: PCL点云库、ROS日志系统
*/

#include "pose_measurement/point_cloud_processor.h"
#include <pcl/filters/extract_indices.h>               // 索引提取器
#include <pcl/ModelCoefficients.h>                     // 模型系数（平面方程等）
#include <pcl/sample_consensus/method_types.h>         // RANSAC方法类型
#include <pcl/sample_consensus/model_types.h>          // 模型类型（平面、球面等）
#include <pcl/common/common.h>                         // PCL通用函数
#include <pcl/common/centroid.h>                       // 质心计算
#include <ros/ros.h>                                   // ROS日志系统

namespace pose_measurement {

/**
* @brief 构造函数
* 初始化所有PCL对象，设置默认参数
* 预先配置可以避免运行时的重复初始化，提高性能
*/
PointCloudProcessor::PointCloudProcessor() {
    // 初始化平面分割器参数
    plane_segmentation_.setOptimizeCoefficients(true);        // 优化平面系数
    plane_segmentation_.setModelType(pcl::SACMODEL_PLANE);    // 设置模型类型为平面
    plane_segmentation_.setMethodType(pcl::SAC_RANSAC);       // 使用RANSAC算法
    
    // 初始化聚类提取器
    // 使用KD树作为搜索结构，提高邻近点查找效率
    cluster_extraction_.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
}

/**
* @brief 析构函数
*/
PointCloudProcessor::~PointCloudProcessor() {}

/**
* @brief 完整的点云处理流水线
* 这是主要的对外接口，集成了所有处理步骤
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::processPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    
    // 边界检查：处理空指针或空点云
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN("Empty or null input cloud provided to PointCloudProcessor");
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    // 记录输入状态
    printCloudInfo(input_cloud, "Input");
    
    // 步骤1：移除异常值
    // 这是第一步，因为异常值会影响后续所有处理
    auto cloud_filtered = removeOutliers(input_cloud);
    printCloudInfo(cloud_filtered, "After outlier removal");
    
    // 步骤2：下采样
    // 减少数据量，提高后续处理速度
    auto cloud_downsampled = downsample(cloud_filtered);
    if (params_.enable_downsampling) {
        printCloudInfo(cloud_downsampled, "After downsampling");
    } else {
        printCloudInfo(cloud_downsampled, "Downsampling skipped");
    }
    
    // 步骤3：移除平面（可选）
    // 根据参数决定是否移除背景平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_planes;
    if (params_.enable_plane_removal) {
        cloud_no_planes = removePlanes(cloud_downsampled);
        printCloudInfo(cloud_no_planes, "After plane removal");
    } else {
        cloud_no_planes = cloud_downsampled;
    }
    
    // 步骤4：聚类分割并提取最大聚类（可选）
    // 分离多个物体并选择目标物体
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud;
    if (params_.enable_clustering) {
        final_cloud = getLargestCluster(cloud_no_planes);
        printCloudInfo(final_cloud, "Final (largest cluster)");
    } else {
        final_cloud = cloud_no_planes;
        printCloudInfo(final_cloud, "Final");
    }
    
    return final_cloud;
}

/**
* @brief 移除异常值
* 采用双重过滤策略确保去噪效果
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::removeOutliers(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 第一层过滤：统计异常值移除
    // 原理：计算每个点到其邻居点的平均距离，移除偏离统计分布的点
    statistical_filter_.setInputCloud(cloud);
    statistical_filter_.setMeanK(params_.statistical_k);                    // 邻居点数量
    statistical_filter_.setStddevMulThresh(params_.statistical_stddev);     // 标准差倍数阈值
    statistical_filter_.filter(*cloud_filtered);
    
    // 第二层过滤：半径异常值移除
    // 原理：在指定半径内，邻居点数量少于阈值的点被视为异常值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    radius_filter_.setInputCloud(cloud_filtered);
    radius_filter_.setRadiusSearch(params_.radius_outlier_radius);          // 搜索半径
    radius_filter_.setMinNeighborsInRadius(params_.radius_outlier_min_neighbors);  // 最少邻居数
    radius_filter_.filter(*cloud_radius_filtered);
    
    return cloud_radius_filtered;
}

/**
* @brief 点云下采样
* 使用体素网格方法减少点云密度
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::downsample(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 如果禁用下采样，直接返回原点云
    if (!params_.enable_downsampling) {
        ROS_INFO("Downsampling disabled - returning original cloud");
        return cloud;
    }
    
    // 检查体素大小
    if (params_.voxel_size <= 0.0f) {
        ROS_WARN("Invalid voxel size (%.6f), skipping downsampling", params_.voxel_size);
        return cloud;
    }

    // 体素网格下采样
    // 原理：将3D空间划分为立方体网格，每个网格内的点用质心代替
    // 优点：既减少数据量又保持几何特征
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
    voxel_filter_.filter(*cloud_filtered);

    ROS_INFO("Downsampling applied: %zu -> %zu points (voxel size: %.6f)", 
             cloud->size(), cloud_filtered->size(), params_.voxel_size);
    
    return cloud_filtered;
}

/**
* @brief 移除平面
* 使用RANSAC算法迭代检测和移除平面
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::removePlanes(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    // 创建工作副本，避免修改输入数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_planes(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_no_planes);
    
    // 准备分割所需的数据结构
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);   // 平面方程系数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                  // 内点索引
    pcl::ExtractIndices<pcl::PointXYZ> extract;                             // 索引提取器
    
    int original_size = cloud_no_planes->size();
    
    // 迭代移除平面，直到满足停止条件
    while (cloud_no_planes->size() > 0.1 * original_size) {  // 保留至少10%的原始点
        // 使用RANSAC分割最大平面
        plane_segmentation_.setInputCloud(cloud_no_planes);
        plane_segmentation_.setDistanceThreshold(params_.plane_distance_threshold);  // 点到平面距离阈值
        plane_segmentation_.setMaxIterations(params_.plane_max_iterations);          // 最大迭代次数
        plane_segmentation_.segment(*inliers, *coefficients);
        
        // 检查是否找到有效平面
        if (inliers->indices.size() == 0) {
            ROS_DEBUG("Could not estimate a planar model for the given dataset.");
            break;
        }
        
        // 检查平面是否足够大（避免移除小的平面特征）
        float plane_ratio = static_cast<float>(inliers->indices.size()) / cloud_no_planes->size();
        if (plane_ratio < 0.1) {  // 平面必须包含至少10%的点才被认为是显著的
            ROS_DEBUG("Plane too small (%.1f%%), stopping plane removal", plane_ratio * 100);
            break;
        }
        
        // 移除平面内点，保留其余点
        extract.setInputCloud(cloud_no_planes);
        extract.setIndices(inliers);
        extract.setNegative(true);  // true表示移除内点，保留外点
        extract.filter(*cloud_no_planes);
        
        ROS_DEBUG("Removed plane with %zu points (%.1f%%)", 
                inliers->indices.size(), plane_ratio * 100);
    }
    
    return cloud_no_planes;
}

/**
* @brief 提取聚类
* 使用欧几里得聚类算法分离不同物体
*/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudProcessor::extractClusters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    
    if (cloud->empty()) {
        return clusters;
    }
    
    // 配置聚类提取器
    cluster_extraction_.setInputCloud(cloud);
    cluster_extraction_.setClusterTolerance(params_.cluster_tolerance);    // 聚类容差（点间最大距离）
    cluster_extraction_.setMinClusterSize(params_.cluster_min_size);       // 最小聚类大小
    cluster_extraction_.setMaxClusterSize(params_.cluster_max_size);       // 最大聚类大小
    
    // 执行聚类
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extraction_.extract(cluster_indices);
    
    // 将每个聚类的索引转换为独立的点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 根据索引提取聚类点
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(indices));
        extract.setIndices(cluster_indices_ptr);
        extract.setNegative(false);  // false表示提取内点
        extract.filter(*cluster);
        
        clusters.push_back(cluster);
    }
    
    ROS_DEBUG("Extracted %zu clusters", clusters.size());
    
    return clusters;
}

/**
* @brief 获取最大聚类
* 假设最大的聚类就是目标物体
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::getLargestCluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    auto clusters = extractClusters(cloud);
    
    if (clusters.empty()) {
        ROS_WARN("No clusters found, returning original cloud");
        return cloud;
    }
    
    // 找到点数最多的聚类
    auto largest_cluster = clusters[0];
    for (const auto& cluster : clusters) {
        if (cluster->size() > largest_cluster->size()) {
            largest_cluster = cluster;
        }
    }
    
    ROS_DEBUG("Selected largest cluster with %zu points out of %zu clusters", 
            largest_cluster->size(), clusters.size());
    
    return largest_cluster;
}

/**
* @brief 打印点云信息
* 用于调试和监控处理效果的工具函数
*/
void PointCloudProcessor::printCloudInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name) {
    // 处理空指针
    if (!cloud) {
        ROS_INFO("%s: NULL cloud", name.c_str());
        return;
    }
    
    // 处理空点云
    if (cloud->empty()) {
        ROS_INFO("%s: Empty cloud", name.c_str());
        return;
    }
    
    // 计算边界框
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    // 计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    
    // 输出详细信息
    ROS_INFO("%s: %zu points, Bounds: [%.3f,%.3f] [%.3f,%.3f] [%.3f,%.3f], Centroid: [%.3f,%.3f,%.3f]",
            name.c_str(), cloud->size(),
            min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z,  // 边界框范围
            centroid[0], centroid[1], centroid[2]);                       // 质心坐标
}

} // namespace pose_measurement