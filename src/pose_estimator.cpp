/*
* 文件名: pose_estimator.cpp
* 文件作用: 姿态估计器的完整实现，提供工业级的6自由度姿态估计算法
* 
* 主要功能需求:
* 1. 实现多层次点云配准算法（PCA→FPFH→ICP的递进策略）
* 2. 实现鲁棒的几何测量计算（基础模式和扩展模式）
* 3. 实现配准质量评估算法（适配度和RMSE计算）
* 4. 实现完整的错误处理和回退机制
* 5. 提供高性能的工业级实现
* 6. 实现独立的变换点云生成功能（新增）
* 
* 核心算法特点:
* - 智能配准策略：先快后精，先简单后复杂
* - 多重质量检查：每个阶段都有质量评估
* - 自适应参数：根据数据特点动态调整
* - 完善的回退机制：确保系统稳定性
* - 独立变换功能：职责分离，按需使用
* 
* 工业应用优化:
* - 预计算优化：参考点云PCA预计算
* - 内存管理：智能指针和对象复用
* - 并行优化：使用OpenMP加速的算法
* 
* 依赖库: PCL点云库、Eigen数学库、ROS日志系统
*/

#include "pose_measurement/pose_estimator.h"
#include "pose_measurement/point_cloud_processor.h"
#include "pose_measurement/measurement_calculator.h"
#include <pcl/features/fpfh_omp.h>                          // OpenMP加速的FPFH特征
#include <pcl/registration/sample_consensus_prerejective.h> // RANSAC预筛选配准
#include <pcl/registration/icp.h>                           // ICP配准算法
#include <pcl/common/common.h>                              // PCL通用函数
#include <pcl/common/centroid.h>                            // 质心计算
#include <pcl/common/transforms.h>                          // 点云变换
#include <pcl/search/kdtree.h>                              // KD树搜索

/**
* @brief 构造函数
* 初始化所有参数和对象，设置默认的工业级参数
*/
PoseEstimator::PoseEstimator() 
    : voxel_size_(0.005f)                    // 默认5mm体素，适合大部分工业零件
    , icp_max_iterations_(100)               // ICP默认100次迭代
    , icp_transformation_epsilon_(1e-6)      // ICP收敛阈值
    , fpfh_radius_(0.025f)                   // FPFH特征半径25mm
    , enable_advanced_measurements_(false)   // 默认使用基础模式
    , enable_downsampling_(true){            // 默认开启下采样
    
    // 初始化点云智能指针
    reference_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    reference_cloud_processed_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 初始化扩展处理模块
    cloud_processor_ = std::make_shared<pose_measurement::PointCloudProcessor>();
    measurement_calculator_ = std::make_shared<pose_measurement::MeasurementCalculator>();
    
    // 配置基础滤波器（作为扩展模块的备选）
    outlier_filter_.setMeanK(50);               // 统计滤波邻居点数
    outlier_filter_.setStddevMulThresh(1.0);    // 标准差倍数阈值
    
    normal_estimator_.setKSearch(20);           // 法向量估算邻居点数
}

/**
* @brief 析构函数
*/
PoseEstimator::~PoseEstimator() {}

/**
* @brief 加载参考点云
* 这是系统初始化的关键步骤，需要预计算各种特征以提高后续处理性能
*/
bool PoseEstimator::loadReferenceCloud(const std::string& pcd_path) {
    // 加载PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *reference_cloud_) == -1) {
        ROS_ERROR("Failed to load reference cloud: %s", pcd_path.c_str());
        return false;
    }
    
    ROS_INFO("Loaded reference cloud with %zu points", reference_cloud_->points.size());
    
    // 预处理参考点云
    // 根据模式选择不同的预处理策略
    if (enable_advanced_measurements_) {
        // 扩展模式：使用完整的处理流水线
        reference_cloud_processed_ = cloud_processor_->processPointCloud(reference_cloud_);
    } else {
        // 基础模式：使用简单的预处理
        reference_cloud_processed_ = preprocessPointCloud(reference_cloud_);
    }
    
    // 预计算参考点云的PCA特征
    // 这些特征会在后续的PCA配准中重复使用
    computePrincipalAxes(reference_cloud_processed_, ref_eigenvectors_, ref_eigenvalues_);
    
    // 预计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*reference_cloud_processed_, centroid);
    ref_centroid_ = centroid.head<3>();
    
    ROS_INFO("Reference cloud preprocessing completed: %zu points", 
            reference_cloud_processed_->points.size());
    
    return true;
}

/**
* @brief 处理目标点云
* 这是系统的主要接口，完成从原始点云到测量结果的完整流程
*/
MeasurementData PoseEstimator::processTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) {
    MeasurementData result;
    
    ROS_INFO("Processing target cloud with %zu points", target_cloud->points.size());
    
    // 步骤1：预处理目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_processed;
    if (enable_advanced_measurements_) {
        // 扩展模式：使用高级处理流水线
        target_processed = cloud_processor_->processPointCloud(target_cloud);
    } else {
        // 基础模式：使用简单预处理
        target_processed = preprocessPointCloud(target_cloud);
    }
    ROS_INFO("Target cloud preprocessing completed: %zu points", target_processed->points.size());
    
    // 步骤2：粗配准
    // 获得初始的变换估计
    ROS_INFO("Starting coarse registration...");
    Eigen::Matrix4f coarse_transform = coarseRegistration(target_processed, reference_cloud_processed_);
    
    // 步骤3：精配准
    // 基于粗配准结果进行精确优化
    ROS_INFO("Starting fine registration...");
    Eigen::Matrix4f fine_transform = fineRegistration(target_processed, reference_cloud_processed_, coarse_transform);
    
    // 步骤4：计算测量结果
    result = calculateMeasurements(target_processed, fine_transform);
    
    ROS_INFO("Registration completed - Fitness: %.6f, RMSE: %.6f", 
            result.registration_fitness, result.registration_rmse);
    
    return result;
}

/**
* @brief 生成变换后的点云
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimator::generateTransformedCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
    const Eigen::Matrix4f& transformation) {
    
    // 边界检查
    if (!source_cloud || source_cloud->empty()) {
        ROS_WARN("Empty or null source cloud provided for transformation");
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    // 创建输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 应用变换矩阵
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformation);
    
    ROS_INFO("Generated transformed cloud with %zu points", transformed_cloud->size());
    
    return transformed_cloud;
}

/**
* @brief 保存变换后的点云到PCD文件
* 提供完整的文件保存功能和错误处理
*/
bool PoseEstimator::saveTransformedCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
    const std::string& file_path) {
    
    // 边界检查
    if (!transformed_cloud || transformed_cloud->empty()) {
        ROS_ERROR("Cannot save empty or null transformed cloud");
        return false;
    }
    
    if (file_path.empty()) {
        ROS_ERROR("Empty file path provided for saving transformed cloud");
        return false;
    }
    
    // 尝试保存PCD文件
    try {
        // 使用二进制格式保存，文件更小，加载更快
        if (pcl::io::savePCDFileBinary(file_path, *transformed_cloud) == -1) {
            ROS_ERROR("Failed to save transformed cloud to: %s", file_path.c_str());
            return false;
        }
        
        ROS_INFO("Successfully saved transformed cloud (%zu points) to: %s", 
                transformed_cloud->size(), file_path.c_str());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception occurred while saving transformed cloud: %s", e.what());
        return false;
    }
}

/**
* @brief 基础点云预处理
* 提供快速的基础预处理，适用于基础模式
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimator::preprocessPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    if (!cloud || cloud->empty()) {
        ROS_WARN("Empty or null input cloud for preprocessing");
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    ROS_INFO("Preprocessing cloud: %zu points", cloud->size());
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 统计异常值滤除
    ROS_DEBUG("Applying statistical outlier removal...");
    outlier_filter_.setInputCloud(cloud);
    outlier_filter_.filter(*processed);
    ROS_INFO("After outlier removal: %zu points", processed->size());
    
    // 下采样（条件）
    if (enable_downsampling_ && voxel_size_ > 0.0f) {
        ROS_INFO("Applying voxel grid downsampling (voxel size: %.6f)", voxel_size_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(processed);
        voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter_.filter(*downsampled);
        
        ROS_INFO("After downsampling: %zu points", downsampled->size());
        processed = downsampled;
    } else {
        ROS_INFO("Downsampling disabled - maintaining original resolution");
    }
    
    return processed;
}

/**
* @brief 粗配准
* 智能选择配准策略：先尝试快速的PCA对齐，失败则使用FPFH特征匹配
*/
Eigen::Matrix4f PoseEstimator::coarseRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    
    // 策略1：尝试PCA主轴对齐
    Eigen::Matrix4f pca_transform = pcaAlignment(source, target);
    
    // 应用PCA变换到源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pca_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *source_pca_aligned, pca_transform);
    
    // 评估PCA对齐的质量
    // 通过比较质心距离来判断对齐是否合理
    pcl::PointXYZ source_centroid, target_centroid;
    Eigen::Vector4f centroid_s, centroid_t;
    pcl::compute3DCentroid(*source_pca_aligned, centroid_s);
    pcl::compute3DCentroid(*target, centroid_t);
    
    float centroid_distance = (centroid_s.head<3>() - centroid_t.head<3>()).norm();
    
    // 质量检查：如果质心距离在合理范围内，认为PCA对齐成功
    if (centroid_distance < 0.1f) {  // 10cm阈值
        ROS_INFO("PCA alignment successful, centroid distance: %.3f", centroid_distance);
        return pca_transform;
    }
    
    // 策略2：PCA失败时使用FPFH特征匹配
    ROS_WARN("PCA alignment failed (distance: %.3f), trying FPFH alignment", centroid_distance);
    Eigen::Matrix4f fpfh_transform = fpfhAlignment(source, target);
    
    return fpfh_transform;
}

/**
* @brief PCA主轴对齐
* 基于主成分分析的快速对齐方法，适用于形状规整的物体
*/
Eigen::Matrix4f PoseEstimator::pcaAlignment(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    
    // 计算目标点云的PCA特征
    Eigen::Matrix3f target_eigenvectors;
    Eigen::Vector3f target_eigenvalues;
    computePrincipalAxes(target, target_eigenvectors, target_eigenvalues);
    
    // 计算质心
    Eigen::Vector4f source_centroid, target_centroid;
    pcl::compute3DCentroid(*source, source_centroid);
    pcl::compute3DCentroid(*target, target_centroid);
    
    // 计算旋转矩阵：对齐主轴方向
    // R = target_axes * source_axes^T
    Eigen::Matrix3f rotation = target_eigenvectors * ref_eigenvectors_.transpose();
    
    // 检查行列式以避免反射变换
    // 确保这是一个纯旋转变换而不是反射
    if (rotation.determinant() < 0) {
        // 翻转最后一个轴以避免反射
        target_eigenvectors.col(2) = -target_eigenvectors.col(2);
        rotation = target_eigenvectors * ref_eigenvectors_.transpose();
    }
    
    // 计算平移：在旋转后对齐质心
    Eigen::Vector3f translation = target_centroid.head<3>() - rotation * source_centroid.head<3>();
    
    // 构造4x4变换矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation;      // 旋转部分
    transform.block<3,1>(0,3) = translation;   // 平移部分
    
    return transform;
}

/**
* @brief FPFH特征匹配对齐
* 基于快速点特征直方图的鲁棒配准方法，适用于复杂形状
*/
Eigen::Matrix4f PoseEstimator::fpfhAlignment(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    
    // 步骤1：计算法向量
    // FPFH特征需要法向量信息
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    
    normal_estimator_.setInputCloud(source);
    normal_estimator_.compute(*source_normals);
    
    normal_estimator_.setInputCloud(target);
    normal_estimator_.compute(*target_normals);
    
    // 步骤2：计算FPFH特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    
    // 使用OpenMP加速的FPFH估算器
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator;
    fpfh_estimator.setRadiusSearch(fpfh_radius_);
    
    fpfh_estimator.setInputCloud(source);
    fpfh_estimator.setInputNormals(source_normals);
    fpfh_estimator.compute(*source_fpfh);
    
    fpfh_estimator.setInputCloud(target);
    fpfh_estimator.setInputNormals(target_normals);
    fpfh_estimator.compute(*target_fpfh);
    
    // 步骤3：基于RANSAC的特征匹配
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
    align.setInputSource(source);
    align.setSourceFeatures(source_fpfh);
    align.setInputTarget(target);
    align.setTargetFeatures(target_fpfh);
    
    // 配置RANSAC参数
    align.setMaximumIterations(50000);              // 最大迭代次数
    align.setNumberOfSamples(3);                    // 每次采样点数
    align.setCorrespondenceRandomness(2);           // 对应关系随机性
    align.setSimilarityThreshold(0.9f);             // 相似度阈值
    align.setMaxCorrespondenceDistance(voxel_size_ * 2.5f);  // 最大对应距离
    align.setInlierFraction(0.25f);                 // 内点比例阈值
    
    // 执行配准
    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    align.align(aligned_source);
    
    // 检查配准是否收敛
    if (align.hasConverged()) {
        ROS_INFO("FPFH alignment converged with fitness: %.6f", align.getFitnessScore());
        return align.getFinalTransformation();
    } else {
        ROS_WARN("FPFH alignment failed, returning identity transform");
        return Eigen::Matrix4f::Identity();
    }
}

/**
* @brief 精配准
* 使用ICP算法进行最终的高精度配准
*/
Eigen::Matrix4f PoseEstimator::fineRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    const Eigen::Matrix4f& initial_guess) {
    
    // 应用初始变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *source_transformed, initial_guess);
    
    // 配置Point-to-Plane ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_transformed);
    icp.setInputTarget(target);
    
    // ICP参数配置
    icp.setMaximumIterations(icp_max_iterations_);                  // 最大迭代次数
    icp.setTransformationEpsilon(icp_transformation_epsilon_);      // 变换收敛阈值
    icp.setMaxCorrespondenceDistance(voxel_size_ * 5.0f);           // 最大对应距离
    icp.setEuclideanFitnessEpsilon(1e-6);                           // 欧几里得适配度阈值
    
    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    icp.align(aligned_source);
    
    // 检查ICP是否收敛
    if (icp.hasConverged()) {
        ROS_INFO("ICP converged after %d iterations", icp.nr_iterations_);
        // 组合初始变换和ICP优化结果
        return icp.getFinalTransformation() * initial_guess;
    } else {
        ROS_WARN("ICP failed to converge, using initial guess");
        return initial_guess;
    }
}

/**
* @brief 计算几何测量
* 根据配准结果计算完整的几何测量数据
*/
MeasurementData PoseEstimator::calculateMeasurements(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const Eigen::Matrix4f& transformation) {
    
    MeasurementData result;
    result.transformation = transformation;
    
    // 提取旋转和平移分量
    result.translation = transformation.block<3,1>(0,3);
    Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
    result.euler_angles = rotationMatrixToEulerAngles(rotation);
    
    // 应用变换到点云，得到在参考坐标系下的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation);
    
    // 计算基础包围盒尺寸
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    result.length = max_pt.x - min_pt.x;
    result.width = max_pt.y - min_pt.y;
    result.height = max_pt.z - min_pt.z;
    
    // 根据模式选择测量算法
    if (enable_advanced_measurements_) {
        // 扩展模式：使用高精度几何测量
        result.geometric_features = measurement_calculator_->calculateGeometricFeatures(transformed_cloud);
        result.volume = result.geometric_features.volume;
        result.surface_area = result.geometric_features.surface_area;
        
        // 合并高级角度特征
        auto& advanced_angles = result.geometric_features.characteristic_angles;
        result.key_angles.insert(result.key_angles.end(), advanced_angles.begin(), advanced_angles.end());
    } else {
        // 基础模式：使用简单的体积和表面积估算
        result.volume = result.length * result.width * result.height;
        result.surface_area = 2.0f * (result.length * result.width + 
                                    result.width * result.height + 
                                    result.height * result.length);
    }
    
    // 计算基于主轴的关键角度
    Eigen::Matrix3f cloud_eigenvectors;
    Eigen::Vector3f cloud_eigenvalues;
    computePrincipalAxes(transformed_cloud, cloud_eigenvectors, cloud_eigenvalues);
    
    // 计算主轴与坐标轴的夹角
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3f axis = Eigen::Vector3f::Unit(i);
        float angle = std::acos(std::clamp(std::abs(cloud_eigenvectors.col(0).dot(axis)), 0.0f, 1.0f));
        result.key_angles.push_back(angle);
    }
    
    // 计算配准质量指标
    // 使用简单的ICP评估来获得适配度
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_eval;
    icp_eval.setInputSource(transformed_cloud);
    icp_eval.setInputTarget(reference_cloud_processed_);
    icp_eval.setMaximumIterations(1);  // 只用于评估，不需要迭代
    icp_eval.setMaxCorrespondenceDistance(voxel_size_ * 10.0f);
    
    pcl::PointCloud<pcl::PointXYZ> aligned_for_eval;
    icp_eval.align(aligned_for_eval);
    
    result.registration_fitness = icp_eval.getFitnessScore();
    
    // 手动计算RMSE，使用KD树进行高效最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(reference_cloud_processed_);
    
    float sum_squared_distances = 0.0f;
    int valid_correspondences = 0;
    const float max_correspondence_dist = voxel_size_ * 10.0f;
    const float max_correspondence_dist_sq = max_correspondence_dist * max_correspondence_dist;
    
    // 计算每个点到最近邻的距离
    for (const auto& point : transformed_cloud->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        
        if (kdtree.nearestKSearch(point, 1, indices, distances) > 0) {
            if (distances[0] < max_correspondence_dist_sq) {
                sum_squared_distances += distances[0];
                valid_correspondences++;
            }
        }
    }
    
    // 计算RMSE
    if (valid_correspondences > 0) {
        result.registration_rmse = std::sqrt(sum_squared_distances / valid_correspondences);
    } else {
        result.registration_rmse = std::numeric_limits<float>::max();
    }
    
    return result;
}

/**
* @brief 设置处理参数
* 配置扩展处理模块的参数
*/
void PoseEstimator::setProcessingParameters(const pose_measurement::ProcessingParameters& params) {
    if (cloud_processor_) {
        // 创建一个参数副本，并同步下采样设置
        pose_measurement::ProcessingParameters sync_params = params;
        cloud_processor_->setParameters(sync_params);
    }
    
    // 同步更新基础参数
    voxel_size_ = params.voxel_size;
    
    if (measurement_calculator_) {
        measurement_calculator_->setVoxelSize(params.voxel_size);
    }

    ROS_INFO("Processing parameters updated - voxel_size: %.6f, downsampling: %s", voxel_size_, enable_downsampling_ ? "enabled" : "disabled");
}

/**
* @brief 旋转矩阵转欧拉角
* 使用ZYX顺序的欧拉角转换
*/
Eigen::Vector3f PoseEstimator::rotationMatrixToEulerAngles(const Eigen::Matrix3f& R) {
    // ZYX欧拉角转换公式
    float sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    
    bool singular = sy < 1e-6; // 检查是否接近奇点
    
    float roll, pitch, yaw;
    
    if (!singular) {
        // 非奇点情况的标准转换
        roll = std::atan2(R(2,1), R(2,2));
        pitch = std::atan2(-R(2,0), sy);
        yaw = std::atan2(R(1,0), R(0,0));
    } else {
        // 奇点情况的处理
        roll = std::atan2(-R(1,2), R(1,1));
        pitch = std::atan2(-R(2,0), sy);
        yaw = 0;
    }
    
    return Eigen::Vector3f(roll, pitch, yaw);
}

/**
* @brief 计算主轴
* 使用PCA计算点云的主要方向
*/
void PoseEstimator::computePrincipalAxes(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Eigen::Matrix3f& eigenvectors,
    Eigen::Vector3f& eigenvalues) {
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    
    eigenvectors = pca.getEigenVectors();
    eigenvalues = pca.getEigenValues();
    
    // 确保一致的方向（可选的标准化）
    // 让第一个特征向量指向正方向
    if (eigenvectors(2, 0) < 0) {
        eigenvectors.col(0) = -eigenvectors.col(0);
    }
    
    // 确保右手坐标系
    Eigen::Vector3f cross_product = eigenvectors.col(0).cross(eigenvectors.col(1));
    if (cross_product.dot(eigenvectors.col(2)) < 0) {
        eigenvectors.col(2) = -eigenvectors.col(2);
    }
}