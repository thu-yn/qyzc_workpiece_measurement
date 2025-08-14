/*
 * 文件名: measurement_calculator.cpp
 * 文件作用: 几何测量计算器的具体实现，提供精确的3D几何测量算法
 * 
 * 主要功能需求:
 * 1. 实现基于PCA的有向包围盒计算算法
 * 2. 实现多种体积计算方法（点密度法 + 包围盒法）
 * 3. 实现基于凸包的精确表面积计算
 * 4. 实现特征角度计算（主轴与坐标轴夹角）
 * 5. 提供鲁棒的错误处理和边界情况处理
 * 
 * 算法特点:
 * - 使用PCA主成分分析获得物体主轴方向
 * - 结合多种算法提高测量精度和鲁棒性
 * - 自动排序尺寸为长宽高（大→中→小）
 * - 使用凸包算法计算精确表面积
 * 
 * 依赖库: PCL点云库（PCA、凸包、质心计算等模块）
*/

#include "pose_measurement/measurement_calculator.h"
#include <pcl/common/common.h>         // PCL通用函数（最小最大值等）
#include <pcl/common/pca.h>            // PCA主成分分析
#include <pcl/common/centroid.h>       // 质心计算
#include <pcl/surface/convex_hull.h>   // 凸包计算
#include <pcl/surface/concave_hull.h>  // 凹包计算（备用）
#include <algorithm>                   // STL算法库
#include <ros/ros.h>                   // ROS日志系统

namespace pose_measurement {

/**
* @brief 构造函数
* 初始化默认体素大小为5mm，适合大多数工业零件测量
*/
MeasurementCalculator::MeasurementCalculator() : voxel_size_(0.005f) {}

/**
* @brief 析构函数
*/
MeasurementCalculator::~MeasurementCalculator() {}

/**
* @brief 计算点云的完整几何特征
* 这是主要的对外接口，集成了所有测量功能
*/
GeometricFeatures MeasurementCalculator::calculateGeometricFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    GeometricFeatures features;
    
    // 边界检查：处理空点云情况
    if (cloud->empty()) {
        ROS_WARN("Empty point cloud provided to MeasurementCalculator");
        return features;  // 返回默认初始化的结构体（全零值）
    }
    
    // 步骤1：计算包围盒尺寸（长宽高）
    calculateBoundingBoxDimensions(cloud, features.length, features.width, features.height);
    
    // 步骤2：计算体积（使用体素网格近似）
    features.volume = calculateVolume(cloud);
    
    // 步骤3：计算表面积（使用凸包算法）
    features.surface_area = calculateSurfaceArea(cloud);
    
    // 步骤4：计算特征角度（主轴与坐标轴的夹角）
    features.characteristic_angles = calculateCharacteristicAngles(cloud);
    
    // 步骤5：计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    features.centroid = centroid.head<3>();  // 取前3个分量（x,y,z）
    
    // 步骤6：计算主轴和主要矩
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    features.principal_axes = pca.getEigenVectors();    // 主轴方向矩阵
    features.principal_moments = pca.getEigenValues();  // 对应的特征值
    
    return features;
}

/**
* @brief 计算有向包围盒的尺寸
* 使用PCA主成分分析计算物体的主要方向，然后计算沿主轴方向的尺寸
*/
void MeasurementCalculator::calculateBoundingBoxDimensions(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float& length, float& width, float& height) {
    
    // 边界检查
    if (cloud->empty()) {
        length = width = height = 0.0f;
        return;
    }
    
    // 计算有向包围盒（OBB: Oriented Bounding Box）
    Eigen::Matrix3f axes;      // 主轴方向矩阵
    Eigen::Vector3f dimensions; // 各轴向尺寸
    computeOrientedBoundingBox(cloud, axes, dimensions);
    
    // 按尺寸大小排序：length >= width >= height
    // TODO：有些工件并非尺寸由大到小排序为长宽高，应该改为由重力方向判断？
    std::vector<float> sorted_dims = {dimensions[0], dimensions[1], dimensions[2]};
    std::sort(sorted_dims.rbegin(), sorted_dims.rend());  // 降序排列
    
    length = sorted_dims[0];   // 最大尺寸
    width = sorted_dims[1];    // 中等尺寸
    height = sorted_dims[2];   // 最小尺寸
}

/**
* @brief 计算点云体积
* 使用两种方法估算体积，取较小值作为更准确的估计
*/
float MeasurementCalculator::calculateVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->empty()) {
        return 0.0f;
    }
    
    // 方法1：基于轴对齐包围盒的体积估算
    // 注意：这通常会高估体积，因为包围盒包含了物体周围的空白空间
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    float bounding_volume = (max_pt.x - min_pt.x) * 
                        (max_pt.y - min_pt.y) * 
                        (max_pt.z - min_pt.z);
    
    // 方法2：基于点密度的体积估算
    // 假设每个点代表一个体素的体积
    float point_volume = voxel_size_ * voxel_size_ * voxel_size_;
    float estimated_volume = cloud->size() * point_volume;
    
    // 取两种方法的较小值，通常更接近真实体积
    // 包围盒法倾向于高估，点密度法相对更准确
    return std::min(bounding_volume, estimated_volume);
}

/**
* @brief 计算点云表面积
* 优先使用凸包算法计算精确表面积，失败时使用包围盒近似
*/
float MeasurementCalculator::calculateSurfaceArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->empty()) {
        return 0.0f;
    }
    
    try {
        // 方法1：使用凸包算法计算精确表面积
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(cloud);
        hull.setDimension(3);  // 3D凸包
        
        pcl::PointCloud<pcl::PointXYZ> hull_points;       // 凸包顶点
        std::vector<pcl::Vertices> hull_polygons;          // 凸包三角面片
        hull.reconstruct(hull_points, hull_polygons);
        
        float surface_area = 0.0f;
        
        // 计算每个三角面片的面积并累加
        for (const auto& polygon : hull_polygons) {
            if (polygon.vertices.size() >= 3) {
                // 对于三角面片，使用叉积计算面积
                const auto& p1 = hull_points.points[polygon.vertices[0]];
                const auto& p2 = hull_points.points[polygon.vertices[1]];
                const auto& p3 = hull_points.points[polygon.vertices[2]];
                
                // 计算两个边向量
                Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
                Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
                
                // 三角形面积 = |v1 × v2| / 2
                float triangle_area = 0.5f * v1.cross(v2).norm();
                surface_area += triangle_area;
            }
        }
        
        return surface_area;
        
    } catch (const std::exception& e) {
        // 凸包计算失败时的处理
        ROS_WARN("Failed to calculate surface area using convex hull: %s", e.what());
        
        // 回退方案：使用轴对齐包围盒估算表面积
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        
        float l = max_pt.x - min_pt.x;  // 长度
        float w = max_pt.y - min_pt.y;  // 宽度
        float h = max_pt.z - min_pt.z;  // 高度
        
        // 长方体表面积公式：2(lw + wh + hl)
        return 2.0f * (l*w + w*h + h*l);
    }
}

/**
* @brief 计算特征角度
* 计算物体主轴与坐标轴的夹角，用于姿态分析
*/
std::vector<float> MeasurementCalculator::calculateCharacteristicAngles(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    std::vector<float> angles;
    
    if (cloud->empty()) {
        return angles;
    }
    
    // 计算主轴方向
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    
    // 定义坐标轴单位向量
    Eigen::Vector3f x_axis(1, 0, 0);
    Eigen::Vector3f y_axis(0, 1, 0);
    Eigen::Vector3f z_axis(0, 0, 1);
    
    // 计算每个主轴与坐标轴的夹角
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3f principal_axis = eigenvectors.col(i);
        
        // 计算主轴与X、Y、Z轴的夹角
        // 使用点积公式：cos(θ) = |a·b| / (|a||b|)
        // 这里取绝对值是因为我们关心的是角度大小，不关心方向
        float angle_x = std::acos(std::clamp(std::abs(principal_axis.dot(x_axis)), 0.0f, 1.0f));
        float angle_y = std::acos(std::clamp(std::abs(principal_axis.dot(y_axis)), 0.0f, 1.0f));
        float angle_z = std::acos(std::clamp(std::abs(principal_axis.dot(z_axis)), 0.0f, 1.0f));
        
        // 将角度添加到结果数组
        angles.push_back(angle_x);
        angles.push_back(angle_y);
        angles.push_back(angle_z);
    }
    
    // 添加主轴之间的夹角
    // 这些角度提供了物体形状的额外信息
    float angle_12 = std::acos(std::clamp(std::abs(eigenvectors.col(0).dot(eigenvectors.col(1))), 0.0f, 1.0f));
    float angle_13 = std::acos(std::clamp(std::abs(eigenvectors.col(0).dot(eigenvectors.col(2))), 0.0f, 1.0f));
    float angle_23 = std::acos(std::clamp(std::abs(eigenvectors.col(1).dot(eigenvectors.col(2))), 0.0f, 1.0f));
    
    angles.push_back(angle_12);
    angles.push_back(angle_13);
    angles.push_back(angle_23);
    
    return angles;
}

/**
* @brief 计算有向包围盒
* 这是一个核心算法，用于计算物体在其主轴方向上的精确尺寸
*/
void MeasurementCalculator::computeOrientedBoundingBox(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Eigen::Matrix3f& axes,
    Eigen::Vector3f& dimensions) {
    
    // 边界检查
    if (cloud->empty()) {
        axes = Eigen::Matrix3f::Identity();      // 单位矩阵
        dimensions = Eigen::Vector3f::Zero();    // 零向量
        return;
    }
    
    // 步骤1：计算PCA得到主轴方向
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    
    axes = pca.getEigenVectors();              // 主轴方向矩阵
    Eigen::Vector3f centroid = pca.getMean().head<3>();  // 质心位置
    
    // 步骤2：将所有点变换到主轴坐标系
    // 在主轴坐标系中，物体的形状更规整，便于计算包围盒
    std::vector<Eigen::Vector3f> transformed_points;
    transformed_points.reserve(cloud->size());
    
    for (const auto& point : cloud->points) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        // 变换公式：p' = R^T * (p - centroid)
        // 其中R是主轴矩阵，R^T是其转置
        Eigen::Vector3f transformed = axes.transpose() * (p - centroid);
        transformed_points.push_back(transformed);
    }
    
    // 步骤3：在主轴坐标系中找到最小最大值
    // 这就得到了有向包围盒的尺寸
    Eigen::Vector3f min_vals = transformed_points[0];
    Eigen::Vector3f max_vals = transformed_points[0];
    
    for (const auto& point : transformed_points) {
        for (int i = 0; i < 3; ++i) {
            min_vals[i] = std::min(min_vals[i], point[i]);
            max_vals[i] = std::max(max_vals[i], point[i]);
        }
    }
    
    // 计算各轴向的尺寸
    dimensions = max_vals - min_vals;
}

} // namespace pose_measurement