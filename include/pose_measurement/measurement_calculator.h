/*
* 文件名: measurement_calculator.h
* 文件作用: 几何测量计算器头文件，定义了用于计算3D点云几何特征的类和数据结构
* 
* 主要功能需求:
* 1. 计算点云的基本几何尺寸（长、宽、高）
* 2. 计算体积和表面积
* 3. 计算特征角度（主轴与坐标轴的夹角等）
* 4. 计算质心和主轴方向
* 5. 提供精确的几何测量算法，用于物体检测和质量评估
* 
* 设计模式: 单一职责原则 - 专门负责几何测量计算
* 依赖库: PCL点云库、Eigen数学库
*/

#ifndef MEASUREMENT_CALCULATOR_H
#define MEASUREMENT_CALCULATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace pose_measurement {

/**
* @brief 几何特征数据结构
* 存储点云的各种几何测量结果
*/
struct GeometricFeatures {
    float length, width, height;              // 长、宽、高（米）
    float volume;                             // 体积（立方米）
    float surface_area;                       // 表面积（平方米）
    std::vector<float> characteristic_angles; // 特征角度集合（弧度）
    Eigen::Vector3f centroid;                 // 质心坐标（米）
    Eigen::Matrix3f principal_axes;           // 主轴方向矩阵（3x3）
    Eigen::Vector3f principal_moments;        // 主轴对应的特征值
};

/**
* @brief 几何测量计算器类
* 负责计算3D点云的各种几何特征和测量参数
*/
class MeasurementCalculator {
public:
    /**
    * @brief 构造函数
    * 初始化默认参数
    */
    MeasurementCalculator();
    
    /**
    * @brief 析构函数
    */
    ~MeasurementCalculator();
    
    // ===== 主要测量功能 =====
    
    /**
    * @brief 计算点云的完整几何特征
    * @param cloud 输入点云
    * @return GeometricFeatures 包含所有几何测量结果的结构体
    * 
    * 功能说明：这是主要的接口函数，会调用下面所有的子函数
    * 来计算完整的几何特征集合
    */
    GeometricFeatures calculateGeometricFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // ===== 单独的测量功能 =====
    
    /**
    * @brief 计算有向包围盒的尺寸
    * @param cloud 输入点云
    * @param length 输出长度（最大尺寸）
    * @param width 输出宽度（中等尺寸）
    * @param height 输出高度（最小尺寸）
    * 
    * 算法说明：使用PCA主成分分析计算有向包围盒，
    * 然后按尺寸大小排序分配给长宽高
    */
    void calculateBoundingBoxDimensions(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        float& length, float& width, float& height);
    
    /**
    * @brief 计算点云体积
    * @param cloud 输入点云
    * @return float 体积值（立方米）
    * 
    * 算法说明：结合点密度估算和包围盒体积，
    * 取较小值作为更准确的体积估算
    */
    float calculateVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
    * @brief 计算点云表面积
    * @param cloud 输入点云
    * @return float 表面积值（平方米）
    * 
    * 算法说明：优先使用凸包算法计算精确表面积，
    * 失败时回退到包围盒表面积估算
    */
    float calculateSurfaceArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
    * @brief 计算特征角度
    * @param cloud 输入点云
    * @return std::vector<float> 角度数组（弧度）
    * 
    * 包含内容：
    * - 三个主轴与X/Y/Z轴的夹角（9个角度）
    * - 主轴之间的夹角（3个角度）
    * 总共12个特征角度
    */
    std::vector<float> calculateCharacteristicAngles(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // ===== 配置接口 =====
    
    /**
    * @brief 设置体素大小
    * @param size 体素大小（米），用于体积计算
    */
    void setVoxelSize(float size) { voxel_size_ = size; }
    
    /**
    * @brief 获取当前体素大小
    * @return float 体素大小（米）
    */
    float getVoxelSize() const { return voxel_size_; }
    
private:
    // ===== 私有成员变量 =====
    float voxel_size_;  // 体素大小，用于体积估算
    
    // ===== 私有辅助函数 =====
    
    /**
    * @brief 计算有向包围盒
    * @param cloud 输入点云
    * @param axes 输出主轴方向矩阵
    * @param dimensions 输出各轴向的尺寸
    * 
    * 算法说明：使用PCA计算主轴，然后在主轴坐标系下
    * 计算最小包围盒的尺寸
    */
    void computeOrientedBoundingBox(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        Eigen::Matrix3f& axes,
        Eigen::Vector3f& dimensions);
};

} // namespace pose_measurement

#endif // MEASUREMENT_CALCULATOR_H