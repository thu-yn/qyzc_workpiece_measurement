#ifndef MEASUREMENT_CALCULATOR_H
#define MEASUREMENT_CALCULATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace pose_measurement {

struct GeometricFeatures {
    float length, width, height;
    float volume;
    float surface_area;
    std::vector<float> characteristic_angles;
    Eigen::Vector3f centroid;
    Eigen::Matrix3f principal_axes;
    Eigen::Vector3f principal_moments;
};

class MeasurementCalculator {
public:
    MeasurementCalculator();
    ~MeasurementCalculator();
    
    // Main measurement functions
    GeometricFeatures calculateGeometricFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Individual measurement functions
    void calculateBoundingBoxDimensions(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        float& length, float& width, float& height);
    
    float calculateVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float calculateSurfaceArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    std::vector<float> calculateCharacteristicAngles(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Configuration
    void setVoxelSize(float size) { voxel_size_ = size; }
    float getVoxelSize() const { return voxel_size_; }
    
private:
    float voxel_size_;
    
    // Helper functions
    void computeOrientedBoundingBox(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        Eigen::Matrix3f& axes,
        Eigen::Vector3f& dimensions);
};

} // namespace pose_measurement

#endif // MEASUREMENT_CALCULATOR_H