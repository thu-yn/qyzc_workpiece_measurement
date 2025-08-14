#include "pose_measurement/measurement_calculator.h"
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <algorithm>
#include <ros/ros.h>

namespace pose_measurement {

MeasurementCalculator::MeasurementCalculator() : voxel_size_(0.005f) {}

MeasurementCalculator::~MeasurementCalculator() {}

GeometricFeatures MeasurementCalculator::calculateGeometricFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    GeometricFeatures features;
    
    if (cloud->empty()) {
        ROS_WARN("Empty point cloud provided to MeasurementCalculator");
        return features;
    }
    
    // Calculate bounding box dimensions
    calculateBoundingBoxDimensions(cloud, features.length, features.width, features.height);
    
    // Calculate volume (approximation using voxel grid)
    features.volume = calculateVolume(cloud);
    
    // Calculate surface area (approximation)
    features.surface_area = calculateSurfaceArea(cloud);
    
    // Calculate characteristic angles
    features.characteristic_angles = calculateCharacteristicAngles(cloud);
    
    // Calculate centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    features.centroid = centroid.head<3>();
    
    // Calculate principal axes and moments
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    features.principal_axes = pca.getEigenVectors();
    features.principal_moments = pca.getEigenValues();
    
    return features;
}

void MeasurementCalculator::calculateBoundingBoxDimensions(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float& length, float& width, float& height) {
    
    if (cloud->empty()) {
        length = width = height = 0.0f;
        return;
    }
    
    // Compute oriented bounding box using PCA
    Eigen::Matrix3f axes;
    Eigen::Vector3f dimensions;
    computeOrientedBoundingBox(cloud, axes, dimensions);
    
    // Sort dimensions to assign length (max), width (mid), height (min)
    std::vector<float> sorted_dims = {dimensions[0], dimensions[1], dimensions[2]};
    std::sort(sorted_dims.rbegin(), sorted_dims.rend());
    
    length = sorted_dims[0];
    width = sorted_dims[1];
    height = sorted_dims[2];
}

float MeasurementCalculator::calculateVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->empty()) {
        return 0.0f;
    }
    
    // Simple volume estimation using point density
    // More sophisticated methods could use convex hull or voxel occupancy
    
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    float bounding_volume = (max_pt.x - min_pt.x) * 
                           (max_pt.y - min_pt.y) * 
                           (max_pt.z - min_pt.z);
    
    // Estimate fill ratio based on point density
    float point_volume = voxel_size_ * voxel_size_ * voxel_size_;
    float estimated_volume = cloud->size() * point_volume;
    
    // Return the smaller of the two estimates
    return std::min(bounding_volume, estimated_volume);
}

float MeasurementCalculator::calculateSurfaceArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->empty()) {
        return 0.0f;
    }
    
    try {
        // Use convex hull for surface area estimation
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(cloud);
        hull.setDimension(3);
        
        pcl::PointCloud<pcl::PointXYZ> hull_points;
        std::vector<pcl::Vertices> hull_polygons;
        hull.reconstruct(hull_points, hull_polygons);
        
        float surface_area = 0.0f;
        
        // Calculate area of each triangle in the hull
        for (const auto& polygon : hull_polygons) {
            if (polygon.vertices.size() >= 3) {
                // For triangular faces, calculate area using cross product
                const auto& p1 = hull_points.points[polygon.vertices[0]];
                const auto& p2 = hull_points.points[polygon.vertices[1]];
                const auto& p3 = hull_points.points[polygon.vertices[2]];
                
                Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
                Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
                
                float triangle_area = 0.5f * v1.cross(v2).norm();
                surface_area += triangle_area;
            }
        }
        
        return surface_area;
        
    } catch (const std::exception& e) {
        ROS_WARN("Failed to calculate surface area using convex hull: %s", e.what());
        
        // Fallback: estimate surface area from bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        
        float l = max_pt.x - min_pt.x;
        float w = max_pt.y - min_pt.y;
        float h = max_pt.z - min_pt.z;
        
        return 2.0f * (l*w + w*h + h*l);
    }
}

std::vector<float> MeasurementCalculator::calculateCharacteristicAngles(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    std::vector<float> angles;
    
    if (cloud->empty()) {
        return angles;
    }
    
    // Calculate principal axes
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    
    // Calculate angles between principal axes and coordinate axes
    Eigen::Vector3f x_axis(1, 0, 0);
    Eigen::Vector3f y_axis(0, 1, 0);
    Eigen::Vector3f z_axis(0, 0, 1);
    
    // Angle between first principal axis and X, Y, Z axes
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3f principal_axis = eigenvectors.col(i);
        
        float angle_x = std::acos(std::clamp(std::abs(principal_axis.dot(x_axis)), 0.0f, 1.0f));
        float angle_y = std::acos(std::clamp(std::abs(principal_axis.dot(y_axis)), 0.0f, 1.0f));
        float angle_z = std::acos(std::clamp(std::abs(principal_axis.dot(z_axis)), 0.0f, 1.0f));
        
        angles.push_back(angle_x);
        angles.push_back(angle_y);
        angles.push_back(angle_z);
    }
    
    // Additional characteristic angles could be added here
    // For example: angles between different principal axes
    float angle_12 = std::acos(std::clamp(std::abs(eigenvectors.col(0).dot(eigenvectors.col(1))), 0.0f, 1.0f));
    float angle_13 = std::acos(std::clamp(std::abs(eigenvectors.col(0).dot(eigenvectors.col(2))), 0.0f, 1.0f));
    float angle_23 = std::acos(std::clamp(std::abs(eigenvectors.col(1).dot(eigenvectors.col(2))), 0.0f, 1.0f));
    
    angles.push_back(angle_12);
    angles.push_back(angle_13);
    angles.push_back(angle_23);
    
    return angles;
}

void MeasurementCalculator::computeOrientedBoundingBox(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Eigen::Matrix3f& axes,
    Eigen::Vector3f& dimensions) {
    
    if (cloud->empty()) {
        axes = Eigen::Matrix3f::Identity();
        dimensions = Eigen::Vector3f::Zero();
        return;
    }
    
    // Compute PCA to get principal axes
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    
    axes = pca.getEigenVectors();
    Eigen::Vector3f centroid = pca.getMean().head<3>();
    
    // Transform points to principal coordinate system
    std::vector<Eigen::Vector3f> transformed_points;
    transformed_points.reserve(cloud->size());
    
    for (const auto& point : cloud->points) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f transformed = axes.transpose() * (p - centroid);
        transformed_points.push_back(transformed);
    }
    
    // Find min/max in each principal direction
    Eigen::Vector3f min_vals = transformed_points[0];
    Eigen::Vector3f max_vals = transformed_points[0];
    
    for (const auto& point : transformed_points) {
        for (int i = 0; i < 3; ++i) {
            min_vals[i] = std::min(min_vals[i], point[i]);
            max_vals[i] = std::max(max_vals[i], point[i]);
        }
    }
    
    dimensions = max_vals - min_vals;
}

} // namespace pose_measurement