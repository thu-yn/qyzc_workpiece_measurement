#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <limits>

// Include extended functionality
#include "pose_measurement/point_cloud_processor.h"
#include "pose_measurement/measurement_calculator.h"

struct MeasurementData {
    // Pose information
    Eigen::Matrix4f transformation;
    Eigen::Vector3f translation;
    Eigen::Vector3f euler_angles;  // roll, pitch, yaw
    
    // Geometric measurements
    float length, width, height;
    float volume, surface_area;
    std::vector<float> key_angles;
    
    // Extended geometric features
    pose_measurement::GeometricFeatures geometric_features;
    
    // Quality metrics
    float registration_fitness;
    float registration_rmse;
};

class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator();
    
    // Main processing functions
    bool loadReferenceCloud(const std::string& pcd_path);
    MeasurementData processTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
    
    // Configuration
    void setVoxelSize(float size) { voxel_size_ = size; }
    void setICPMaxIterations(int iter) { icp_max_iterations_ = iter; }
    void setICPTransformationEpsilon(float eps) { icp_transformation_epsilon_ = eps; }
    void setEnableAdvancedMeasurements(bool enable) { enable_advanced_measurements_ = enable; }
    void setProcessingParameters(const pose_measurement::ProcessingParameters& params);
    
private:
    // Point cloud processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Registration methods
    Eigen::Matrix4f coarseRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    Eigen::Matrix4f fineRegistration(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target,
        const Eigen::Matrix4f& initial_guess);
    
    // PCA-based coarse alignment
    Eigen::Matrix4f pcaAlignment(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    // FPFH + RANSAC coarse alignment
    Eigen::Matrix4f fpfhAlignment(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    
    // Measurement calculation
    MeasurementData calculateMeasurements(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const Eigen::Matrix4f& transformation);
    
    // Utility functions
    Eigen::Vector3f rotationMatrixToEulerAngles(const Eigen::Matrix3f& R);
    void computePrincipalAxes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        Eigen::Matrix3f& eigenvectors,
        Eigen::Vector3f& eigenvalues);
    
    // Member variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_processed_;
    
    // Extended processing modules
    std::shared_ptr<pose_measurement::PointCloudProcessor> cloud_processor_;
    std::shared_ptr<pose_measurement::MeasurementCalculator> measurement_calculator_;
    bool enable_advanced_measurements_;
    
    // PCA data for reference cloud
    Eigen::Matrix3f ref_eigenvectors_;
    Eigen::Vector3f ref_eigenvalues_;
    Eigen::Vector3f ref_centroid_;
    
    // Parameters
    float voxel_size_;
    int icp_max_iterations_;
    float icp_transformation_epsilon_;
    float fpfh_radius_;
    
    // PCL objects
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;
};

#endif // POSE_ESTIMATOR_H