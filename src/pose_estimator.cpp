#include "pose_measurement/pose_estimator.h"
#include "pose_measurement/point_cloud_processor.h"
#include "pose_measurement/measurement_calculator.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

PoseEstimator::PoseEstimator() 
    : voxel_size_(0.005f)
    , icp_max_iterations_(100)
    , icp_transformation_epsilon_(1e-6)
    , fpfh_radius_(0.025f)
    , enable_advanced_measurements_(false) {
    
    reference_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    reference_cloud_processed_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Initialize extended processing modules
    cloud_processor_ = std::make_shared<pose_measurement::PointCloudProcessor>();
    measurement_calculator_ = std::make_shared<pose_measurement::MeasurementCalculator>();
    
    // Configure filters (fallback for basic processing)
    outlier_filter_.setMeanK(50);
    outlier_filter_.setStddevMulThresh(1.0);
    
    normal_estimator_.setKSearch(20);
}

PoseEstimator::~PoseEstimator() {}

bool PoseEstimator::loadReferenceCloud(const std::string& pcd_path) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *reference_cloud_) == -1) {
        ROS_ERROR("Failed to load reference cloud: %s", pcd_path.c_str());
        return false;
    }
    
    ROS_INFO("Loaded reference cloud with %zu points", reference_cloud_->points.size());
    
    // Preprocess reference cloud
    if (enable_advanced_measurements_) {
        reference_cloud_processed_ = cloud_processor_->processPointCloud(reference_cloud_);
    } else {
        reference_cloud_processed_ = preprocessPointCloud(reference_cloud_);
    }
    
    // Compute PCA for reference cloud
    computePrincipalAxes(reference_cloud_processed_, ref_eigenvectors_, ref_eigenvalues_);
    
    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*reference_cloud_processed_, centroid);
    ref_centroid_ = centroid.head<3>();
    
    ROS_INFO("Reference cloud preprocessing completed: %zu points", 
             reference_cloud_processed_->points.size());
    
    return true;
}

MeasurementData PoseEstimator::processTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) {
    MeasurementData result;
    
    ROS_INFO("Processing target cloud with %zu points", target_cloud->points.size());
    
    // Preprocess target cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_processed;
    if (enable_advanced_measurements_) {
        target_processed = cloud_processor_->processPointCloud(target_cloud);
    } else {
        target_processed = preprocessPointCloud(target_cloud);
    }
    ROS_INFO("Target cloud preprocessing completed: %zu points", target_processed->points.size());
    
    // Coarse registration
    ROS_INFO("Starting coarse registration...");
    Eigen::Matrix4f coarse_transform = coarseRegistration(target_processed, reference_cloud_processed_);
    
    // Fine registration
    ROS_INFO("Starting fine registration...");
    Eigen::Matrix4f fine_transform = fineRegistration(target_processed, reference_cloud_processed_, coarse_transform);
    
    // Calculate measurements
    result = calculateMeasurements(target_processed, fine_transform);
    
    ROS_INFO("Registration completed - Fitness: %.6f, RMSE: %.6f", 
             result.registration_fitness, result.registration_rmse);
    
    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimator::preprocessPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Statistical outlier removal
    outlier_filter_.setInputCloud(cloud);
    outlier_filter_.filter(*processed);
    
    // Voxel grid downsampling
    voxel_filter_.setInputCloud(processed);
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter_.filter(*processed);
    
    return processed;
}

Eigen::Matrix4f PoseEstimator::coarseRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    
    // Try PCA alignment first
    Eigen::Matrix4f pca_transform = pcaAlignment(source, target);
    
    // Apply PCA transformation to source
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pca_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *source_pca_aligned, pca_transform);
    
    // Check if PCA alignment is reasonable by computing a simple distance metric
    pcl::PointXYZ source_centroid, target_centroid;
    Eigen::Vector4f centroid_s, centroid_t;
    pcl::compute3DCentroid(*source_pca_aligned, centroid_s);
    pcl::compute3DCentroid(*target, centroid_t);
    
    float centroid_distance = (centroid_s.head<3>() - centroid_t.head<3>()).norm();
    
    // If PCA alignment seems reasonable, return it
    if (centroid_distance < 0.1f) {  // 10cm threshold
        ROS_INFO("PCA alignment successful, centroid distance: %.3f", centroid_distance);
        return pca_transform;
    }
    
    // If PCA fails, try FPFH + RANSAC
    ROS_WARN("PCA alignment failed (distance: %.3f), trying FPFH alignment", centroid_distance);
    Eigen::Matrix4f fpfh_transform = fpfhAlignment(source, target);
    
    return fpfh_transform;
}

Eigen::Matrix4f PoseEstimator::pcaAlignment(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    
    // Compute PCA for target cloud
    Eigen::Matrix3f target_eigenvectors;
    Eigen::Vector3f target_eigenvalues;
    computePrincipalAxes(target, target_eigenvectors, target_eigenvalues);
    
    // Compute centroids
    Eigen::Vector4f source_centroid, target_centroid;
    pcl::compute3DCentroid(*source, source_centroid);
    pcl::compute3DCentroid(*target, target_centroid);
    
    // Compute rotation matrix (align principal axes)
    // Note: This is a simplified alignment - you might need to handle axis flipping
    Eigen::Matrix3f rotation = target_eigenvectors * ref_eigenvectors_.transpose();
    
    // Check for determinant to avoid reflection
    if (rotation.determinant() < 0) {
        // Flip the last column to avoid reflection
        target_eigenvectors.col(2) = -target_eigenvectors.col(2);
        rotation = target_eigenvectors * ref_eigenvectors_.transpose();
    }
    
    // Compute translation (align centroids after rotation)
    Eigen::Vector3f translation = target_centroid.head<3>() - rotation * source_centroid.head<3>();
    
    // Construct transformation matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;
    
    return transform;
}

Eigen::Matrix4f PoseEstimator::fpfhAlignment(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    
    // Compute normals for both clouds
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    
    normal_estimator_.setInputCloud(source);
    normal_estimator_.compute(*source_normals);
    
    normal_estimator_.setInputCloud(target);
    normal_estimator_.compute(*target_normals);
    
    // Compute FPFH features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator;
    fpfh_estimator.setRadiusSearch(fpfh_radius_);
    
    fpfh_estimator.setInputCloud(source);
    fpfh_estimator.setInputNormals(source_normals);
    fpfh_estimator.compute(*source_fpfh);
    
    fpfh_estimator.setInputCloud(target);
    fpfh_estimator.setInputNormals(target_normals);
    fpfh_estimator.compute(*target_fpfh);
    
    // RANSAC-based alignment
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
    align.setInputSource(source);
    align.setSourceFeatures(source_fpfh);
    align.setInputTarget(target);
    align.setTargetFeatures(target_fpfh);
    align.setMaximumIterations(50000);
    align.setNumberOfSamples(3);
    align.setCorrespondenceRandomness(2);
    align.setSimilarityThreshold(0.9f);
    align.setMaxCorrespondenceDistance(voxel_size_ * 2.5f);
    align.setInlierFraction(0.25f);
    
    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    align.align(aligned_source);
    
    if (align.hasConverged()) {
        ROS_INFO("FPFH alignment converged with fitness: %.6f", align.getFitnessScore());
        return align.getFinalTransformation();
    } else {
        ROS_WARN("FPFH alignment failed, returning identity transform");
        return Eigen::Matrix4f::Identity();
    }
}

Eigen::Matrix4f PoseEstimator::fineRegistration(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    const Eigen::Matrix4f& initial_guess) {
    
    // Apply initial transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *source_transformed, initial_guess);
    
    // Point-to-Plane ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_transformed);
    icp.setInputTarget(target);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setMaxCorrespondenceDistance(voxel_size_ * 5.0f);
    icp.setEuclideanFitnessEpsilon(1e-6);
    
    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    icp.align(aligned_source);
    
    if (icp.hasConverged()) {
        ROS_INFO("ICP converged after %d iterations", icp.nr_iterations_);
        // Combine initial transformation with ICP refinement
        return icp.getFinalTransformation() * initial_guess;
    } else {
        ROS_WARN("ICP failed to converge, using initial guess");
        return initial_guess;
    }
}

MeasurementData PoseEstimator::calculateMeasurements(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const Eigen::Matrix4f& transformation) {
    
    MeasurementData result;
    result.transformation = transformation;
    
    // Extract rotation and translation
    result.translation = transformation.block<3,1>(0,3);
    Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
    result.euler_angles = rotationMatrixToEulerAngles(rotation);
    
    // Apply transformation to cloud for measurements
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation);
    
    // Calculate bounding box dimensions in reference frame
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    result.length = max_pt.x - min_pt.x;
    result.width = max_pt.y - min_pt.y;
    result.height = max_pt.z - min_pt.z;
    
    // Advanced geometric measurements (if enabled)
    if (enable_advanced_measurements_) {
        result.geometric_features = measurement_calculator_->calculateGeometricFeatures(transformed_cloud);
        result.volume = result.geometric_features.volume;
        result.surface_area = result.geometric_features.surface_area;
        
        // Merge with basic measurements
        auto& advanced_angles = result.geometric_features.characteristic_angles;
        result.key_angles.insert(result.key_angles.end(), advanced_angles.begin(), advanced_angles.end());
    } else {
        // Basic volume estimation
        result.volume = result.length * result.width * result.height;
        result.surface_area = 2.0f * (result.length * result.width + 
                                     result.width * result.height + 
                                     result.height * result.length);
    }
    
    // Calculate some key angles based on principal axes
    Eigen::Matrix3f cloud_eigenvectors;
    Eigen::Vector3f cloud_eigenvalues;
    computePrincipalAxes(transformed_cloud, cloud_eigenvectors, cloud_eigenvalues);
    
    // Angle between principal axes and reference frame axes
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3f axis = Eigen::Vector3f::Unit(i);
        float angle = std::acos(std::clamp(std::abs(cloud_eigenvectors.col(0).dot(axis)), 0.0f, 1.0f));
        result.key_angles.push_back(angle);
    }
    
    // Calculate registration quality metrics
    // For this, we need to align the clouds and compute distances
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_eval;
    icp_eval.setInputSource(transformed_cloud);
    icp_eval.setInputTarget(reference_cloud_processed_);
    icp_eval.setMaximumIterations(1);  // Just for evaluation
    icp_eval.setMaxCorrespondenceDistance(voxel_size_ * 10.0f);
    
    pcl::PointCloud<pcl::PointXYZ> aligned_for_eval;
    icp_eval.align(aligned_for_eval);
    
    result.registration_fitness = icp_eval.getFitnessScore();
    
    // Calculate RMSE manually using KD-tree for efficient nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(reference_cloud_processed_);
    
    float sum_squared_distances = 0.0f;
    int valid_correspondences = 0;
    const float max_correspondence_dist = voxel_size_ * 10.0f;
    const float max_correspondence_dist_sq = max_correspondence_dist * max_correspondence_dist;
    
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
    
    if (valid_correspondences > 0) {
        result.registration_rmse = std::sqrt(sum_squared_distances / valid_correspondences);
    } else {
        result.registration_rmse = std::numeric_limits<float>::max();
    }
    
    return result;
}

void PoseEstimator::setProcessingParameters(const pose_measurement::ProcessingParameters& params) {
    if (cloud_processor_) {
        cloud_processor_->setParameters(params);
    }
    
    // Update basic parameters too
    voxel_size_ = params.voxel_size;
    
    if (measurement_calculator_) {
        measurement_calculator_->setVoxelSize(params.voxel_size);
    }
}

Eigen::Vector3f PoseEstimator::rotationMatrixToEulerAngles(const Eigen::Matrix3f& R) {
    // ZYX Euler angles (roll, pitch, yaw)
    float sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    
    bool singular = sy < 1e-6; // If sin(pitch) is close to zero
    
    float roll, pitch, yaw;
    
    if (!singular) {
        roll = std::atan2(R(2,1), R(2,2));
        pitch = std::atan2(-R(2,0), sy);
        yaw = std::atan2(R(1,0), R(0,0));
    } else {
        roll = std::atan2(-R(1,2), R(1,1));
        pitch = std::atan2(-R(2,0), sy);
        yaw = 0;
    }
    
    return Eigen::Vector3f(roll, pitch, yaw);
}

void PoseEstimator::computePrincipalAxes(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Eigen::Matrix3f& eigenvectors,
    Eigen::Vector3f& eigenvalues) {
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    
    eigenvectors = pca.getEigenVectors();
    eigenvalues = pca.getEigenValues();
    
    // Ensure consistent orientation (optional)
    // Make sure the first eigenvector points in positive direction
    if (eigenvectors(2, 0) < 0) {
        eigenvectors.col(0) = -eigenvectors.col(0);
    }
    
    // Ensure right-handed coordinate system
    Eigen::Vector3f cross_product = eigenvectors.col(0).cross(eigenvectors.col(1));
    if (cross_product.dot(eigenvectors.col(2)) < 0) {
        eigenvectors.col(2) = -eigenvectors.col(2);
    }
}