#include "pose_measurement/point_cloud_processor.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <ros/ros.h>

namespace pose_measurement {

PointCloudProcessor::PointCloudProcessor() {
    // Initialize segmentation parameters
    plane_segmentation_.setOptimizeCoefficients(true);
    plane_segmentation_.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation_.setMethodType(pcl::SAC_RANSAC);
    
    // Initialize cluster extraction
    cluster_extraction_.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
}

PointCloudProcessor::~PointCloudProcessor() {}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::processPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN("Empty or null input cloud provided to PointCloudProcessor");
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    printCloudInfo(input_cloud, "Input");
    
    // Step 1: Remove outliers
    auto cloud_filtered = removeOutliers(input_cloud);
    printCloudInfo(cloud_filtered, "After outlier removal");
    
    // Step 2: Downsample
    auto cloud_downsampled = downsample(cloud_filtered);
    printCloudInfo(cloud_downsampled, "After downsampling");
    
    // Step 3: Remove planes (optional)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_planes;
    if (params_.enable_plane_removal) {
        cloud_no_planes = removePlanes(cloud_downsampled);
        printCloudInfo(cloud_no_planes, "After plane removal");
    } else {
        cloud_no_planes = cloud_downsampled;
    }
    
    // Step 4: Extract largest cluster (optional)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::removeOutliers(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Statistical outlier removal
    statistical_filter_.setInputCloud(cloud);
    statistical_filter_.setMeanK(params_.statistical_k);
    statistical_filter_.setStddevMulThresh(params_.statistical_stddev);
    statistical_filter_.filter(*cloud_filtered);
    
    // Radius outlier removal (additional filtering)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    radius_filter_.setInputCloud(cloud_filtered);
    radius_filter_.setRadiusSearch(params_.radius_outlier_radius);
    radius_filter_.setMinNeighborsInRadius(params_.radius_outlier_min_neighbors);
    radius_filter_.filter(*cloud_radius_filtered);
    
    return cloud_radius_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::downsample(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
    voxel_filter_.filter(*cloud_filtered);
    
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::removePlanes(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_planes(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_no_planes);
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    int original_size = cloud_no_planes->size();
    
    // Remove multiple planes iteratively
    while (cloud_no_planes->size() > 0.1 * original_size) {  // Keep at least 10% of original points
        // Segment the largest planar component
        plane_segmentation_.setInputCloud(cloud_no_planes);
        plane_segmentation_.setDistanceThreshold(params_.plane_distance_threshold);
        plane_segmentation_.setMaxIterations(params_.plane_max_iterations);
        plane_segmentation_.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() == 0) {
            ROS_DEBUG("Could not estimate a planar model for the given dataset.");
            break;
        }
        
        // Check if the plane is significant (contains enough points)
        float plane_ratio = static_cast<float>(inliers->indices.size()) / cloud_no_planes->size();
        if (plane_ratio < 0.1) {  // Plane must contain at least 10% of points to be considered significant
            ROS_DEBUG("Plane too small (%.1f%%), stopping plane removal", plane_ratio * 100);
            break;
        }
        
        // Extract the planar inliers (remove them)
        extract.setInputCloud(cloud_no_planes);
        extract.setIndices(inliers);
        extract.setNegative(true);  // Remove the inliers, keep the rest
        extract.filter(*cloud_no_planes);
        
        ROS_DEBUG("Removed plane with %zu points (%.1f%%)", 
                 inliers->indices.size(), plane_ratio * 100);
    }
    
    return cloud_no_planes;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudProcessor::extractClusters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    
    if (cloud->empty()) {
        return clusters;
    }
    
    // Set up cluster extraction
    cluster_extraction_.setInputCloud(cloud);
    cluster_extraction_.setClusterTolerance(params_.cluster_tolerance);
    cluster_extraction_.setMinClusterSize(params_.cluster_min_size);
    cluster_extraction_.setMaxClusterSize(params_.cluster_max_size);
    
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extraction_.extract(cluster_indices);
    
    // Extract each cluster
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(indices));
        extract.setIndices(cluster_indices_ptr);
        extract.setNegative(false);
        extract.filter(*cluster);
        
        clusters.push_back(cluster);
    }
    
    ROS_DEBUG("Extracted %zu clusters", clusters.size());
    
    return clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::getLargestCluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    auto clusters = extractClusters(cloud);
    
    if (clusters.empty()) {
        ROS_WARN("No clusters found, returning original cloud");
        return cloud;
    }
    
    // Find the largest cluster
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

void PointCloudProcessor::printCloudInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name) {
    if (!cloud) {
        ROS_INFO("%s: NULL cloud", name.c_str());
        return;
    }
    
    if (cloud->empty()) {
        ROS_INFO("%s: Empty cloud", name.c_str());
        return;
    }
    
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    
    ROS_INFO("%s: %zu points, Bounds: [%.3f,%.3f] [%.3f,%.3f] [%.3f,%.3f], Centroid: [%.3f,%.3f,%.3f]",
             name.c_str(), cloud->size(),
             min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z,
             centroid[0], centroid[1], centroid[2]);
}

} // namespace pose_measurement