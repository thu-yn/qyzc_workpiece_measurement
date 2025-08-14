#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pose_measurement {

struct ProcessingParameters {
    // Filtering parameters
    float voxel_size = 0.005f;
    int statistical_k = 50;
    float statistical_stddev = 1.0f;
    float radius_outlier_radius = 0.01f;
    int radius_outlier_min_neighbors = 10;
    
    // Segmentation parameters
    bool enable_plane_removal = true;
    float plane_distance_threshold = 0.01f;
    int plane_max_iterations = 1000;
    
    // Clustering parameters
    bool enable_clustering = true;
    float cluster_tolerance = 0.02f;
    int cluster_min_size = 100;
    int cluster_max_size = 100000;
};

class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();
    
    // Main processing pipeline
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    
    // Individual processing steps
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr removePlanes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLargestCluster(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Configuration
    void setParameters(const ProcessingParameters& params) { params_ = params; }
    ProcessingParameters getParameters() const { return params_; }
    
    // Utility functions
    static void printCloudInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name = "Cloud");
    
private:
    ProcessingParameters params_;
    
    // PCL filter objects
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_filter_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter_;
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter_;
    
    // Segmentation objects
    pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction_;
};

} // namespace pose_measurement

#endif // POINT_CLOUD_PROCESSOR_H