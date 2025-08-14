/**
* @file pose_measurement_node.cpp
* @brief 工件姿态估计和测量系统的主控制节点
* 
* 功能：
* - 参数管理：读取和配置系统参数（体素大小、ICP参数、扩展模式开关等）
* - 工作流控制：协调点云加载、姿态估计、几何测量的执行流程
* - 双模式处理：支持单次处理模式和交互式连续处理模式
* - 结果输出：格式化终端显示和CSV文件保存
* - 错误处理：完善的文件加载检查和异常处理
* 
* 作为系统入口点，该节点通过PoseEstimator调用核心算法模块，
* 为用户和外部脚本提供统一的ROS接口。
*/

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "pose_measurement/pose_estimator.h"

class PoseMeasurementNode {
public:
    PoseMeasurementNode() : nh_("~") {
        // Initialize parameters
        // 文件路径参数
        nh_.param<std::string>("reference_pcd_path", reference_pcd_path_, "");
        nh_.param<std::string>("target_pcd_path", target_pcd_path_, "");
        nh_.param<std::string>("output_csv_path", output_csv_path_, "measurements.csv");
        // 核心处理参数
        nh_.param<float>("voxel_size", voxel_size_, 0.005f);  // 5mm
        nh_.param<int>("icp_max_iterations", icp_max_iterations_, 100);
        nh_.param<float>("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-6);
        // 功能开关
        nh_.param<bool>("enable_csv_output", enable_csv_output_, true);
        nh_.param<bool>("enable_advanced_processing", enable_advanced_processing_, false);
        nh_.param<bool>("enable_plane_removal", enable_plane_removal_, true);
        nh_.param<bool>("enable_clustering", enable_clustering_, true);
        
        // Initialize pose estimator
        // 初始化PoseEstimator类
        pose_estimator_ = std::make_shared<PoseEstimator>();
        pose_estimator_->setVoxelSize(voxel_size_);
        pose_estimator_->setICPMaxIterations(icp_max_iterations_);
        pose_estimator_->setICPTransformationEpsilon(icp_transformation_epsilon_);
        pose_estimator_->setEnableAdvancedMeasurements(enable_advanced_processing_);
        
        // Configure advanced processing if enabled
        // 高级处理
        if (enable_advanced_processing_) {
            pose_measurement::ProcessingParameters params;
            params.voxel_size = voxel_size_;
            params.enable_plane_removal = enable_plane_removal_;
            params.enable_clustering = enable_clustering_;
            pose_estimator_->setProcessingParameters(params);
        }
        
        // Initialize CSV file
        if (enable_csv_output_) {
            initializeCSVFile();
        }
    }
    
    bool run() {
        // Load reference cloud
        if (reference_pcd_path_.empty()) {
            ROS_ERROR("Reference PCD path not specified!");
            return false;
        }
        
        ROS_INFO("Loading reference cloud: %s", reference_pcd_path_.c_str());
        if (!pose_estimator_->loadReferenceCloud(reference_pcd_path_)) {
            ROS_ERROR("Failed to load reference cloud!");
            return false;
        }
        
        // Process target cloud(s)
        if (!target_pcd_path_.empty()) {
            // Single target cloud mode
            return processSingleTarget();
        } else {
            // Interactive mode - process multiple clouds
            return processInteractiveMode();
        }
    }
    
private:
    bool processSingleTarget() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        ROS_INFO("Loading target cloud: %s", target_pcd_path_.c_str());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_path_, *target_cloud) == -1) {
            ROS_ERROR("Failed to load target cloud!");
            return false;
        }
        
        // Process and get measurements
        auto start_time = ros::Time::now();
        MeasurementData result = pose_estimator_->processTargetCloud(target_cloud);
        auto processing_time = (ros::Time::now() - start_time).toSec();
        
        // Display results
        displayResults(result, processing_time, target_pcd_path_);
        
        // Save to CSV
        if (enable_csv_output_) {
            saveToCSV(result, target_pcd_path_, processing_time);
        }
        
        return true;
    }
    
    bool processInteractiveMode() {
        ROS_INFO("=== Interactive Mode ===");
        ROS_INFO("Enter target PCD file paths (or 'quit' to exit):");
        
        std::string input;
        while (ros::ok()) {
            std::cout << "Target PCD path: ";
            std::getline(std::cin, input);
            
            if (input == "quit" || input == "q") {
                break;
            }
            
            if (input.empty()) {
                continue;
            }
            
            // Load target cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(input, *target_cloud) == -1) {
                ROS_ERROR("Failed to load target cloud: %s", input.c_str());
                continue;
            }
            
            // Process
            auto start_time = ros::Time::now();
            MeasurementData result = pose_estimator_->processTargetCloud(target_cloud);
            auto processing_time = (ros::Time::now() - start_time).toSec();
            
            // Display and save results
            displayResults(result, processing_time, input);
            if (enable_csv_output_) {
                saveToCSV(result, input, processing_time);
            }
            
            std::cout << std::endl;
        }
        
        return true;
    }
    
    void displayResults(const MeasurementData& data, double processing_time, const std::string& filename) {
        std::cout << "\n=== Measurement Results ===" << std::endl;
        std::cout << "File: " << filename << std::endl;
        std::cout << "Processing time: " << std::fixed << std::setprecision(3) 
                  << processing_time << " seconds" << std::endl;
        
        // Pose information
        std::cout << "\n--- Pose Information ---" << std::endl;
        std::cout << "Translation (x, y, z): " 
                  << std::fixed << std::setprecision(6)
                  << data.translation.x() << ", "
                  << data.translation.y() << ", "
                  << data.translation.z() << " [m]" << std::endl;
        
        std::cout << "Euler Angles (roll, pitch, yaw): "
                  << std::fixed << std::setprecision(3)
                  << data.euler_angles.x() * 180.0 / M_PI << "°, "
                  << data.euler_angles.y() * 180.0 / M_PI << "°, "
                  << data.euler_angles.z() * 180.0 / M_PI << "°" << std::endl;
        
        // Geometric measurements
        std::cout << "\n--- Geometric Measurements ---" << std::endl;
        std::cout << "Length: " << std::fixed << std::setprecision(6) << data.length << " [m]" << std::endl;
        std::cout << "Width:  " << std::fixed << std::setprecision(6) << data.width << " [m]" << std::endl;
        std::cout << "Height: " << std::fixed << std::setprecision(6) << data.height << " [m]" << std::endl;
        std::cout << "Volume: " << std::fixed << std::setprecision(6) << data.volume << " [m³]" << std::endl;
        std::cout << "Surface Area: " << std::fixed << std::setprecision(6) << data.surface_area << " [m²]" << std::endl;
        
        if (!data.key_angles.empty()) {
            std::cout << "Key Angles: ";
            for (size_t i = 0; i < data.key_angles.size(); ++i) {
                std::cout << std::fixed << std::setprecision(2) 
                          << data.key_angles[i] * 180.0 / M_PI << "°";
                if (i < data.key_angles.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
        
        // Quality metrics
        std::cout << "\n--- Registration Quality ---" << std::endl;
        std::cout << "Fitness Score: " << std::fixed << std::setprecision(6) << data.registration_fitness << std::endl;
        std::cout << "RMSE: " << std::fixed << std::setprecision(6) << data.registration_rmse << " [m]" << std::endl;
        
        // Transformation matrix
        std::cout << "\n--- Transformation Matrix ---" << std::endl;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << std::fixed << std::setprecision(6) << data.transformation(i, j);
                if (j < 3) std::cout << "\t";
            }
            std::cout << std::endl;
        }
    }
    
    void initializeCSVFile() {
        std::ofstream file(output_csv_path_);
        if (!file.is_open()) {
            ROS_ERROR("Failed to create CSV file: %s", output_csv_path_.c_str());
            return;
        }
        
        file << "filename,processing_time,"
             << "tx,ty,tz,roll,pitch,yaw,"
             << "length,width,height,volume,surface_area,"
             << "fitness,rmse" << std::endl;
        
        file.close();
        ROS_INFO("CSV file initialized: %s", output_csv_path_.c_str());
    }
    
    void saveToCSV(const MeasurementData& data, const std::string& filename, double processing_time) {
        std::ofstream file(output_csv_path_, std::ios::app);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open CSV file for writing");
            return;
        }
        
        file << filename << ","
             << std::fixed << std::setprecision(6) << processing_time << ","
             << data.translation.x() << ","
             << data.translation.y() << ","
             << data.translation.z() << ","
             << data.euler_angles.x() * 180.0 / M_PI << ","
             << data.euler_angles.y() * 180.0 / M_PI << ","
             << data.euler_angles.z() * 180.0 / M_PI << ","
             << data.length << ","
             << data.width << ","
             << data.height << ","
             << data.volume << ","
             << data.surface_area << ","
             << data.registration_fitness << ","
             << data.registration_rmse << std::endl;
        
        file.close();
    }
    
    ros::NodeHandle nh_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    
    // Parameters
    std::string reference_pcd_path_;
    std::string target_pcd_path_;
    std::string output_csv_path_;
    bool enable_csv_output_;
    float voxel_size_;
    int icp_max_iterations_;
    float icp_transformation_epsilon_;
    bool enable_advanced_processing_;
    bool enable_plane_removal_;
    bool enable_clustering_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_measurement_node");
    
    PoseMeasurementNode node;
    
    if (!node.run()) {
        ROS_ERROR("Pose measurement node failed to run!");
        return -1;
    }
    
    ROS_INFO("Pose measurement completed successfully.");
    return 0;
}