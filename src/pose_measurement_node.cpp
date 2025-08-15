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
* - 下采样控制：支持禁用下采样以保持高精度点云
* - 变换点云：支持生成和保存变换后的点云用于可视化对比（新增功能）
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
#include <sstream>

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
        nh_.param<bool>("enable_downsampling", enable_downsampling_, true); 
        nh_.param<bool>("enable_advanced_processing", enable_advanced_processing_, false);
        nh_.param<bool>("enable_plane_removal", enable_plane_removal_, true);
        nh_.param<bool>("enable_clustering", enable_clustering_, true);
        
        // 变换点云功能参数（新增）
        nh_.param<bool>("enable_transformed_cloud_save", enable_transformed_cloud_save_, false);
        nh_.param<std::string>("transformed_cloud_output_dir", transformed_cloud_output_dir_, "./results/");
        
        // 验证参数合理性
        validateParameters();
        
        // Initialize pose estimator
        pose_estimator_ = std::make_shared<PoseEstimator>();
        configurePoseEstimator();
        
        // Initialize CSV file
        if (enable_csv_output_) {
            initializeCSVFile();
        }
        
        // 输出配置信息
        printConfiguration();
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
    void validateParameters() {
        if (enable_downsampling_ && voxel_size_ <= 0.0f) {
            ROS_WARN("Downsampling enabled but voxel_size <= 0 (%.6f). Setting voxel_size to 0.005m", voxel_size_);
            voxel_size_ = 0.005f;
        }
        
        if (!enable_downsampling_) {
            ROS_INFO("Downsampling disabled - using original point cloud resolution");
        }
        
        // 验证ICP参数
        if (icp_max_iterations_ <= 0) {
            ROS_WARN("Invalid ICP max iterations (%d). Setting to 100", icp_max_iterations_);
            icp_max_iterations_ = 100;
        }
        
        if (icp_transformation_epsilon_ <= 0.0f) {
            ROS_WARN("Invalid ICP transformation epsilon (%.2e). Setting to 1e-6", icp_transformation_epsilon_);
            icp_transformation_epsilon_ = 1e-6;
        }
    }
    
    void configurePoseEstimator() {
        // 设置基础参数
        pose_estimator_->setVoxelSize(voxel_size_);
        pose_estimator_->setICPMaxIterations(icp_max_iterations_);
        pose_estimator_->setICPTransformationEpsilon(icp_transformation_epsilon_);
        pose_estimator_->setEnableAdvancedMeasurements(enable_advanced_processing_);
        
        // 设置下采样开关
        pose_estimator_->setEnableDownsampling(enable_downsampling_);
        
        // Configure advanced processing if enabled
        if (enable_advanced_processing_) {
            pose_measurement::ProcessingParameters params;
            params.voxel_size = voxel_size_;
            params.enable_plane_removal = enable_plane_removal_;
            params.enable_clustering = enable_clustering_;
            params.enable_downsampling = enable_downsampling_;
            
            pose_estimator_->setProcessingParameters(params);
        }
    }
    
    void printConfiguration() {
        ROS_INFO("=== Configuration ===");
        ROS_INFO("Downsampling: %s", enable_downsampling_ ? "ENABLED" : "DISABLED");
        if (enable_downsampling_) {
            ROS_INFO("Voxel size: %.6f [m]", voxel_size_);
        }
        ROS_INFO("Advanced processing: %s", enable_advanced_processing_ ? "ENABLED" : "DISABLED");
        if (enable_advanced_processing_) {
            ROS_INFO("  - Plane removal: %s", enable_plane_removal_ ? "ENABLED" : "DISABLED");
            ROS_INFO("  - Clustering: %s", enable_clustering_ ? "ENABLED" : "DISABLED");
        }
        ROS_INFO("ICP max iterations: %d", icp_max_iterations_);
        ROS_INFO("ICP transformation epsilon: %.2e", icp_transformation_epsilon_);
        ROS_INFO("CSV output: %s", enable_csv_output_ ? "ENABLED" : "DISABLED");
        if (enable_csv_output_) {
            ROS_INFO("CSV file: %s", output_csv_path_.c_str());
        }
        ROS_INFO("Transformed cloud save: %s", enable_transformed_cloud_save_ ? "ENABLED" : "DISABLED");
        if (enable_transformed_cloud_save_) {
            ROS_INFO("Transformed cloud output directory: %s", transformed_cloud_output_dir_.c_str());
        }
        ROS_INFO("====================");
    }
    
    bool processSingleTarget() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        ROS_INFO("Loading target cloud: %s", target_pcd_path_.c_str());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_path_, *target_cloud) == -1) {
            ROS_ERROR("Failed to load target cloud!");
            return false;
        }
        
        ROS_INFO("Target cloud loaded: %zu points", target_cloud->size());
        
        // Process and get measurements
        auto start_time = ros::Time::now();
        MeasurementData result = pose_estimator_->processTargetCloud(target_cloud);
        auto processing_time = (ros::Time::now() - start_time).toSec();
        
        // Handle transformed cloud save if enabled
        std::string transformed_cloud_path;
        if (enable_transformed_cloud_save_) {
            transformed_cloud_path = generateTransformedCloudPath(target_pcd_path_);
            if (!saveTransformedCloud(target_cloud, result.transformation, transformed_cloud_path)) {
                ROS_WARN("Failed to save transformed cloud, but continuing with other results");
            }
        }
        
        // Display results
        displayResults(result, processing_time, target_pcd_path_, transformed_cloud_path);
        
        // Save to CSV
        if (enable_csv_output_) {
            saveToCSV(result, target_pcd_path_, processing_time, transformed_cloud_path);
        }
        
        return true;
    }
    
    bool processInteractiveMode() {
        ROS_INFO("=== Interactive Mode ===");
        if (enable_transformed_cloud_save_) {
            ROS_INFO("Transformed cloud save is enabled. You can specify custom file names.");
        }
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
            
            ROS_INFO("Target cloud loaded: %zu points", target_cloud->size());
            
            // Process
            auto start_time = ros::Time::now();
            MeasurementData result = pose_estimator_->processTargetCloud(target_cloud);
            auto processing_time = (ros::Time::now() - start_time).toSec();
            
            // Handle transformed cloud save if enabled
            std::string transformed_cloud_path;
            if (enable_transformed_cloud_save_) {
                transformed_cloud_path = getInteractiveTransformedCloudPath(input);
                if (!transformed_cloud_path.empty()) {
                    if (!saveTransformedCloud(target_cloud, result.transformation, transformed_cloud_path)) {
                        ROS_WARN("Failed to save transformed cloud, but continuing with other results");
                        transformed_cloud_path.clear(); // 清空路径表示保存失败
                    }
                }
            }
            
            // Display and save results
            displayResults(result, processing_time, input, transformed_cloud_path);
            if (enable_csv_output_) {
                saveToCSV(result, input, processing_time, transformed_cloud_path);
            }
            
            std::cout << std::endl;
        }
        
        return true;
    }
    
    /**
    * @brief 生成变换后点云的保存路径
    * @param target_pcd_path 目标点云路径
    * @return 变换后点云的保存路径
    */
    std::string generateTransformedCloudPath(const std::string& target_pcd_path) {
        // 提取文件名（不含路径和扩展名）
        size_t last_slash = target_pcd_path.find_last_of("/\\");
        size_t last_dot = target_pcd_path.find_last_of(".");
        
        std::string filename;
        if (last_slash != std::string::npos) {
            if (last_dot != std::string::npos && last_dot > last_slash) {
                filename = target_pcd_path.substr(last_slash + 1, last_dot - last_slash - 1);
            } else {
                filename = target_pcd_path.substr(last_slash + 1);
            }
        } else {
            if (last_dot != std::string::npos) {
                filename = target_pcd_path.substr(0, last_dot);
            } else {
                filename = target_pcd_path;
            }
        }
        
        // 生成时间戳
        auto now = ros::Time::now();
        std::stringstream ss;
        ss << std::fixed << std::setprecision(0) << now.toSec();
        std::string timestamp = ss.str();
        
        // 构造完整路径
        std::string transformed_filename = "transformed_" + filename + "_" + timestamp + ".pcd";
        return transformed_cloud_output_dir_ + transformed_filename;
    }
    
    /**
    * @brief 在交互模式下获取变换后点云的保存路径
    * @param target_pcd_path 目标点云路径
    * @return 用户指定的保存路径，空字符串表示跳过保存
    */
    std::string getInteractiveTransformedCloudPath(const std::string& target_pcd_path) {
        // 生成默认文件名建议
        std::string default_path = generateTransformedCloudPath(target_pcd_path);
        
        std::cout << "Save transformed cloud? (y/n/custom): ";
        std::string choice;
        std::getline(std::cin, choice);
        
        if (choice == "n" || choice == "no" || choice == "N") {
            return ""; // 跳过保存
        } else if (choice == "y" || choice == "yes" || choice == "Y" || choice.empty()) {
            return default_path; // 使用默认路径
        } else if (choice == "custom" || choice == "c" || choice == "C") {
            std::cout << "Enter custom file path (default: " << default_path << "): ";
            std::string custom_path;
            std::getline(std::cin, custom_path);
            
            if (custom_path.empty()) {
                return default_path;
            } else {
                // 确保路径有.pcd扩展名
                if (custom_path.find(".pcd") == std::string::npos) {
                    custom_path += ".pcd";
                }
                return custom_path;
            }
        } else {
            // 用户直接输入了路径
            if (choice.find(".pcd") == std::string::npos) {
                choice += ".pcd";
            }
            return choice;
        }
    }
    
    /**
    * @brief 保存变换后的点云
    * @param target_cloud 原始目标点云
    * @param transformation 变换矩阵
    * @param file_path 保存路径
    * @return 成功返回true，失败返回false
    */
    bool saveTransformedCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
        const Eigen::Matrix4f& transformation,
        const std::string& file_path) {
        
        if (file_path.empty()) {
            return false;
        }
        
        // 生成变换后的点云
        auto transformed_cloud = pose_estimator_->generateTransformedCloud(target_cloud, transformation);
        
        if (!transformed_cloud || transformed_cloud->empty()) {
            ROS_ERROR("Failed to generate transformed cloud");
            return false;
        }
        
        // 保存到文件
        bool success = pose_estimator_->saveTransformedCloud(transformed_cloud, file_path);
        
        if (success) {
            ROS_INFO("Transformed cloud saved successfully: %s", file_path.c_str());
        } else {
            ROS_ERROR("Failed to save transformed cloud: %s", file_path.c_str());
        }
        
        return success;
    }
    
    void displayResults(const MeasurementData& data, double processing_time, 
                       const std::string& filename, const std::string& transformed_cloud_path = "") {
        std::cout << "\n=== Measurement Results ===" << std::endl;
        std::cout << "File: " << filename << std::endl;
        std::cout << "Processing time: " << std::fixed << std::setprecision(3) 
                  << processing_time << " seconds" << std::endl;
        
        // 显示处理模式信息
        std::cout << "Processing mode: " << (enable_downsampling_ ? "With downsampling" : "No downsampling (full resolution)") << std::endl;
        
        // 显示变换点云保存信息
        if (enable_transformed_cloud_save_) {
            if (!transformed_cloud_path.empty()) {
                std::cout << "Transformed cloud saved: " << transformed_cloud_path << std::endl;
            } else {
                std::cout << "Transformed cloud: Not saved" << std::endl;
            }
        }
        
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
        
        file << "filename,processing_time,downsampling_enabled,"
             << "tx,ty,tz,roll,pitch,yaw,"
             << "length,width,height,volume,surface_area,"
             << "fitness,rmse";
        
        // 添加变换点云路径列（如果启用）
        if (enable_transformed_cloud_save_) {
            file << ",transformed_cloud_path";
        }
        
        file << std::endl;
        
        file.close();
        ROS_INFO("CSV file initialized: %s", output_csv_path_.c_str());
    }
    
    void saveToCSV(const MeasurementData& data, const std::string& filename, 
                  double processing_time, const std::string& transformed_cloud_path = "") {
        std::ofstream file(output_csv_path_, std::ios::app);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open CSV file for writing");
            return;
        }
        
        file << filename << ","
             << std::fixed << std::setprecision(6) << processing_time << ","
             << (enable_downsampling_ ? "true" : "false") << ","
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
             << data.registration_rmse;
        
        // 添加变换点云路径（如果启用）
        if (enable_transformed_cloud_save_) {
            file << "," << (transformed_cloud_path.empty() ? "not_saved" : transformed_cloud_path);
        }
        
        file << std::endl;
        
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
    bool enable_downsampling_;  
    bool enable_advanced_processing_;
    bool enable_plane_removal_;
    bool enable_clustering_;
    
    // 变换点云功能参数（新增）
    bool enable_transformed_cloud_save_;        // 是否启用变换点云保存功能
    std::string transformed_cloud_output_dir_;  // 变换点云输出目录
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