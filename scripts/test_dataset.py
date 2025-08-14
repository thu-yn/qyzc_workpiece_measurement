#!/usr/bin/env python3
"""
测试脚本：用于批量处理多视角数据集
"""

import os
import sys
import subprocess
import glob
import argparse
from pathlib import Path

def run_pose_measurement(reference_pcd, target_pcd, output_dir):
    """运行姿态测量节点"""
    
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 构建输出CSV文件名
    target_name = Path(target_pcd).stem
    output_csv = os.path.join(output_dir, f"result_{target_name}.csv")
    
    # 构建roslaunch命令
    cmd = [
        'roslaunch', 'pose_measurement', 'pose_measurement.launch',
        f'reference_pcd:={reference_pcd}',
        f'target_pcd:={target_pcd}',
        f'output_csv:={output_csv}',
        'enable_csv:=true'
    ]
    
    print(f"Processing: {target_name}")
    print(f"Command: {' '.join(cmd)}")
    
    try:
        # 运行命令
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        
        if result.returncode == 0:
            print(f"✓ Success: {target_name}")
            return True
        else:
            print(f"✗ Failed: {target_name}")
            print(f"Error: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"✗ Timeout: {target_name}")
        return False
    except Exception as e:
        print(f"✗ Exception: {target_name} - {str(e)}")
        return False

def batch_process_dataset(dataset_dir, reference_file, output_dir, file_pattern="*.pcd"):
    """批量处理数据集"""
    
    # 检查参考文件
    if not os.path.exists(reference_file):
        print(f"Error: Reference file not found: {reference_file}")
        return False
    
    # 查找目标文件
    search_pattern = os.path.join(dataset_dir, file_pattern)
    target_files = glob.glob(search_pattern)
    
    if not target_files:
        print(f"Error: No target files found with pattern: {search_pattern}")
        return False
    
    # 过滤掉参考文件
    target_files = [f for f in target_files if f != reference_file]
    target_files.sort()
    
    print(f"Found {len(target_files)} target files to process")
    print(f"Reference file: {reference_file}")
    print(f"Output directory: {output_dir}")
    print("-" * 50)
    
    # 批量处理
    success_count = 0
    for target_file in target_files:
        if run_pose_measurement(reference_file, target_file, output_dir):
            success_count += 1
        print()  # 空行分隔
    
    print("-" * 50)
    print(f"Processing completed: {success_count}/{len(target_files)} successful")
    
    return success_count == len(target_files)

def merge_csv_results(output_dir, merged_filename="merged_results.csv"):
    """合并所有CSV结果文件"""
    
    csv_files = glob.glob(os.path.join(output_dir, "result_*.csv"))
    
    if not csv_files:
        print("No CSV files found to merge")
        return
    
    merged_path = os.path.join(output_dir, merged_filename)
    
    with open(merged_path, 'w') as outfile:
        # 写入表头
        header_written = False
        
        for csv_file in sorted(csv_files):
            with open(csv_file, 'r') as infile:
                lines = infile.readlines()
                
                if not header_written and lines:
                    outfile.write(lines[0])  # 写入表头
                    header_written = True
                
                if len(lines) > 1:
                    outfile.write(lines[1])  # 写入数据行
    
    print(f"Merged results saved to: {merged_path}")

def main():
    parser = argparse.ArgumentParser(description='Batch process point cloud dataset for pose measurement')
    
    parser.add_argument('dataset_dir', help='Directory containing target PCD files')
    parser.add_argument('reference_file', help='Path to reference PCD file')
    parser.add_argument('-o', '--output', default='./results', 
                       help='Output directory for results (default: ./results)')
    parser.add_argument('-p', '--pattern', default='*.pcd',
                       help='File pattern for target files (default: *.pcd)')
    parser.add_argument('--merge', action='store_true',
                       help='Merge all CSV results into one file')
    
    args = parser.parse_args()
    
    # 检查ROS环境
    if 'ROS_DISTRO' not in os.environ:
        print("Error: ROS environment not found. Please source setup.bash first.")
        sys.exit(1)
    
    # 批量处理
    success = batch_process_dataset(
        args.dataset_dir, 
        args.reference_file, 
        args.output, 
        args.pattern
    )
    
    # 合并结果
    if success and args.merge:
        merge_csv_results(args.output)
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()