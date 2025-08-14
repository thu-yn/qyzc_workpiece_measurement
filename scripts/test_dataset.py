#!/usr/bin/env python3
"""
文件名: test_dataset.py
文件作用: 批量测试脚本，用于自动化处理多视角点云数据集的姿态测量任务

主要功能需求:
1. 批量处理多个PCD文件的姿态测量任务
2. 自动调用ROS姿态测量节点并管理参数
3. 结果文件管理和CSV数据合并
4. 提供命令行接口和灵活的配置选项
5. 错误处理和超时控制，确保批处理稳定性

使用场景:
- 工业质检的批量数据处理
- 算法性能评估和基准测试
- 多视角数据集的自动化分析
- 科研实验的数据处理自动化

设计特点:
- 命令行友好：支持灵活的参数配置
- 错误恢复：单个文件失败不影响整体处理
- 进度监控：实时显示处理状态和统计信息
- 结果管理：自动生成和合并CSV报告

依赖环境: ROS环境、Python3、pose_measurement包
"""

import os
import sys
import subprocess
import glob
import argparse
from pathlib import Path

def run_pose_measurement(reference_pcd, target_pcd, output_dir):
    """
    运行单个姿态测量任务
    
    Args:
        reference_pcd (str): 参考点云文件路径（标准姿态）
        target_pcd (str): 目标点云文件路径（待测姿态）
        output_dir (str): 输出目录路径
    
    Returns:
        bool: 处理是否成功
    
    功能说明:
    1. 构建roslaunch命令并执行姿态测量
    2. 处理超时和异常情况
    3. 返回处理结果状态
    """
    
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 构建输出CSV文件名
    # 使用目标文件的基础名称作为结果文件名
    target_name = Path(target_pcd).stem
    output_csv = os.path.join(output_dir, f"result_{target_name}.csv")
    
    # 构建roslaunch命令
    # 使用pose_measurement包的launch文件
    cmd = [
        'roslaunch', 'pose_measurement', 'pose_measurement.launch',
        f'reference_pcd:={reference_pcd}',      # 参考点云路径
        f'target_pcd:={target_pcd}',            # 目标点云路径
        f'output_csv:={output_csv}',            # 输出CSV路径
        'enable_csv:=true'                      # 启用CSV输出
    ]
    
    print(f"Processing: {target_name}")
    print(f"Command: {' '.join(cmd)}")
    
    try:
        # 运行命令，设置5分钟超时
        # capture_output=True: 捕获标准输出和错误输出
        # text=True: 以文本模式处理输出
        # timeout=300: 5分钟超时限制
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        
        # 检查命令执行结果
        if result.returncode == 0:
            print(f"✓ Success: {target_name}")
            return True
        else:
            print(f"✗ Failed: {target_name}")
            print(f"Error: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        # 处理超时情况
        print(f"✗ Timeout: {target_name}")
        return False
    except Exception as e:
        # 处理其他异常
        print(f"✗ Exception: {target_name} - {str(e)}")
        return False

def batch_process_dataset(dataset_dir, reference_file, output_dir, file_pattern="*.pcd"):
    """
    批量处理数据集
    
    Args:
        dataset_dir (str): 数据集目录路径
        reference_file (str): 参考文件路径
        output_dir (str): 输出目录路径
        file_pattern (str): 文件匹配模式，默认"*.pcd"
    
    Returns:
        bool: 批处理是否完全成功
    
    处理流程:
    1. 验证输入参数和文件存在性
    2. 搜索并过滤目标文件列表
    3. 逐个处理目标文件
    4. 统计和报告处理结果
    """
    
    # 检查参考文件是否存在
    if not os.path.exists(reference_file):
        print(f"Error: Reference file not found: {reference_file}")
        return False
    
    # 搜索目标文件
    # 使用glob模式匹配查找所有符合条件的PCD文件
    search_pattern = os.path.join(dataset_dir, file_pattern)
    target_files = glob.glob(search_pattern)
    
    if not target_files:
        print(f"Error: No target files found with pattern: {search_pattern}")
        return False
    
    # 过滤掉参考文件，避免自己与自己比较
    target_files = [f for f in target_files if f != reference_file]
    target_files.sort()  # 按文件名排序，确保处理顺序一致
    
    # 显示批处理信息
    print(f"Found {len(target_files)} target files to process")
    print(f"Reference file: {reference_file}")
    print(f"Output directory: {output_dir}")
    print("-" * 50)
    
    # 批量处理循环
    success_count = 0
    for target_file in target_files:
        if run_pose_measurement(reference_file, target_file, output_dir):
            success_count += 1
        print()  # 添加空行分隔，便于阅读
    
    # 输出处理结果统计
    print("-" * 50)
    print(f"Processing completed: {success_count}/{len(target_files)} successful")
    
    # 返回是否全部成功
    return success_count == len(target_files)

def merge_csv_results(output_dir, merged_filename="merged_results.csv"):
    """
    合并所有CSV结果文件
    
    Args:
        output_dir (str): 结果文件目录
        merged_filename (str): 合并后的文件名
    
    功能说明:
    1. 搜索所有result_*.csv文件
    2. 合并表头和数据行
    3. 生成统一的结果文件
    
    CSV合并策略:
    - 只写入一次表头（从第一个文件）
    - 跳过其他文件的表头行
    - 按文件名顺序合并数据
    """
    
    # 搜索所有结果CSV文件
    csv_files = glob.glob(os.path.join(output_dir, "result_*.csv"))
    
    if not csv_files:
        print("No CSV files found to merge")
        return
    
    # 构建合并文件路径
    merged_path = os.path.join(output_dir, merged_filename)
    
    # 合并CSV文件
    with open(merged_path, 'w') as outfile:
        header_written = False
        
        # 逐个处理CSV文件
        for csv_file in sorted(csv_files):
            with open(csv_file, 'r') as infile:
                lines = infile.readlines()
                
                # 处理表头：只在第一个文件时写入
                if not header_written and lines:
                    outfile.write(lines[0])  # 写入表头行
                    header_written = True
                
                # 写入数据行（跳过表头）
                if len(lines) > 1:
                    outfile.write(lines[1])  # 写入数据行
    
    print(f"Merged results saved to: {merged_path}")

def main():
    """
    主函数：解析命令行参数并执行相应操作
    
    命令行接口设计:
    - 位置参数：数据集目录和参考文件（必需）
    - 可选参数：输出目录、文件模式、合并选项
    - 帮助信息：提供详细的使用说明
    
    程序流程:
    1. 参数解析和验证
    2. ROS环境检查
    3. 批量处理执行
    4. 结果合并（可选）
    """
    
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='Batch process point cloud dataset for pose measurement')
    
    # 定义命令行参数
    parser.add_argument('dataset_dir', help='Directory containing target PCD files')
    parser.add_argument('reference_file', help='Path to reference PCD file')
    parser.add_argument('-o', '--output', default='./results', 
                       help='Output directory for results (default: ./results)')
    parser.add_argument('-p', '--pattern', default='*.pcd',
                       help='File pattern for target files (default: *.pcd)')
    parser.add_argument('--merge', action='store_true',
                       help='Merge all CSV results into one file')
    
    # 解析命令行参数
    args = parser.parse_args()
    
    # 检查ROS环境
    # 确保ROS已正确设置，pose_measurement包可用
    if 'ROS_DISTRO' not in os.environ:
        print("Error: ROS environment not found. Please source setup.bash first.")
        sys.exit(1)
    
    # 执行批量处理
    success = batch_process_dataset(
        args.dataset_dir, 
        args.reference_file, 
        args.output, 
        args.pattern
    )
    
    # 可选的结果合并
    if success and args.merge:
        merge_csv_results(args.output)
    
    # 根据处理结果设置退出码
    # 0: 成功，1: 失败
    sys.exit(0 if success else 1)

# Python脚本入口点
if __name__ == "__main__":
    main()

"""
使用示例:

1. 基础批量处理:
   python3 test_dataset.py /path/to/dataset/ /path/to/reference.pcd

2. 指定输出目录:
   python3 test_dataset.py /path/to/dataset/ /path/to/reference.pcd -o /path/to/output/

3. 自定义文件模式:
   python3 test_dataset.py /path/to/dataset/ /path/to/reference.pcd -p "target_*.pcd"

4. 处理并合并结果:
   python3 test_dataset.py /path/to/dataset/ /path/to/reference.pcd --merge

5. 完整示例:
   python3 test_dataset.py \
       /home/user/data/objects/ \
       /home/user/data/reference.pcd \
       -o /home/user/results/ \
       -p "obj_*.pcd" \
       --merge

输出文件结构:
results/
├── result_target_001.csv     # 单个文件结果
├── result_target_002.csv
├── ...
└── merged_results.csv        # 合并结果（可选）

错误处理:
- 文件不存在：友好的错误提示
- ROS环境未设置：自动检查和提示
- 处理超时：5分钟超时限制
- 单个文件失败：不影响后续处理
- 异常捕获：详细的错误信息记录
"""