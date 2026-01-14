#!/usr/bin/env python3
"""
Generate flame graph for KF-GINS performance analysis

This script uses py-spy to collect performance data and generate a flame graph
"""

import os
import sys
import subprocess
import tempfile
import shutil

# Check if py-spy is installed
try:
    subprocess.run(["py-spy", "--version"], check=True, capture_output=True)
    print("py-spy is already installed")
except (subprocess.CalledProcessError, FileNotFoundError):
    print("Installing py-spy...")
    subprocess.run(["pip", "install", "py-spy"], check=True)

# Define the KF-GINS executable path and config file
executable_path = os.path.join(os.getcwd(), "bin", "KF-GINS.exe")
config_path = os.path.join(os.getcwd(), "dataset", "kf-gins.yaml")

# Check if executable exists
if not os.path.exists(executable_path):
    print(f"Error: KF-GINS executable not found at {executable_path}")
    print("Please build the project first")
    sys.exit(1)

# Check if config file exists
if not os.path.exists(config_path):
    print(f"Error: Config file not found at {config_path}")
    sys.exit(1)

# Create a temporary directory for output
temp_dir = tempfile.mkdtemp()
print(f"Using temporary directory: {temp_dir}")

# Define output files
flamegraph_svg = os.path.join(os.getcwd(), "kf_gins_flamegraph.svg")
cpu_profile = os.path.join(temp_dir, "kf_gins_cpu_profile.svg")

# Run py-spy to generate flame graph
print("Generating flame graph...")
try:
    # 使用 py-spy 对 KF-GINS 可执行文件进行性能分析
    # 添加 --native 参数以支持C++程序分析
    result = subprocess.run(
        [
            "py-spy", "record",
            "--rate", "100",          # 采样频率：每秒 100 次
            "--duration", "30",       # 采样时长：30 秒
            "--output", flamegraph_svg,  # 输出 SVG 火焰图
            "--format", "flamegraph",    # 指定输出格式
            "--native",                # 支持分析本地（C++）程序
            "--",
            executable_path, config_path  # 被分析的程序及其配置
        ],
        check=True,
        capture_output=True,
        text=True
    )
    print(f"Flame graph generated successfully: {flamegraph_svg}")
    print("\nPerformance analysis complete!")
    print("You can open the flame graph in a web browser to analyze performance bottlenecks.")
    print("Look for wide or tall function calls in the flame graph - these are the performance bottlenecks.")
    print("\nKey functions to look for:")
    print("- GIEngine::insPropagation")
    print("- INSMech::insMech")
    print("- INSMech::velUpdate")
    print("- INSMech::posUpdate")
    print("- INSMech::attUpdate")
    print("- writeNavResult")
    print("- writeSTD")
except subprocess.CalledProcessError as e:
    print(f"Error generating flame graph: {e}")
    print(f"Exit code: {e.returncode}")
    print(f"Output: {e.stdout}")
    print(f"Error output: {e.stderr}")
finally:
    # Clean up temporary directory
    shutil.rmtree(temp_dir)
    print(f"Cleaned up temporary directory: {temp_dir}")
