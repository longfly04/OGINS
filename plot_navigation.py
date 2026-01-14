#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

"""
KF-GINS导航轨迹可视化脚本

该脚本用于读取KF-GINS生成的导航结果文件，并可视化导航轨迹

使用方法：
python plot_navigation.py -f /path/to/KF_GINS_Navresult.nav

或者
python plot_navigation.py --file /path/to/KF_GINS_Navresult.nav
"""

def parse_args():
    """
    解析命令行参数
    """
    parser = argparse.ArgumentParser(description='KF-GINS导航轨迹可视化')
    parser.add_argument('-f', '--file', type=str, required=True, help='导航结果文件路径')
    parser.add_argument('-o', '--output', type=str, default=None, help='输出图像文件路径')
    parser.add_argument('--show', action='store_true', help='显示图像')
    return parser.parse_args()

def read_nav_file(file_path):
    """
    读取导航结果文件
    
    参数：
    file_path: 导航结果文件路径
    
    返回：
    time: 时间序列
    lat: 纬度序列（度）
    lon: 经度序列（度）
    alt: 高程序列（米）
    vel: 速度序列（米/秒）
    euler: 欧拉角序列（度）
    """
    # 检查文件是否存在
    if not os.path.exists(file_path):
        print(f"错误：文件 {file_path} 不存在")
        exit(1)
    
    # 读取文件
    data = np.loadtxt(file_path)
    
    # 提取数据
    time = data[:, 1]  # 时间
    lat = data[:, 2]   # 纬度（度）
    lon = data[:, 3]   # 经度（度）
    alt = data[:, 4]   # 高程（米）
    vel = data[:, 5:8] # 速度（北、东、下，米/秒）
    euler = data[:, 8:11] # 欧拉角（横滚、俯仰、航向，度）
    
    return time, lat, lon, alt, vel, euler

def plot_2d_trajectory(lon, lat, alt, output_path=None, show=False):
    """
    绘制2D导航轨迹
    
    参数：
    lon: 经度序列（度）
    lat: 纬度序列（度）
    alt: 高程序列（米）
    output_path: 输出图像文件路径
    show: 是否显示图像
    """
    plt.figure(figsize=(10, 8))
    
    # 绘制轨迹
    plt.scatter(lon, lat, c=alt, cmap='jet', s=5, alpha=0.8)
    plt.plot(lon, lat, 'k-', linewidth=0.5, alpha=0.5)
    
    # 添加颜色条
    cbar = plt.colorbar()
    cbar.set_label('高程（米）', fontsize=12)
    
    # 设置标题和标签
    plt.title('KF-GINS 2D导航轨迹', fontsize=14)
    plt.xlabel('经度（度）', fontsize=12)
    plt.ylabel('纬度（度）', fontsize=12)
    
    # 设置坐标轴刻度
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    
    # 添加网格
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"2D轨迹图像已保存到 {output_path}")
    
    # 显示图像
    if show:
        plt.show()
    
    plt.close()

def plot_3d_trajectory(lon, lat, alt, output_path=None, show=False):
    """
    绘制3D导航轨迹
    
    参数：
    lon: 经度序列（度）
    lat: 纬度序列（度）
    alt: 高程序列（米）
    output_path: 输出图像文件路径
    show: 是否显示图像
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制轨迹
    sc = ax.scatter(lon, lat, alt, c=alt, cmap='jet', s=5, alpha=0.8)
    ax.plot(lon, lat, alt, 'k-', linewidth=0.5, alpha=0.5)
    
    # 添加颜色条
    cbar = fig.colorbar(sc)
    cbar.set_label('高程（米）', fontsize=12)
    
    # 设置标题和标签
    ax.set_title('KF-GINS 3D导航轨迹', fontsize=14)
    ax.set_xlabel('经度（度）', fontsize=12)
    ax.set_ylabel('纬度（度）', fontsize=12)
    ax.set_zlabel('高程（米）', fontsize=12)
    
    # 设置坐标轴刻度
    ax.tick_params(axis='both', which='major', labelsize=10)
    
    # 调整视角
    ax.view_init(elev=30, azim=45)
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"3D轨迹图像已保存到 {output_path}")
    
    # 显示图像
    if show:
        plt.show()
    
    plt.close()

def plot_altitude_profile(time, alt, output_path=None, show=False):
    """
    绘制高程剖面图
    
    参数：
    time: 时间序列
    alt: 高程序列（米）
    output_path: 输出图像文件路径
    show: 是否显示图像
    """
    plt.figure(figsize=(10, 6))
    
    # 绘制高程剖面
    plt.plot(time, alt, 'b-', linewidth=1.5, alpha=0.8)
    
    # 设置标题和标签
    plt.title('KF-GINS 高程剖面', fontsize=14)
    plt.xlabel('时间（秒）', fontsize=12)
    plt.ylabel('高程（米）', fontsize=12)
    
    # 设置坐标轴刻度
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    
    # 添加网格
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"高程剖面图像已保存到 {output_path}")
    
    # 显示图像
    if show:
        plt.show()
    
    plt.close()

def plot_velocity_profile(time, vel, output_path=None, show=False):
    """
    绘制速度剖面图
    
    参数：
    time: 时间序列
    vel: 速度序列（北、东、下，米/秒）
    output_path: 输出图像文件路径
    show: 是否显示图像
    """
    plt.figure(figsize=(10, 6))
    
    # 绘制速度分量
    plt.plot(time, vel[:, 0], 'r-', linewidth=1.5, alpha=0.8, label='北向速度')
    plt.plot(time, vel[:, 1], 'g-', linewidth=1.5, alpha=0.8, label='东向速度')
    plt.plot(time, vel[:, 2], 'b-', linewidth=1.5, alpha=0.8, label='垂向速度')
    
    # 设置标题和标签
    plt.title('KF-GINS 速度剖面', fontsize=14)
    plt.xlabel('时间（秒）', fontsize=12)
    plt.ylabel('速度（米/秒）', fontsize=12)
    
    # 设置坐标轴刻度
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    
    # 添加图例
    plt.legend(fontsize=10)
    
    # 添加网格
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"速度剖面图像已保存到 {output_path}")
    
    # 显示图像
    if show:
        plt.show()
    
    plt.close()

def main():
    """
    主函数
    """
    # 解析命令行参数
    args = parse_args()
    
    # 读取导航文件
    print(f"正在读取导航结果文件：{args.file}")
    time, lat, lon, alt, vel, euler = read_nav_file(args.file)
    
    print(f"成功读取 {len(time)} 个导航数据点")
    print(f"时间范围：{time[0]} - {time[-1]} 秒")
    print(f"位置范围：")
    print(f"  纬度：{min(lat):.6f} - {max(lat):.6f} 度")
    print(f"  经度：{min(lon):.6f} - {max(lon):.6f} 度")
    print(f"  高程：{min(alt):.2f} - {max(alt):.2f} 米")
    
    # 绘制2D轨迹
    output_2d = args.output.replace('.png', '_2d.png') if args.output else None
    plot_2d_trajectory(lon, lat, alt, output_2d, args.show)
    
    # 绘制3D轨迹
    output_3d = args.output.replace('.png', '_3d.png') if args.output else None
    plot_3d_trajectory(lon, lat, alt, output_3d, args.show)
    
    # 绘制高程剖面
    output_alt = args.output.replace('.png', '_alt.png') if args.output else None
    plot_altitude_profile(time, alt, output_alt, args.show)
    
    # 绘制速度剖面
    output_vel = args.output.replace('.png', '_vel.png') if args.output else None
    plot_velocity_profile(time, vel, output_vel, args.show)
    
    print("可视化完成！")

if __name__ == '__main__':
    main()
