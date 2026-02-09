#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 usb_cam 摄像头节点和 detect_node 检测节点"""
    
    ld = LaunchDescription()
    
    # 获取 usb_cam 包的共享目录
    usb_cam_dir = get_package_share_directory('usb_cam')
    
    # 摄像头参数文件路径
    param_file = Path(usb_cam_dir) / 'config' / 'params_1.yaml'
    
    # 摄像头 remapping 配置
    camera_remappings = [
        ('image_raw', 'camera1/image_raw'),
        ('image_raw/compressed', 'camera1/image_compressed'),
        ('image_raw/compressedDepth', 'camera1/compressedDepth'),
        ('image_raw/theora', 'camera1/image_raw/theora'),
        ('camera_info', 'camera1/camera_info'),
    ]
    
    # 启动 usb_cam 节点
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',
        name='camera1',
        parameters=[str(param_file)],
        remappings=camera_remappings
    )
    
    # 启动 detect_node
    detect_node = Node(
        package='detect',
        executable='detect_node',
        output='screen',
        name='detect_node'
    )
    
    # 将节点添加到启动描述
    ld.add_action(usb_cam_node)
    ld.add_action(detect_node)
    
    return ld
