#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SLAM建图节点 - 适配ROS2 Humble
基于slam_toolbox提供实时建图和定位功能
适配四轮小车的建图需求
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String, Bool, Header
from std_srvs.srv import Empty, SetBool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_ros
import json
import time
import os
import threading
import numpy as np
from pathlib import Path
import yaml


class SlamMappingNode(Node):
    """SLAM建图节点"""
    
    def __init__(self):
        super().__init__('slam_mapping_node')
        
        # 声明参数
        self._declare_parameters()
        self._get_parameters()
        
        # 状态变量
        self.mapping_active = False
        self.map_data = None
        self.robot_pose = None
        self.laser_data = None
        self.mapping_start_time = None
        
        # TF相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 创建订阅者
        self._create_subscriptions()
        
        # 创建发布者
        self._create_publishers()
        
        # 创建服务
        self._create_services()
        
        # 创建定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.pose_timer = self.create_timer(0.1, self.update_robot_pose)
        
        # 地图保存路径
        self.map_save_dir = Path.home() / 'agrisage3_maps'
        self.map_save_dir.mkdir(exist_ok=True)
        
        self.get_logger().info(f'SLAM建图节点已启动')
        self.get_logger().info(f'地图保存路径: {self.map_save_dir}')
        
    def _declare_parameters(self):
        """声明ROS参数"""
        self.declare_parameter('slam_enabled', True)
        self.declare_parameter('auto_start_mapping', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'lidar')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('auto_save_map', True)
        self.declare_parameter('save_interval', 30.0)
        self.declare_parameter('map_name_prefix', 'agrisage3_map')
        
    def _get_parameters(self):
        """获取参数值"""
        self.slam_enabled = self.get_parameter('slam_enabled').value
        self.auto_start_mapping = self.get_parameter('auto_start_mapping').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.auto_save_map = self.get_parameter('auto_save_map').value
        self.save_interval = self.get_parameter('save_interval').value
        self.map_name_prefix = self.get_parameter('map_name_prefix').value
        
    def _create_subscriptions(self):
        """创建订阅者"""
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.mapping_control_sub = self.create_subscription(
            String,
            'slam/control',
            self.mapping_control_callback,
            10
        )
        
    def _create_publishers(self):
        """创建发布者"""
        self.slam_status_pub = self.create_publisher(
            String,
            'slam/status',
            10
        )
        
        self.robot_pose_pub = self.create_publisher(
            PoseStamped,
            'slam/robot_pose',
            10
        )
        
        self.map_metadata_pub = self.create_publisher(
            String,
            'slam/map_info',
            10
        )
        
    def _create_services(self):
        """创建服务"""
        self.start_mapping_srv = self.create_service(
            SetBool,
            'slam/start_mapping',
            self.start_mapping_callback
        )
        
        self.save_map_srv = self.create_service(
            Empty,
            'slam/save_map',
            self.save_map_callback
        )
        
    def laser_callback(self, msg):
        """激光雷达数据回调"""
        with self.lock:
            self.laser_data = msg
            
    def map_callback(self, msg):
        """地图数据回调"""
        with self.lock:
            self.map_data = msg
        self.publish_map_info()
        
    def mapping_control_callback(self, msg):
        """建图控制回调"""
        try:
            control_data = json.loads(msg.data)
            command = control_data.get('command', '')
            
            if command == 'start':
                self.start_mapping()
            elif command == 'stop':
                self.stop_mapping()
            elif command == 'save':
                self.save_current_map()
            
        except Exception as e:
            self.get_logger().error(f'解析建图控制命令失败: {e}')
            
    def update_robot_pose(self):
        """更新机器人位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.orientation = transform.transform.rotation
            
            with self.lock:
                self.robot_pose = pose_stamped
                
            self.robot_pose_pub.publish(pose_stamped)
            
        except Exception:
            pass
            
    def start_mapping(self):
        """开始建图"""
        if not self.slam_enabled:
            return False
            
        self.mapping_active = True
        self.mapping_start_time = time.time()
        self.get_logger().info('开始SLAM建图...')
        return True
        
    def stop_mapping(self):
        """停止建图"""
        self.mapping_active = False
        self.get_logger().info('SLAM建图已停止')
        if self.auto_save_map:
            self.save_current_map()
        return True
        
    def save_current_map(self):
        """保存当前地图"""
        if self.map_data is None:
            self.get_logger().warn('没有地图数据可保存')
            return False
            
        try:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            map_name = f'{self.map_name_prefix}_{timestamp}'
            
            map_file = self.map_save_dir / f'{map_name}.yaml'
            pgm_file = self.map_save_dir / f'{map_name}.pgm'
            
            self._save_pgm_map(pgm_file)
            self._save_yaml_metadata(map_file, f'{map_name}.pgm')
            
            self.get_logger().info(f'地图已保存: {map_file}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'保存地图失败: {e}')
            return False
            
    def _save_pgm_map(self, pgm_file):
        """保存PGM格式地图文件"""
        if self.map_data is None:
            return
            
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape((height, width))
        
        pgm_data = np.zeros_like(data, dtype=np.uint8)
        pgm_data[data == 0] = 255      # 空闲区域
        pgm_data[data == 100] = 0      # 占用区域
        pgm_data[data == -1] = 128     # 未知区域
        
        with open(pgm_file, 'wb') as f:
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            f.write(pgm_data.tobytes())
            
    def _save_yaml_metadata(self, yaml_file, pgm_filename):
        """保存YAML元数据文件"""
        if self.map_data is None:
            return
            
        metadata = {
            'image': pgm_filename,
            'resolution': float(self.map_data.info.resolution),
            'origin': [
                float(self.map_data.info.origin.position.x),
                float(self.map_data.info.origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)
            
    def publish_status(self):
        """发布SLAM状态"""
        status = {
            'slam_enabled': self.slam_enabled,
            'mapping_active': self.mapping_active,
            'has_map_data': self.map_data is not None,
            'has_laser_data': self.laser_data is not None,
            'robot_localized': self.robot_pose is not None,
            'timestamp': time.time()
        }
        
        if self.map_data:
            status.update({
                'map_width': self.map_data.info.width,
                'map_height': self.map_data.info.height,
                'map_resolution': self.map_data.info.resolution
            })
            
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.slam_status_pub.publish(msg)
        
    def publish_map_info(self):
        """发布地图信息"""
        if self.map_data is None:
            return
            
        info = {
            'width': self.map_data.info.width,
            'height': self.map_data.info.height,
            'resolution': self.map_data.info.resolution,
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(info, ensure_ascii=False)
        self.map_metadata_pub.publish(msg)
        
    def start_mapping_callback(self, request, response):
        """开始建图服务回调"""
        if request.data:
            success = self.start_mapping()
        else:
            success = self.stop_mapping()
            
        response.success = success
        response.message = '建图控制命令执行完成'
        return response
        
    def save_map_callback(self, request, response):
        """保存地图服务回调"""
        self.save_current_map()
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SlamMappingNode()
        if node.auto_start_mapping:
            node.start_mapping()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 