#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证重构后模块导入是否正确的测试脚本
"""

import sys
import traceback

def test_imports():
    """测试所有模块的导入"""
    tests = []
    
    # 测试核心视觉模块
    try:
        from bottle_detection_ros2.core.vision.stereo_camera import StereoCamera
        tests.append(("✓", "core.vision.stereo_camera", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "core.vision.stereo_camera", str(e)))
    
    try:
        from bottle_detection_ros2.core.vision.bottle_detector import BottleDetector
        tests.append(("✓", "core.vision.bottle_detector", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "core.vision.bottle_detector", str(e)))
    
    try:
        from bottle_detection_ros2.core.vision.bottle_detector_async import detect_bottle_async
        tests.append(("✓", "core.vision.bottle_detector_async", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "core.vision.bottle_detector_async", str(e)))
    
    # 测试核心处理模块
    try:
        from bottle_detection_ros2.core.processing.bottle_rknn_pool import BottleRKNNPoolExecutor
        tests.append(("✓", "core.processing.bottle_rknn_pool", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "core.processing.bottle_rknn_pool", str(e)))
    
    # 测试核心硬件模块
    try:
        from bottle_detection_ros2.core.hardware.laser_obstacle_avoidance import LaserObstacleAvoidance
        tests.append(("✓", "core.hardware.laser_obstacle_avoidance", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "core.hardware.laser_obstacle_avoidance", str(e)))
    
    # 测试工具模块
    try:
        from bottle_detection_ros2.utils.utils import MedianFilter
        tests.append(("✓", "utils.utils", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "utils.utils", str(e)))
    
    # 测试主包导入
    try:
        import bottle_detection_ros2
        tests.append(("✓", "主包导入", "SUCCESS"))
    except Exception as e:
        tests.append(("✗", "主包导入", str(e)))
    
    # 测试节点模块（不实例化，只测试导入）
    node_modules = [
        "nodes.detection.bottle_detection_node",
        "nodes.detection.bottle_detection_node_async", 
        "nodes.detection.integrated_bottle_detection_node",
        "nodes.control.robot_control_node",
        "nodes.control.servo_control_node",
        "nodes.control.auto_harvest_controller",
        "nodes.communication.websocket_bridge_node",
    ]
    
    for module in node_modules:
        try:
            exec(f"from bottle_detection_ros2.{module} import *")
            tests.append(("✓", module, "SUCCESS"))
        except Exception as e:
            tests.append(("✗", module, str(e)))
    
    # 输出测试结果
    print("="*80)
    print("模块导入验证结果")
    print("="*80)
    
    success_count = 0
    for status, module, result in tests:
        if status == "✓":
            success_count += 1
            print(f"{status} {module:<40} {result}")
        else:
            print(f"{status} {module:<40} FAILED: {result}")
    
    print("="*80)
    print(f"总计: {len(tests)} 个模块, 成功: {success_count}, 失败: {len(tests) - success_count}")
    
    if success_count == len(tests):
        print("🎉 所有模块导入成功！重构完成！")
        return True
    else:
        print("❌ 部分模块导入失败，需要检查import路径")
        return False

if __name__ == "__main__":
    success = test_imports()
    sys.exit(0 if success else 1) 