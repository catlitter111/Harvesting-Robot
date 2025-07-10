#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试图像查看器
用于查看和分析瓶子检测系统保存的调试图像
"""

import os
import cv2
import glob
import argparse
from pathlib import Path
import time

def get_debug_directory():
    """获取调试图像目录"""
    return os.path.expanduser("~/bottle_detection_debug_images")

def list_image_sets(debug_dir):
    """列出所有图像集合（按时间戳分组）"""
    if not os.path.exists(debug_dir):
        print(f"调试目录不存在: {debug_dir}")
        return []
    
    # 查找所有截取信息文件
    info_files = glob.glob(os.path.join(debug_dir, "cropped_bottles", "crop_info_*.txt"))
    
    image_sets = []
    for info_file in info_files:
        filename = os.path.basename(info_file)
        # 提取时间戳和请求ID
        parts = filename.replace("crop_info_", "").replace(".txt", "")
        timestamp_parts = parts.split("_")
        if len(timestamp_parts) >= 3:
            date_part = timestamp_parts[0]
            time_part = timestamp_parts[1]
            ms_part = timestamp_parts[2]
            request_id = "_".join(timestamp_parts[3:])
            
            timestamp_str = f"{date_part}_{time_part}_{ms_part}"
            
            # 检查相关文件是否存在
            base_pattern = f"{timestamp_str}_{request_id}"
            
            full_frame_file = os.path.join(debug_dir, "full_frames", f"full_frame_{base_pattern}.jpg")
            cropped_file = os.path.join(debug_dir, "cropped_bottles", f"bottle_crop_{base_pattern}.jpg")
            annotated_file = os.path.join(debug_dir, "annotated_frames", f"annotated_{base_pattern}.jpg")
            
            image_sets.append({
                'timestamp': timestamp_str,
                'request_id': request_id,
                'base_pattern': base_pattern,
                'info_file': info_file,
                'full_frame': full_frame_file if os.path.exists(full_frame_file) else None,
                'cropped': cropped_file if os.path.exists(cropped_file) else None,
                'annotated': annotated_file if os.path.exists(annotated_file) else None,
                'time_readable': format_timestamp(timestamp_str)
            })
    
    # 按时间戳排序（最新的在前）
    image_sets.sort(key=lambda x: x['timestamp'], reverse=True)
    return image_sets

def format_timestamp(timestamp_str):
    """格式化时间戳为可读格式"""
    try:
        parts = timestamp_str.split("_")
        if len(parts) >= 3:
            date_part = parts[0]
            time_part = parts[1]
            ms_part = parts[2]
            
            # 格式化日期和时间
            year = date_part[:4]
            month = date_part[4:6]
            day = date_part[6:8]
            
            hour = time_part[:2]
            minute = time_part[2:4]
            second = time_part[4:6]
            
            return f"{year}-{month}-{day} {hour}:{minute}:{second}.{ms_part}"
    except:
        pass
    return timestamp_str

def read_info_file(info_file):
    """读取截取信息文件"""
    try:
        with open(info_file, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception as e:
        return f"读取信息文件失败: {e}"

def display_image_set(image_set, show_details=True):
    """显示一组图像"""
    print(f"\n{'='*60}")
    print(f"时间: {image_set['time_readable']}")
    print(f"请求ID: {image_set['request_id']}")
    print(f"{'='*60}")
    
    if show_details and image_set['info_file']:
        print("\n详细信息:")
        print("-" * 40)
        info_content = read_info_file(image_set['info_file'])
        print(info_content)
        print("-" * 40)
    
    # 显示图像
    images_to_show = []
    window_names = []
    
    if image_set['full_frame']:
        img = cv2.imread(image_set['full_frame'])
        if img is not None:
            images_to_show.append(img)
            window_names.append("Complete Frame")
    
    if image_set['annotated']:
        img = cv2.imread(image_set['annotated'])
        if img is not None:
            images_to_show.append(img)
            window_names.append("Annotated Frame")
    
    if image_set['cropped']:
        img = cv2.imread(image_set['cropped'])
        if img is not None:
            # 放大截取的图像以便查看
            h, w = img.shape[:2]
            if h < 300 or w < 300:
                scale = max(300 / h, 300 / w)
                new_h, new_w = int(h * scale), int(w * scale)
                img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
            images_to_show.append(img)
            window_names.append("Cropped Bottle")
    
    if not images_to_show:
        print("没有找到图像文件")
        return
    
    # 显示图像窗口
    for img, name in zip(images_to_show, window_names):
        cv2.imshow(name, img)
    
    print(f"\n显示 {len(images_to_show)} 个图像窗口")
    print("按任意键继续，ESC退出...")
    
    key = cv2.waitKey(0) & 0xFF
    cv2.destroyAllWindows()
    
    return key != 27  # ESC键退出

def main():
    parser = argparse.ArgumentParser(description='瓶子检测调试图像查看器')
    parser.add_argument('--dir', type=str, help='调试图像目录路径（默认: ~/bottle_detection_debug_images）')
    parser.add_argument('--latest', action='store_true', help='只显示最新的图像集')
    parser.add_argument('--no-details', action='store_true', help='不显示详细信息')
    parser.add_argument('--count', type=int, default=10, help='显示的图像集数量（默认10个）')
    
    args = parser.parse_args()
    
    # 确定调试目录
    debug_dir = args.dir if args.dir else get_debug_directory()
    
    print(f"瓶子检测调试图像查看器")
    print(f"调试目录: {debug_dir}")
    print("="*60)
    
    # 获取图像集列表
    image_sets = list_image_sets(debug_dir)
    
    if not image_sets:
        print("没有找到调试图像")
        return
    
    print(f"找到 {len(image_sets)} 个图像集")
    
    if args.latest:
        # 只显示最新的
        image_sets = image_sets[:1]
    else:
        # 显示指定数量
        image_sets = image_sets[:args.count]
    
    print(f"将显示 {len(image_sets)} 个图像集")
    
    # 逐个显示图像集
    for i, image_set in enumerate(image_sets):
        print(f"\n[{i+1}/{len(image_sets)}]")
        
        continue_viewing = display_image_set(image_set, show_details=not args.no_details)
        
        if not continue_viewing:
            print("退出查看")
            break
        
        if i < len(image_sets) - 1:
            print(f"\n准备显示下一个图像集...")
            time.sleep(1)
    
    print("\n查看完成")

def list_command():
    """列出所有可用的图像集"""
    debug_dir = get_debug_directory()
    image_sets = list_image_sets(debug_dir)
    
    if not image_sets:
        print("没有找到调试图像")
        return
    
    print(f"调试目录: {debug_dir}")
    print(f"找到 {len(image_sets)} 个图像集:")
    print("="*80)
    
    for i, image_set in enumerate(image_sets):
        status = []
        if image_set['full_frame']: status.append("完整图像")
        if image_set['annotated']: status.append("标注图像")
        if image_set['cropped']: status.append("截取图像")
        if image_set['info_file']: status.append("详细信息")
        
        print(f"{i+1:3d}. {image_set['time_readable']} | {image_set['request_id']}")
        print(f"     文件: {', '.join(status)}")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc() 