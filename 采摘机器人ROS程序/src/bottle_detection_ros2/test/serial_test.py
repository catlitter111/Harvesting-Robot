import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QFrame, QLabel, QPushButton, QComboBox, QSlider,
    QLineEdit, QCheckBox, QGroupBox, QScrollArea, QSplitter,
    QMessageBox, QFormLayout, QSpacerItem, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QObject
from PySide6.QtGui import QFont, QPalette, QColor, QPainter, QBrush, QPen, QPixmap
import serial
import serial.tools.list_ports
import struct
import time
from collections import deque
import numpy as np
import platform
import warnings
import math
import os

# 忽略字体警告
warnings.filterwarnings('ignore', category=UserWarning)

# Ubuntu系统串口设备配置
SERIAL_DEVICE = "/dev/ttyS3"  # 硬编码的串口设备，可根据实际情况修改
# 常见的Ubuntu串口设备：
# /dev/ttyUSB0, /dev/ttyUSB1 (USB转串口设备)
# /dev/ttyACM0, /dev/ttyACM1 (Arduino等设备)
# /dev/ttyS0, /dev/ttyS1 (传统串口)


class MaterialCard(QFrame):
    """Material Design风格的卡片组件"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameStyle(QFrame.Box)
        self.setLineWidth(1)
        self.setStyleSheet("""
            QFrame {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                margin: 2px;
            }
        """)
        self.setContentsMargins(0, 0, 0, 0)


class MaterialButton(QPushButton):
    """Material Design风格的按钮"""
    def __init__(self, text="", style="primary", parent=None):
        super().__init__(text, parent)
        
        # Material Design颜色方案
        colors = {
            "primary": {"bg": "#1976D2", "hover": "#1565C0", "pressed": "#0D47A1"},
            "secondary": {"bg": "#424242", "hover": "#303030", "pressed": "#212121"},
            "success": {"bg": "#388E3C", "hover": "#2E7D32", "pressed": "#1B5E20"},
            "danger": {"bg": "#D32F2F", "hover": "#C62828", "pressed": "#B71C1C"},
            "warning": {"bg": "#F57C00", "hover": "#E65100", "pressed": "#BF360C"},
            "fab": {"bg": "#FF4081", "hover": "#F50057", "pressed": "#C51162"}
        }
        
        color_scheme = colors.get(style, colors["primary"])
        
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {color_scheme["bg"]};
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 10pt;
            }}
            QPushButton:hover {{
                background-color: {color_scheme["hover"]};
            }}
            QPushButton:pressed {{
                background-color: {color_scheme["pressed"]};
            }}
        """)
        
        self.setCursor(Qt.PointingHandCursor)


class SerialThread(QThread):
    """串口接收线程"""
    packet_received = Signal(int, list)  # cmd, data
    packet_count_updated = Signal(int)
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
        
    def run(self):
        packet_state = 'WAIT_HEADER1'
        packet_data = []
        packet_cmd = 0
        packet_len = 0
        packet_checksum = 0
        packet_count = 0
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    
                    for byte in data:
                        if packet_state == 'WAIT_HEADER1':
                            if byte == 0xAA:
                                packet_state = 'WAIT_HEADER2'
                                
                        elif packet_state == 'WAIT_HEADER2':
                            if byte == 0x55:
                                packet_state = 'WAIT_CMD'
                            else:
                                packet_state = 'WAIT_HEADER1'
                                
                        elif packet_state == 'WAIT_CMD':
                            packet_cmd = byte
                            packet_state = 'WAIT_LEN'
                            
                        elif packet_state == 'WAIT_LEN':
                            packet_len = byte
                            packet_data = []
                            packet_checksum = packet_cmd + packet_len
                            if packet_len > 0:
                                packet_state = 'WAIT_DATA'
                            else:
                                packet_state = 'WAIT_CHECK'
                                
                        elif packet_state == 'WAIT_DATA':
                            packet_data.append(byte)
                            packet_checksum += byte
                            if len(packet_data) >= packet_len:
                                packet_state = 'WAIT_CHECK'
                                
                        elif packet_state == 'WAIT_CHECK':
                            if byte == (packet_checksum & 0xFF):
                                packet_count += 1
                                self.packet_received.emit(packet_cmd, packet_data)
                                self.packet_count_updated.emit(packet_count)
                            packet_state = 'WAIT_HEADER1'
                            
            except Exception as e:
                print(f"Receive error: {e}")
                
            self.msleep(1)
            
    def stop(self):
        self.running = False
        self.wait()


class SimpleChartWidget(QWidget):
    """简化的图表组件，避免matplotlib Qt后端问题"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 400)
        
        # 数据存储
        self.time_data = []
        self.fr_actual = []
        self.fr_target = []
        self.rl_actual = []
        self.rl_target = []
        
        # 图表参数
        self.max_points = 200
        self.x_range = 20  # 显示最近20秒的数据
        self.y_range = 200  # Y轴范围 -100到100
        
        # 颜色设置
        self.colors = {
            'background': QColor(250, 250, 250),
            'grid': QColor(224, 224, 224),
            'fr_actual': QColor(25, 118, 210),
            'fr_target': QColor(25, 118, 210, 120),
            'rl_actual': QColor(255, 64, 129),
            'rl_target': QColor(255, 64, 129, 120),
            'text': QColor(117, 117, 117),
            'axis': QColor(224, 224, 224)
        }
        
        # 边距
        self.margin = 60
        
    def add_data(self, time_val, fr_a, fr_t, rl_a, rl_t):
        """添加新数据点"""
        self.time_data.append(time_val)
        self.fr_actual.append(fr_a)
        self.fr_target.append(fr_t)
        self.rl_actual.append(rl_a)
        self.rl_target.append(rl_t)
        
        # 保持数据点数量限制
        if len(self.time_data) > self.max_points:
            self.time_data = self.time_data[-self.max_points:]
            self.fr_actual = self.fr_actual[-self.max_points:]
            self.fr_target = self.fr_target[-self.max_points:]
            self.rl_actual = self.rl_actual[-self.max_points:]
            self.rl_target = self.rl_target[-self.max_points:]
        
        self.update()
        
    def clear_data(self):
        """清除所有数据"""
        self.time_data.clear()
        self.fr_actual.clear()
        self.fr_target.clear()
        self.rl_actual.clear()
        self.rl_target.clear()
        self.update()
        
    def paintEvent(self, event):
        """绘制图表"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 获取绘制区域
        rect = self.rect()
        chart_rect = rect.adjusted(self.margin, self.margin, -self.margin, -self.margin)
        
        # 绘制背景
        painter.fillRect(rect, self.colors['background'])
        
        # 绘制网格和坐标轴
        self.draw_grid(painter, chart_rect)
        
        # 绘制数据线
        if len(self.time_data) > 1:
            self.draw_data_lines(painter, chart_rect)
        
        # 绘制图例
        self.draw_legend(painter, rect)
        
    def draw_grid(self, painter, chart_rect):
        """绘制网格和坐标轴"""
        painter.setPen(QPen(self.colors['grid'], 1))
        
        # 绘制垂直网格线
        for i in range(6):
            x = chart_rect.left() + (chart_rect.width() * i / 5)
            painter.drawLine(int(x), chart_rect.top(), int(x), chart_rect.bottom())
            
        # 绘制水平网格线
        for i in range(6):
            y = chart_rect.top() + (chart_rect.height() * i / 5)
            painter.drawLine(chart_rect.left(), int(y), chart_rect.right(), int(y))
        
        # 绘制坐标轴
        painter.setPen(QPen(self.colors['axis'], 2))
        painter.drawRect(chart_rect)
        
        # 绘制标签
        painter.setPen(QPen(self.colors['text'], 1))
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        
        # Y轴标签
        for i in range(6):
            y = chart_rect.top() + (chart_rect.height() * i / 5)
            value = 100 - (i * 40)  # 从100到-100
            painter.drawText(10, int(y + 5), f"{value}")
        
        # X轴标签
        if self.time_data:
            current_time = self.time_data[-1] if self.time_data else 0
            for i in range(6):
                x = chart_rect.left() + (chart_rect.width() * i / 5)
                time_val = current_time - self.x_range + (i * self.x_range / 5)
                painter.drawText(int(x - 15), chart_rect.bottom() + 20, f"{time_val:.1f}s")
        
        # 坐标轴标题
        painter.drawText(chart_rect.left(), chart_rect.bottom() + 40, "Time (s)")
        painter.save()
        painter.translate(20, chart_rect.center().y())
        painter.rotate(-90)
        painter.drawText(-30, 0, "Speed")
        painter.restore()
        
    def draw_data_lines(self, painter, chart_rect):
        """绘制数据线"""
        if not self.time_data or len(self.time_data) < 2:
            return
            
        # 计算时间范围
        current_time = self.time_data[-1]
        time_start = max(0, current_time - self.x_range)
        time_end = current_time
        
        # 数据映射函数
        def map_x(time_val):
            ratio = (time_val - time_start) / self.x_range if self.x_range > 0 else 0
            return chart_rect.left() + ratio * chart_rect.width()
            
        def map_y(speed_val):
            ratio = (100 - speed_val) / 200  # 将-100到100映射到0-1
            return chart_rect.top() + ratio * chart_rect.height()
        
        # 筛选显示范围内的数据
        visible_indices = []
        for i, t in enumerate(self.time_data):
            if time_start <= t <= time_end:
                visible_indices.append(i)
        
        if len(visible_indices) < 2:
            return
        
        # 绘制目标值线（虚线效果）
        pen_target = QPen(self.colors['fr_target'], 1)
        pen_target.setStyle(Qt.DashLine)
        painter.setPen(pen_target)
        
        # FR目标值线
        for i in range(len(visible_indices) - 1):
            idx1, idx2 = visible_indices[i], visible_indices[i + 1]
            x1, y1 = map_x(self.time_data[idx1]), map_y(self.fr_target[idx1])
            x2, y2 = map_x(self.time_data[idx2]), map_y(self.fr_target[idx2])
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
        
        # RL目标值线
        pen_target.setColor(self.colors['rl_target'])
        painter.setPen(pen_target)
        for i in range(len(visible_indices) - 1):
            idx1, idx2 = visible_indices[i], visible_indices[i + 1]
            x1, y1 = map_x(self.time_data[idx1]), map_y(self.rl_target[idx1])
            x2, y2 = map_x(self.time_data[idx2]), map_y(self.rl_target[idx2])
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
        
        # 绘制实际值线（实线）
        painter.setPen(QPen(self.colors['fr_actual'], 2))
        
        # FR实际值线
        for i in range(len(visible_indices) - 1):
            idx1, idx2 = visible_indices[i], visible_indices[i + 1]
            x1, y1 = map_x(self.time_data[idx1]), map_y(self.fr_actual[idx1])
            x2, y2 = map_x(self.time_data[idx2]), map_y(self.fr_actual[idx2])
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
        
        # RL实际值线
        painter.setPen(QPen(self.colors['rl_actual'], 2))
        for i in range(len(visible_indices) - 1):
            idx1, idx2 = visible_indices[i], visible_indices[i + 1]
            x1, y1 = map_x(self.time_data[idx1]), map_y(self.rl_actual[idx1])
            x2, y2 = map_x(self.time_data[idx2]), map_y(self.rl_actual[idx2])
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
    
    def draw_legend(self, painter, rect):
        """绘制图例"""
        legend_x = rect.width() - 150
        legend_y = 20
        
        painter.setPen(QPen(self.colors['text'], 1))
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        
        # FR Actual
        painter.setPen(QPen(self.colors['fr_actual'], 2))
        painter.drawLine(legend_x, legend_y, legend_x + 20, legend_y)
        painter.setPen(QPen(self.colors['text'], 1))
        painter.drawText(legend_x + 25, legend_y + 5, "FR Actual")
        
        # FR Target
        legend_y += 20
        pen = QPen(self.colors['fr_target'], 1)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.drawLine(legend_x, legend_y, legend_x + 20, legend_y)
        painter.setPen(QPen(self.colors['text'], 1))
        painter.drawText(legend_x + 25, legend_y + 5, "FR Target")
        
        # RL Actual
        legend_y += 20
        painter.setPen(QPen(self.colors['rl_actual'], 2))
        painter.drawLine(legend_x, legend_y, legend_x + 20, legend_y)
        painter.setPen(QPen(self.colors['text'], 1))
        painter.drawText(legend_x + 25, legend_y + 5, "RL Actual")
        
        # RL Target
        legend_y += 20
        pen = QPen(self.colors['rl_target'], 1)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.drawLine(legend_x, legend_y, legend_x + 20, legend_y)
        painter.setPen(QPen(self.colors['text'], 1))
        painter.drawText(legend_x + 25, legend_y + 5, "RL Target")


class PIDAnalyzer:
    """PID控制器分析和调试建议类"""
    
    def __init__(self):
        self.analysis_interval = 2.0  # 分析间隔时间（秒）
        self.settling_tolerance = 0.05  # 稳态误差容限（5%）
        self.overshoot_threshold = 0.1  # 超调阈值（10%）
        self.oscillation_threshold = 3  # 振荡检测阈值
        
    def analyze_pid_performance(self, motor_data):
        """分析PID性能并生成建议"""
        if len(motor_data['errors']) < 10:
            return "数据收集中...", {}, "info"
            
        current_time = time.time()
        
        # 计算分析统计数据
        stats = self._calculate_stats(motor_data)
        motor_data['analysis_stats'] = stats
        
        # 检测目标值变化
        if len(motor_data['targets']) >= 2:
            if abs(motor_data['targets'][-1] - motor_data['targets'][-2]) > 1.0:
                motor_data['last_target_change'] = current_time
                motor_data['settling_started'] = current_time
                motor_data['overshoot_detected'] = False
                motor_data['oscillation_count'] = 0
        
        # 生成分析建议
        recommendation, status = self._generate_recommendation(motor_data, stats, current_time)
        
        return recommendation, stats, status
    
    def _calculate_stats(self, motor_data):
        """计算统计数据"""
        if len(motor_data['errors']) < 5:
            return {}
            
        errors = list(motor_data['errors'])
        outputs = list(motor_data['outputs'])
        targets = list(motor_data['targets'])
        actuals = list(motor_data['actuals'])
        
        # 基础统计
        current_error = abs(errors[-1]) if errors else 0
        avg_error = np.mean([abs(e) for e in errors[-10:]]) if len(errors) >= 10 else 0
        error_std = np.std(errors[-10:]) if len(errors) >= 10 else 0
        
        # 稳态误差（最近5个数据点的平均误差）
        steady_state_error = np.mean([abs(e) for e in errors[-5:]]) if len(errors) >= 5 else 0
        
        # 超调检测
        target = targets[-1] if targets else 0
        max_actual = max(actuals[-10:]) if len(actuals) >= 10 else 0
        min_actual = min(actuals[-10:]) if len(actuals) >= 10 else 0
        
        overshoot = 0
        if target > 0:
            overshoot = max(0, (max_actual - target) / target * 100)
        elif target < 0:
            overshoot = max(0, (target - min_actual) / abs(target) * 100)
            
        # 振荡检测（误差符号变化次数）
        oscillations = 0
        if len(errors) >= 10:
            for i in range(len(errors) - 9, len(errors) - 1):
                if errors[i] * errors[i + 1] < 0:  # 符号变化
                    oscillations += 1
        
        # 响应时间估算
        response_time = 0
        if motor_data['settling_started'] and len(actuals) >= 5:
            time_elapsed = time.time() - motor_data['settling_started']
            if steady_state_error < self.settling_tolerance * abs(target):
                response_time = time_elapsed
        
        return {
            'current_error': current_error,
            'avg_error': avg_error,
            'error_std': error_std,
            'steady_state_error': steady_state_error,
            'overshoot': overshoot,
            'oscillations': oscillations,
            'response_time': response_time,
            'target': target,
            'actual': actuals[-1] if actuals else 0,
            'output': outputs[-1] if outputs else 0
        }
    
    def _generate_recommendation(self, motor_data, stats, current_time):
        """生成PID调整建议"""
        recommendations = []
        status = "info"
        
        # 检查是否有足够的数据
        if not stats or len(motor_data['errors']) < 10:
            return "收集数据中，请等待更多数据...", "info"
        
        target = stats.get('target', 0)
        steady_state_error = stats.get('steady_state_error', 0)
        overshoot = stats.get('overshoot', 0)
        oscillations = stats.get('oscillations', 0)
        error_std = stats.get('error_std', 0)
        
        # 如果目标为0，不进行分析
        if abs(target) < 1.0:
            return "目标速度为0，暂停分析", "info"
        
        # 稳态误差分析
        error_percentage = (steady_state_error / abs(target)) * 100
        
        if error_percentage > 10:  # 稳态误差大于10%
            if error_percentage > 20:
                recommendations.append("⚠️ 稳态误差过大(%.1f%%)，建议增大Ki参数" % error_percentage)
                status = "warning"
            else:
                recommendations.append("💡 稳态误差较大(%.1f%%)，可适量增大Ki参数" % error_percentage)
        elif error_percentage < 2:
            recommendations.append("✅ 稳态误差良好(%.1f%%)" % error_percentage)
        
        # 超调分析
        if overshoot > 15:
            recommendations.append("⚠️ 超调过大(%.1f%%)，建议减小Kp或增大Kd" % overshoot)
            status = "warning"
        elif overshoot > 5:
            recommendations.append("💡 有轻微超调(%.1f%%)，可适量减小Kp" % overshoot)
        elif overshoot < 2:
            recommendations.append("✅ 超调控制良好(%.1f%%)" % overshoot)
        
        # 振荡分析
        if oscillations >= 5:
            recommendations.append("⚠️ 检测到振荡，建议减小Kp和Ki参数")
            status = "warning"
        elif oscillations >= 3:
            recommendations.append("💡 轻微振荡，建议微调Kd参数")
        elif error_std < 1.0:
            recommendations.append("✅ 系统稳定，无振荡")
        
        # 响应速度分析
        if len(motor_data['actuals']) >= 10:
            time_since_change = current_time - motor_data['last_target_change']
            if time_since_change > 3.0 and error_percentage > 5:
                recommendations.append("⏱️ 响应较慢，建议适量增大Kp参数")
                if status == "info":
                    status = "warning"
        
        # 综合建议
        if not recommendations:
            recommendations.append("✅ PID参数调节良好")
        
        # 参数建议
        current_kp = 1.5  # 这里应该从界面获取当前参数
        current_ki = 0.2
        current_kd = 0.1
        
        param_suggestions = []
        
        if error_percentage > 15:
            new_ki = min(current_ki * 1.2, 2.0)
            param_suggestions.append(f"建议Ki: {current_ki:.2f} → {new_ki:.2f}")
        
        if overshoot > 10:
            new_kp = max(current_kp * 0.8, 0.5)
            param_suggestions.append(f"建议Kp: {current_kp:.2f} → {new_kp:.2f}")
        
        if oscillations >= 4:
            new_kd = min(current_kd * 1.3, 0.5)
            param_suggestions.append(f"建议Kd: {current_kd:.2f} → {new_kd:.2f}")
        
        if param_suggestions:
            recommendations.extend(param_suggestions)
        
        return "\n".join(recommendations), status


class RobotControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Robot Control System - Ubuntu")
        self.setGeometry(100, 100, 1400, 900)
        
        # Material Design配色
        self.colors = {
            'primary': '#1976D2',
            'primary_dark': '#1565C0',
            'primary_light': '#42A5F5',
            'secondary': '#FF4081',
            'background': '#FAFAFA',
            'surface': '#FFFFFF',
            'error': '#D32F2F',
            'success': '#388E3C',
            'warning': '#F57C00',
            'text_primary': '#212121',
            'text_secondary': '#757575',
            'divider': '#BDBDBD'
        }
        
        # 设置样式
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {self.colors['background']};
            }}
        """)
        
        # 串口相关变量
        self.serial_port = None
        self.serial_thread = None
        
        # 数据存储
        self.plot_data = {
            'time': deque(maxlen=200),
            'fr_actual': deque(maxlen=200),
            'fr_target': deque(maxlen=200),
            'rl_actual': deque(maxlen=200),
            'rl_target': deque(maxlen=200)
        }
        
        self.plot_start_time = None
        self.last_update_time = 0
        
        # PID参数
        self.pid_params = {
            'front_right': {'kp': 1.5, 'ki': 0.2, 'kd': 0.1},
            'rear_left': {'kp': 1.5, 'ki': 0.2, 'kd': 0.1}
        }
        
        # 当前状态
        self.current_speed_fr = 0.0
        self.current_speed_rl = 0.0
        self.target_speed_fr = 0.0
        self.target_speed_rl = 0.0
        self.pid_output_fr = 0.0
        self.pid_output_rl = 0.0
        
        # PID分析数据
        self.pid_analysis_data = {
            'fr': {
                'errors': deque(maxlen=50),
                'outputs': deque(maxlen=50), 
                'targets': deque(maxlen=50),
                'actuals': deque(maxlen=50),
                'timestamps': deque(maxlen=50),
                'last_target_change': time.time(),
                'settling_started': None,
                'overshoot_detected': False,
                'oscillation_count': 0,
                'last_recommendation': '',
                'analysis_stats': {}
            },
            'rl': {
                'errors': deque(maxlen=50),
                'outputs': deque(maxlen=50),
                'targets': deque(maxlen=50),
                'actuals': deque(maxlen=50),
                'timestamps': deque(maxlen=50),
                'last_target_change': time.time(),
                'settling_started': None,
                'overshoot_detected': False,
                'oscillation_count': 0,
                'last_recommendation': '',
                'analysis_stats': {}
            }
        }
        
        # 定时器
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)
        
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.request_pid_status)
        
        # PID分析器
        self.pid_analyzer = PIDAnalyzer()
        
        # PID分析定时器
        self.analysis_timer = QTimer()
        self.analysis_timer.timeout.connect(self.update_pid_analysis)
        self.analysis_timer.start(1000)  # 每秒更新一次分析
        
        # 创建主界面
        self.create_ui()
        
    def create_ui(self):
        """创建Material Design风格的UI"""
        # 中央控件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 顶部工具栏
        self.create_toolbar(main_layout)
        
        # 主内容区域
        content_widget = QWidget()
        content_widget.setStyleSheet(f"background-color: {self.colors['background']};")
        content_layout = QHBoxLayout(content_widget)
        content_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.addWidget(content_widget)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        content_layout.addWidget(splitter)
        
        # 左侧控制面板
        left_widget = self.create_left_panel()
        splitter.addWidget(left_widget)
        
        # 右侧内容
        right_widget = self.create_right_panel()
        splitter.addWidget(right_widget)
        
        # 设置分割比例
        splitter.setSizes([450, 950])
        
    def create_toolbar(self, parent_layout):
        """创建顶部工具栏"""
        toolbar = QWidget()
        toolbar.setFixedHeight(60)
        toolbar.setStyleSheet(f"""
            QWidget {{
                background-color: {self.colors['primary']};
            }}
        """)
        
        layout = QHBoxLayout(toolbar)
        layout.setContentsMargins(20, 15, 20, 15)
        
        # 应用标题
        title_label = QLabel("STM32 Robot Control System - Ubuntu")
        title_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 18pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }
        """)
        layout.addWidget(title_label)
        
        # 弹性空间
        layout.addStretch()
        
        # 显示当前串口设备
        device_label = QLabel(f"Device: {SERIAL_DEVICE}")
        device_label.setStyleSheet("""
            QLabel {
                color: #E3F2FD;
                font-size: 12pt;
                font-family: 'Roboto', Arial, sans-serif;
            }
        """)
        layout.addWidget(device_label)
        
        # 右侧状态指示器
        self.connection_indicator = QLabel("● Disconnected")
        self.connection_indicator.setStyleSheet("""
            QLabel {
                color: #FF5252;
                font-size: 12pt;
                font-family: 'Roboto', Arial, sans-serif;
            }
        """)
        layout.addWidget(self.connection_indicator)
        
        parent_layout.addWidget(toolbar)
        
    def create_left_panel(self):
        """创建左侧控制面板"""
        # 滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setStyleSheet(f"""
            QScrollArea {{
                border: none;
                background-color: {self.colors['background']};
            }}
        """)
        
        # 滚动内容
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(15)
        
        # 串口连接卡片
        self.create_connection_card(scroll_layout)
        
        # 运动控制卡片
        self.create_motion_card(scroll_layout)
        
        # PID控制卡片
        self.create_pid_card(scroll_layout)
        
        # 弹性空间
        scroll_layout.addStretch()
        
        scroll_area.setWidget(scroll_content)
        scroll_area.setFixedWidth(450)
        
        return scroll_area
        
    def create_right_panel(self):
        """创建右侧面板"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(15)
        
        # 实时图表卡片
        self.create_chart_card(layout)
        
        # PID调试建议卡片
        self.create_pid_analysis_card(layout)
        
        # 状态显示卡片
        self.create_status_card(layout)
        
        return widget
        
    def create_connection_card(self, parent_layout):
        """创建串口连接卡片"""
        card = MaterialCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题
        title = QLabel("Serial Connection")
        title.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }}
        """)
        layout.addWidget(title)
        
        # 显示当前配置的串口设备
        device_info_layout = QVBoxLayout()
        
        device_label = QLabel(f"Device: {SERIAL_DEVICE}")
        device_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 12pt;
                font-weight: bold;
                background-color: {self.colors['primary']};
                color: white;
                padding: 8px;
                border-radius: 4px;
            }}
        """)
        device_info_layout.addWidget(device_label)
        
        # 检查设备是否存在
        device_exists = os.path.exists(SERIAL_DEVICE)
        status_text = "✓ Device found" if device_exists else "✗ Device not found"
        status_color = self.colors['success'] if device_exists else self.colors['error']
        
        status_label = QLabel(status_text)
        status_label.setStyleSheet(f"""
            QLabel {{
                color: {status_color};
                font-size: 10pt;
                font-weight: bold;
                padding: 4px;
            }}
        """)
        device_info_layout.addWidget(status_label)
        
        layout.addLayout(device_info_layout)
        
        # 波特率选择
        baud_layout = QHBoxLayout()
        baud_label = QLabel("Baudrate:")
        baud_label.setStyleSheet(f"color: {self.colors['text_secondary']}; font-size: 10pt;")
        
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "230400"])
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setMinimumWidth(150)
        
        baud_layout.addWidget(baud_label)
        baud_layout.addWidget(self.baud_combo)
        baud_layout.addStretch()
        layout.addLayout(baud_layout)
        
        # 连接按钮
        self.connect_btn = MaterialButton("CONNECT", "primary")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        # 添加设备权限提示
        permission_label = QLabel("Note: Make sure you have permission to access the serial device.\nYou may need to add your user to the 'dialout' group:\nsudo usermod -a -G dialout $USER")
        permission_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_secondary']};
                font-size: 9pt;
                font-style: italic;
                padding: 8px;
                background-color: #FFF3E0;
                border-radius: 4px;
                border: 1px solid #FFE0B2;
            }}
        """)
        permission_label.setWordWrap(True)
        layout.addWidget(permission_label)
        
        parent_layout.addWidget(card)
        
    def create_pid_analysis_card(self, parent_layout):
        """创建PID分析建议卡片"""
        card = MaterialCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题栏
        header_layout = QHBoxLayout()
        
        title = QLabel("PID Tuning Assistant")
        title.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }}
        """)
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        # 自动调参按钮
        auto_tune_btn = MaterialButton("AUTO TUNE", "secondary")
        auto_tune_btn.clicked.connect(self.auto_tune_pid)
        header_layout.addWidget(auto_tune_btn)
        
        layout.addLayout(header_layout)
        
        # 分析结果显示区域
        analysis_layout = QHBoxLayout()
        
        # 前右电机分析
        fr_analysis_group = QGroupBox("Front Right Motor Analysis")
        fr_analysis_group.setStyleSheet(f"""
            QGroupBox::title {{
                color: {self.colors['primary']};
                font-weight: bold;
            }}
        """)
        fr_analysis_layout = QVBoxLayout(fr_analysis_group)
        
        # 前右电机统计信息
        self.fr_stats_label = QLabel("等待数据...")
        self.fr_stats_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_secondary']};
                font-size: 9pt;
                background-color: #F5F5F5;
                padding: 8px;
                border-radius: 4px;
                font-family: monospace;
            }}
        """)
        fr_analysis_layout.addWidget(self.fr_stats_label)
        
        # 前右电机建议
        self.fr_recommendation_label = QLabel("分析中...")
        self.fr_recommendation_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 10pt;
                background-color: #E3F2FD;
                padding: 12px;
                border-radius: 4px;
                border-left: 4px solid {self.colors['primary']};
                line-height: 1.4;
            }}
        """)
        self.fr_recommendation_label.setWordWrap(True)
        fr_analysis_layout.addWidget(self.fr_recommendation_label)
        
        analysis_layout.addWidget(fr_analysis_group)
        
        # 后左电机分析
        rl_analysis_group = QGroupBox("Rear Left Motor Analysis")
        rl_analysis_group.setStyleSheet(f"""
            QGroupBox::title {{
                color: {self.colors['primary']};
                font-weight: bold;
            }}
        """)
        rl_analysis_layout = QVBoxLayout(rl_analysis_group)
        
        # 后左电机统计信息
        self.rl_stats_label = QLabel("等待数据...")
        self.rl_stats_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_secondary']};
                font-size: 9pt;
                background-color: #F5F5F5;
                padding: 8px;
                border-radius: 4px;
                font-family: monospace;
            }}
        """)
        rl_analysis_layout.addWidget(self.rl_stats_label)
        
        # 后左电机建议
        self.rl_recommendation_label = QLabel("分析中...")
        self.rl_recommendation_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 10pt;
                background-color: #E8F5E8;
                padding: 12px;
                border-radius: 4px;
                border-left: 4px solid {self.colors['success']};
                line-height: 1.4;
            }}
        """)
        self.rl_recommendation_label.setWordWrap(True)
        rl_analysis_layout.addWidget(self.rl_recommendation_label)
        
        analysis_layout.addWidget(rl_analysis_group)
        
        layout.addLayout(analysis_layout)
        
        # 全局建议区域
        global_advice_label = QLabel("💡 全局调参建议")
        global_advice_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 12pt;
                font-weight: bold;
                margin-top: 10px;
            }}
        """)
        layout.addWidget(global_advice_label)
        
        self.global_advice_label = QLabel("开始运动控制以获取调参建议...")
        self.global_advice_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 10pt;
                background-color: #FFF3E0;
                padding: 12px;
                border-radius: 4px;
                border-left: 4px solid {self.colors['warning']};
                line-height: 1.4;
            }}
        """)
        self.global_advice_label.setWordWrap(True)
        layout.addWidget(self.global_advice_label)
        
    def create_motion_card(self, parent_layout):
        """创建运动控制卡片"""
        card = MaterialCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题
        title = QLabel("Motion Control")
        title.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }}
        """)
        layout.addWidget(title)
        
        # 方向控制按钮
        control_layout = QGridLayout()
        
        # 创建方向按钮
        button_style = f"""
            QPushButton {{
                background-color: {self.colors['primary']};
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 16pt;
                min-width: 50px;
                min-height: 50px;
            }}
            QPushButton:hover {{
                background-color: {self.colors['primary_dark']};
            }}
        """
        
        stop_button_style = f"""
            QPushButton {{
                background-color: {self.colors['error']};
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 16pt;
                min-width: 50px;
                min-height: 50px;
            }}
            QPushButton:hover {{
                background-color: #C62828;
            }}
        """
        
        # 前进
        forward_btn = QPushButton("▲")
        forward_btn.setStyleSheet(button_style)
        forward_btn.clicked.connect(lambda: self.send_direction_cmd(0x00))
        control_layout.addWidget(forward_btn, 0, 1)
        
        # 左转
        left_btn = QPushButton("◄")
        left_btn.setStyleSheet(button_style)
        left_btn.clicked.connect(lambda: self.send_direction_cmd(0x02))
        control_layout.addWidget(left_btn, 1, 0)
        
        # 停止
        stop_btn = QPushButton("■")
        stop_btn.setStyleSheet(stop_button_style)
        stop_btn.clicked.connect(lambda: self.send_direction_cmd(0x04))
        control_layout.addWidget(stop_btn, 1, 1)
        
        # 右转
        right_btn = QPushButton("►")
        right_btn.setStyleSheet(button_style)
        right_btn.clicked.connect(lambda: self.send_direction_cmd(0x03))
        control_layout.addWidget(right_btn, 1, 2)
        
        # 后退
        backward_btn = QPushButton("▼")
        backward_btn.setStyleSheet(button_style)
        backward_btn.clicked.connect(lambda: self.send_direction_cmd(0x01))
        control_layout.addWidget(backward_btn, 2, 1)
        
        control_widget = QWidget()
        control_widget.setLayout(control_layout)
        layout.addWidget(control_widget, alignment=Qt.AlignCenter)
        
        # 速度控制
        speed_label = QLabel("Speed Control")
        speed_label.setStyleSheet(f"color: {self.colors['text_secondary']}; font-size: 12pt; margin-top: 20px;")
        layout.addWidget(speed_label)
        
        speed_layout = QHBoxLayout()
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        
        self.speed_value_label = QLabel("50%")
        self.speed_value_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['primary']};
                font-weight: bold;
                font-size: 12pt;
                min-width: 40px;
            }}
        """)
        
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_value_label)
        layout.addLayout(speed_layout)
        
        parent_layout.addWidget(card)
        
    def create_pid_card(self, parent_layout):
        """创建PID控制卡片"""
        card = MaterialCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题和开关
        header_layout = QHBoxLayout()
        
        title = QLabel("PID Control")
        title.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }}
        """)
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        # PID开关
        self.pid_checkbox = QCheckBox("Enable")
        self.pid_checkbox.setChecked(True)
        self.pid_checkbox.stateChanged.connect(self.toggle_pid)
        header_layout.addWidget(self.pid_checkbox)
        
        layout.addLayout(header_layout)
        
        # 前右电机PID
        fr_group = QGroupBox("Front Right Motor (TIM2)")
        self.create_pid_inputs(fr_group, 'front_right')
        layout.addWidget(fr_group)
        
        # 后左电机PID
        rl_group = QGroupBox("Rear Left Motor (TIM4)")
        self.create_pid_inputs(rl_group, 'rear_left')
        layout.addWidget(rl_group)
        
        # 操作按钮
        apply_btn = MaterialButton("APPLY PARAMETERS", "primary")
        apply_btn.clicked.connect(self.apply_pid_params)
        layout.addWidget(apply_btn)
        
        button_layout = QHBoxLayout()
        reset_btn = MaterialButton("RESET", "secondary")
        reset_btn.clicked.connect(self.reset_pid)
        test_btn = MaterialButton("TEST", "secondary")
        test_btn.clicked.connect(self.test_communication)
        
        button_layout.addWidget(reset_btn)
        button_layout.addWidget(test_btn)
        layout.addLayout(button_layout)
        
        parent_layout.addWidget(card)
        
    def create_pid_inputs(self, parent_group, motor_name):
        """创建PID参数输入框"""
        layout = QFormLayout(parent_group)
        
        # 创建输入框
        if motor_name == 'front_right':
            self.fr_kp_entry = QLineEdit(str(self.pid_params[motor_name]['kp']))
            self.fr_ki_entry = QLineEdit(str(self.pid_params[motor_name]['ki']))
            self.fr_kd_entry = QLineEdit(str(self.pid_params[motor_name]['kd']))
            
            layout.addRow("Kp:", self.fr_kp_entry)
            layout.addRow("Ki:", self.fr_ki_entry)
            layout.addRow("Kd:", self.fr_kd_entry)
        else:
            self.rl_kp_entry = QLineEdit(str(self.pid_params[motor_name]['kp']))
            self.rl_ki_entry = QLineEdit(str(self.pid_params[motor_name]['ki']))
            self.rl_kd_entry = QLineEdit(str(self.pid_params[motor_name]['kd']))
            
            layout.addRow("Kp:", self.rl_kp_entry)
            layout.addRow("Ki:", self.rl_ki_entry)
            layout.addRow("Kd:", self.rl_kd_entry)
        
    def create_chart_card(self, parent_layout):
        """创建实时图表卡片"""
        card = MaterialCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题栏
        header_layout = QHBoxLayout()
        
        title = QLabel("Real-time Speed Chart")
        title.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }}
        """)
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        clear_btn = MaterialButton("CLEAR", "secondary")
        clear_btn.clicked.connect(self.clear_chart)
        header_layout.addWidget(clear_btn)
        
        layout.addLayout(header_layout)
        
        # 创建简化图表
        self.chart_widget = SimpleChartWidget()
        layout.addWidget(self.chart_widget)
        
        parent_layout.addWidget(card)
        
    def create_status_card(self, parent_layout):
        """创建状态显示卡片"""
        card = MaterialCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题
        title = QLabel("System Status")
        title.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Roboto', Arial, sans-serif;
            }}
        """)
        layout.addWidget(title)
        
        # 状态网格
        status_layout = QGridLayout()
        
        # 前右电机状态
        fr_group = self.create_status_group("Front Right Motor")
        self.fr_speed_label = self.create_status_item(fr_group, "Speed", "0.0")
        self.fr_target_label = self.create_status_item(fr_group, "Target", "0.0")
        self.fr_output_label = self.create_status_item(fr_group, "PID Out", "0.0")
        status_layout.addWidget(fr_group, 0, 0)
        
        # 后左电机状态
        rl_group = self.create_status_group("Rear Left Motor")
        self.rl_speed_label = self.create_status_item(rl_group, "Speed", "0.0")
        self.rl_target_label = self.create_status_item(rl_group, "Target", "0.0")
        self.rl_output_label = self.create_status_item(rl_group, "PID Out", "0.0")
        status_layout.addWidget(rl_group, 0, 1)
        
        # 通信状态
        comm_group = self.create_status_group("Communication")
        self.comm_status_label = self.create_status_item(comm_group, "Status", "Waiting...")
        self.packet_count_label = self.create_status_item(comm_group, "Packets", "0")
        status_layout.addWidget(comm_group, 0, 2)
        
        layout.addLayout(status_layout)
        parent_layout.addWidget(card)
        
    def create_status_group(self, title):
        """创建状态组"""
        group = QGroupBox(title)
        group.setStyleSheet(f"""
            QGroupBox::title {{
                color: {self.colors['primary']};
                font-weight: bold;
            }}
        """)
        layout = QVBoxLayout(group)
        return group
        
    def create_status_item(self, parent, label, value):
        """创建状态项"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 2, 0, 2)
        
        label_widget = QLabel(f"{label}:")
        label_widget.setStyleSheet(f"color: {self.colors['text_secondary']}; font-size: 9pt;")
        
        value_widget = QLabel(value)
        value_widget.setStyleSheet(f"color: {self.colors['text_primary']}; font-weight: bold; font-size: 9pt;")
        
        layout.addWidget(label_widget)
        layout.addWidget(value_widget)
        layout.addStretch()
        
        parent.layout().addWidget(widget)
        return value_widget
        
    # ========== 串口通信相关方法 ==========
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """连接串口"""
        try:
            # 检查设备是否存在
            if not os.path.exists(SERIAL_DEVICE):
                QMessageBox.critical(self, "Device Not Found", 
                                   f"Serial device {SERIAL_DEVICE} not found!\n\n"
                                   f"Please check:\n"
                                   f"1. Device is connected\n"
                                   f"2. Device path is correct\n"
                                   f"3. You have permission to access the device\n\n"
                                   f"You may need to run:\n"
                                   f"sudo chmod 666 {SERIAL_DEVICE}\n"
                                   f"or add your user to dialout group:\n"
                                   f"sudo usermod -a -G dialout $USER")
                return
            
            baud = int(self.baud_combo.currentText())
            
            self.serial_port = serial.Serial(SERIAL_DEVICE, baud, timeout=0.1)
            
            # 启动接收线程
            self.serial_thread = SerialThread(self.serial_port)
            self.serial_thread.packet_received.connect(self.process_packet)
            self.serial_thread.packet_count_updated.connect(self.update_packet_count)
            self.serial_thread.start()
            
            # 更新UI
            self.connect_btn.setText("DISCONNECT")
            self.connect_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {self.colors['error']};
                    color: white;
                    border: none;
                    border-radius: 4px;
                    padding: 8px 16px;
                    font-weight: bold;
                    font-size: 10pt;
                }}
                QPushButton:hover {{
                    background-color: #C62828;
                }}
            """)
            
            self.connection_indicator.setText("● Connected")
            self.connection_indicator.setStyleSheet("""
                QLabel {
                    color: #4CAF50;
                    font-size: 12pt;
                    font-family: 'Roboto', Arial, sans-serif;
                }
            """)
            
            # 清除图表
            self.clear_chart()
            
            # 启动定时器
            self.status_timer.start(100)
            
            QMessageBox.information(self, "Success", f"Connected to {SERIAL_DEVICE}")
            
        except serial.SerialException as e:
            error_msg = f"Failed to connect to {SERIAL_DEVICE}\n\nError: {str(e)}\n\n"
            
            if "Permission denied" in str(e):
                error_msg += ("Permission denied. Please try:\n"
                            f"sudo chmod 666 {SERIAL_DEVICE}\n"
                            "or add your user to dialout group:\n"
                            "sudo usermod -a -G dialout $USER\n"
                            "Then logout and login again.")
            elif "No such file or directory" in str(e):
                error_msg += ("Device not found. Please check:\n"
                            "1. Device is connected\n"
                            "2. Correct device path\n"
                            "3. Driver is loaded")
            
            QMessageBox.critical(self, "Connection Failed", error_msg)
            
        except Exception as e:
            QMessageBox.critical(self, "Connection Failed", f"Unexpected error: {str(e)}")
            
    def disconnect(self):
        """断开串口连接"""
        if self.serial_thread:
            self.serial_thread.stop()
            
        if self.serial_port:
            self.serial_port.close()
            
        self.status_timer.stop()
        
        self.connect_btn.setText("CONNECT")
        self.connect_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors['primary']};
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 10pt;
            }}
            QPushButton:hover {{
                background-color: {self.colors['primary_dark']};
            }}
        """)
        
        self.connection_indicator.setText("● Disconnected")
        self.connection_indicator.setStyleSheet("""
            QLabel {
                color: #FF5252;
                font-size: 12pt;
                font-family: 'Roboto', Arial, sans-serif;
            }
        """)
        
    def process_packet(self, cmd, data):
        """处理接收到的数据包"""
        if cmd == 0x08:  # CMD_PID_GET_STATUS
            if len(data) >= 7:
                motor_id = data[0]
                
                def convert_signed_16bit(high_byte, low_byte):
                    value = (high_byte << 8) | low_byte
                    if value & 0x8000:
                        value = value - 0x10000
                    return value / 10.0
                
                current_speed = convert_signed_16bit(data[1], data[2])
                target_speed = convert_signed_16bit(data[3], data[4])
                pid_output = convert_signed_16bit(data[5], data[6])
                
                current_time = time.time()
                if self.plot_start_time is None:
                    self.plot_start_time = current_time
                    
                elapsed_time = current_time - self.plot_start_time
                
                if motor_id == 0:  # 前右电机
                    self.current_speed_fr = current_speed
                    self.target_speed_fr = target_speed
                    self.pid_output_fr = pid_output
                    self.update_fr_display()
                    
                    # 收集PID分析数据
                    self.collect_pid_data('fr', current_speed, target_speed, pid_output)
                    
                elif motor_id == 1:  # 后左电机
                    self.current_speed_rl = current_speed
                    self.target_speed_rl = target_speed
                    self.pid_output_rl = pid_output
                    self.update_rl_display()
                    
                    # 收集PID分析数据  
                    self.collect_pid_data('rl', current_speed, target_speed, pid_output)
                    
                self.update_plot_data(elapsed_time)
                self.update_comm_status("Active")
                
    def send_packet(self, cmd, data):
        """发送数据包到下位机"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "Please connect serial port first")
            return
            
        packet = bytearray()
        packet.append(0xAA)  # Header 1
        packet.append(0x55)  # Header 2
        packet.append(cmd)   # Command
        packet.append(len(data))  # Length
        
        checksum = cmd + len(data)
        for byte in data:
            packet.append(byte)
            checksum += byte
            
        packet.append(checksum & 0xFF)  # Checksum
        
        try:
            self.serial_port.write(packet)
        except Exception as e:
            QMessageBox.critical(self, "Send Failed", str(e))
            
    # ========== 控制命令相关方法 ==========
    
    def send_direction_cmd(self, direction):
        """发送方向控制命令"""
        speed = self.speed_slider.value()
        data = [direction, speed]
        self.send_packet(0x01, data)
        
    def on_speed_change(self, value):
        """速度滑块变化时的回调"""
        self.speed_value_label.setText(f"{value}%")
        
        if self.serial_port and self.serial_port.is_open:
            self.send_packet(0x02, [value])
            
    def toggle_pid(self, state):
        """切换PID控制状态"""
        enable = 1 if state == Qt.Checked else 0
        self.send_packet(0x06, [enable])
        
    def apply_pid_params(self):
        """应用PID参数"""
        try:
            # 获取前右电机参数
            fr_kp = float(self.fr_kp_entry.text())
            fr_ki = float(self.fr_ki_entry.text())
            fr_kd = float(self.fr_kd_entry.text())
            
            # 获取后左电机参数
            rl_kp = float(self.rl_kp_entry.text())
            rl_ki = float(self.rl_ki_entry.text())
            rl_kd = float(self.rl_kd_entry.text())
            
            # 发送前右电机PID参数
            kp_fr = int(fr_kp * 100)
            ki_fr = int(fr_ki * 100)
            kd_fr = int(fr_kd * 100)
            
            data_fr = [0,  # Motor ID
                      (kp_fr >> 8) & 0xFF, kp_fr & 0xFF,
                      (ki_fr >> 8) & 0xFF, ki_fr & 0xFF,
                      (kd_fr >> 8) & 0xFF, kd_fr & 0xFF]
                      
            self.send_packet(0x07, data_fr)
            
            # 延时后发送后左电机参数
            QTimer.singleShot(100, lambda: self.send_rl_pid_params(rl_kp, rl_ki, rl_kd))
            
        except ValueError:
            QMessageBox.critical(self, "Error", "Please enter valid numeric values")
            
    def send_rl_pid_params(self, kp, ki, kd):
        """发送后左电机PID参数"""
        kp_rl = int(kp * 100)
        ki_rl = int(ki * 100)
        kd_rl = int(kd * 100)
        
        data_rl = [1,  # Motor ID
                  (kp_rl >> 8) & 0xFF, kp_rl & 0xFF,
                  (ki_rl >> 8) & 0xFF, ki_rl & 0xFF,
                  (kd_rl >> 8) & 0xFF, kd_rl & 0xFF]
                  
        self.send_packet(0x07, data_rl)
        
        QMessageBox.information(self, "Success", "PID parameters updated")
        
    def reset_pid(self):
        """重置PID参数为默认值"""
        # 更新输入框
        self.fr_kp_entry.setText("1.5")
        self.fr_ki_entry.setText("0.2")
        self.fr_kd_entry.setText("0.1")
        
        self.rl_kp_entry.setText("1.5")
        self.rl_ki_entry.setText("0.2")
        self.rl_kd_entry.setText("0.1")
        
        # 应用到STM32
        self.apply_pid_params()
        
    def test_communication(self):
        """测试与STM32的通信"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "Please connect serial port first")
            return
            
        self.send_packet(0x06, [1])  # 启用PID
        QTimer.singleShot(100, lambda: self.send_packet(0x08, [0]))  # 请求前右电机状态
        QTimer.singleShot(200, lambda: self.send_packet(0x08, [1]))  # 请求后左电机状态
        
        QMessageBox.information(self, "Test", "Communication test started")
        
    def request_pid_status(self):
        """请求PID状态"""
        if self.serial_port and self.serial_port.is_open:
            self.send_packet(0x08, [0])  # 请求前右电机状态
            QTimer.singleShot(50, lambda: self.send_packet(0x08, [1]))  # 请求后左电机状态
            
    # ========== 显示更新相关方法 ==========
    
    def update_plot_data(self, elapsed_time):
        """更新图表数据"""
        if elapsed_time - self.last_update_time < 0.05:
            return
            
        self.last_update_time = elapsed_time
        
        self.plot_data['time'].append(elapsed_time)
        self.plot_data['fr_actual'].append(self.current_speed_fr)
        self.plot_data['fr_target'].append(self.target_speed_fr)
        self.plot_data['rl_actual'].append(self.current_speed_rl)
        self.plot_data['rl_target'].append(self.target_speed_rl)
        
    def update_plot(self):
        """更新实时曲线图"""
        if len(self.plot_data['time']) > 1:
            # 更新简化图表
            self.chart_widget.add_data(
                self.plot_data['time'][-1] if self.plot_data['time'] else 0,
                self.plot_data['fr_actual'][-1] if self.plot_data['fr_actual'] else 0,
                self.plot_data['fr_target'][-1] if self.plot_data['fr_target'] else 0,
                self.plot_data['rl_actual'][-1] if self.plot_data['rl_actual'] else 0,
                self.plot_data['rl_target'][-1] if self.plot_data['rl_target'] else 0
            )
        
    def clear_chart(self):
        """清除图表数据"""
        for key in self.plot_data:
            self.plot_data[key].clear()
            
        self.plot_start_time = None
        self.last_update_time = 0
        
        self.chart_widget.clear_data()
        
    def update_fr_display(self):
        """更新前右电机显示"""
        self.fr_speed_label.setText(f"{self.current_speed_fr:.1f}")
        self.fr_target_label.setText(f"{self.target_speed_fr:.1f}")
        self.fr_output_label.setText(f"{self.pid_output_fr:.1f}")
        
    def update_rl_display(self):
        """更新后左电机显示"""
        self.rl_speed_label.setText(f"{self.current_speed_rl:.1f}")
        self.rl_target_label.setText(f"{self.target_speed_rl:.1f}")
        self.rl_output_label.setText(f"{self.pid_output_rl:.1f}")
        
    def update_comm_status(self, status):
        """更新通信状态"""
        self.comm_status_label.setText(status)
        
    def update_packet_count(self, count):
        """更新接收包数"""
        self.packet_count_label.setText(str(count))
        
    def collect_pid_data(self, motor_key, actual, target, output):
        """收集PID分析数据"""
        current_time = time.time()
        motor_data = self.pid_analysis_data[motor_key]
        
        error = target - actual
        
        motor_data['errors'].append(error)
        motor_data['outputs'].append(output)
        motor_data['targets'].append(target)
        motor_data['actuals'].append(actual)
        motor_data['timestamps'].append(current_time)
        
    def update_pid_analysis(self):
        """更新PID分析和建议"""
        # 分析前右电机
        fr_recommendation, fr_stats, fr_status = self.pid_analyzer.analyze_pid_performance(
            self.pid_analysis_data['fr']
        )
        
        # 分析后左电机
        rl_recommendation, rl_stats, rl_status = self.pid_analyzer.analyze_pid_performance(
            self.pid_analysis_data['rl']
        )
        
        # 更新前右电机显示
        if fr_stats:
            fr_stats_text = f"""误差: {fr_stats.get('current_error', 0):.2f}
稳态误差: {fr_stats.get('steady_state_error', 0):.2f}
超调: {fr_stats.get('overshoot', 0):.1f}%
振荡: {fr_stats.get('oscillations', 0)}次
目标: {fr_stats.get('target', 0):.1f}
实际: {fr_stats.get('actual', 0):.1f}"""
            self.fr_stats_label.setText(fr_stats_text)
        
        # 更新建议颜色
        fr_color = self.get_status_color(fr_status)
        self.fr_recommendation_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 10pt;
                background-color: {fr_color['bg']};
                padding: 12px;
                border-radius: 4px;
                border-left: 4px solid {fr_color['border']};
                line-height: 1.4;
            }}
        """)
        self.fr_recommendation_label.setText(fr_recommendation)
        
        # 更新后左电机显示
        if rl_stats:
            rl_stats_text = f"""误差: {rl_stats.get('current_error', 0):.2f}
稳态误差: {rl_stats.get('steady_state_error', 0):.2f}
超调: {rl_stats.get('overshoot', 0):.1f}%
振荡: {rl_stats.get('oscillations', 0)}次
目标: {rl_stats.get('target', 0):.1f}
实际: {rl_stats.get('actual', 0):.1f}"""
            self.rl_stats_label.setText(rl_stats_text)
        
        # 更新建议颜色
        rl_color = self.get_status_color(rl_status)
        self.rl_recommendation_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 10pt;
                background-color: {rl_color['bg']};
                padding: 12px;
                border-radius: 4px;
                border-left: 4px solid {rl_color['border']};
                line-height: 1.4;
            }}
        """)
        self.rl_recommendation_label.setText(rl_recommendation)
        
        # 生成全局建议
        self.update_global_advice(fr_stats, rl_stats, fr_status, rl_status)
        
    def get_status_color(self, status):
        """根据状态获取对应的颜色"""
        if status == "warning":
            return {"bg": "#FFEBEE", "border": self.colors['error']}
        elif status == "good":
            return {"bg": "#E8F5E8", "border": self.colors['success']}
        else:
            return {"bg": "#E3F2FD", "border": self.colors['primary']}
            
    def update_global_advice(self, fr_stats, rl_stats, fr_status, rl_status):
        """更新全局调参建议"""
        advice_parts = []
        
        # 检查两个电机的整体状态
        if fr_status == "warning" or rl_status == "warning":
            advice_parts.append("🔧 检测到性能问题，建议按照上述建议调整PID参数")
        
        # 对比两个电机的性能
        if fr_stats and rl_stats:
            fr_error = fr_stats.get('steady_state_error', 0)
            rl_error = rl_stats.get('steady_state_error', 0)
            
            if abs(fr_error - rl_error) > 2.0:
                advice_parts.append("⚖️ 两个电机性能差异较大，建议分别调整参数")
            
            fr_overshoot = fr_stats.get('overshoot', 0)
            rl_overshoot = rl_stats.get('overshoot', 0)
            
            if fr_overshoot > 10 and rl_overshoot > 10:
                advice_parts.append("📉 两个电机都有超调，建议同时减小Kp参数")
            
            if fr_stats.get('oscillations', 0) > 3 and rl_stats.get('oscillations', 0) > 3:
                advice_parts.append("〰️ 检测到系统振荡，建议降低Kp和Ki参数")
        
        # 通用建议
        advice_parts.append("💡 调参原则：\n• Kp影响响应速度和稳态误差\n• Ki消除稳态误差但可能引起振荡\n• Kd减少超调和振荡")
        
        if not advice_parts:
            advice_parts.append("✅ 系统运行良好，PID参数调节合适")
        
        self.global_advice_label.setText("\n\n".join(advice_parts))
        
    def auto_tune_pid(self):
        """自动调参功能"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "请先连接串口")
            return
            
        # 基于当前分析结果自动调整参数
        fr_stats = self.pid_analysis_data['fr'].get('analysis_stats', {})
        rl_stats = self.pid_analysis_data['rl'].get('analysis_stats', {})
        
        if not fr_stats or not rl_stats:
            QMessageBox.information(self, "Info", "数据不足，请先运行一段时间收集数据")
            return
        
        # 获取当前PID参数
        try:
            current_fr_kp = float(self.fr_kp_entry.text())
            current_fr_ki = float(self.fr_ki_entry.text())
            current_fr_kd = float(self.fr_kd_entry.text())
            
            current_rl_kp = float(self.rl_kp_entry.text())
            current_rl_ki = float(self.rl_ki_entry.text())
            current_rl_kd = float(self.rl_kd_entry.text())
        except ValueError:
            QMessageBox.critical(self, "Error", "请检查当前PID参数是否为有效数值")
            return
        
        # 自动调整算法
        new_fr_params = self.calculate_auto_tune_params(fr_stats, current_fr_kp, current_fr_ki, current_fr_kd)
        new_rl_params = self.calculate_auto_tune_params(rl_stats, current_rl_kp, current_rl_ki, current_rl_kd)
        
        # 显示建议的参数
        suggestion_text = f"""自动调参建议：

前右电机：
Kp: {current_fr_kp:.2f} → {new_fr_params[0]:.2f}
Ki: {current_fr_ki:.2f} → {new_fr_params[1]:.2f}  
Kd: {current_fr_kd:.2f} → {new_fr_params[2]:.2f}

后左电机：
Kp: {current_rl_kp:.2f} → {new_rl_params[0]:.2f}
Ki: {current_rl_ki:.2f} → {new_rl_params[1]:.2f}
Kd: {current_rl_kd:.2f} → {new_rl_params[2]:.2f}

是否应用这些参数？"""
        
        reply = QMessageBox.question(self, "自动调参", suggestion_text, 
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # 应用新参数
            self.fr_kp_entry.setText(f"{new_fr_params[0]:.2f}")
            self.fr_ki_entry.setText(f"{new_fr_params[1]:.2f}")
            self.fr_kd_entry.setText(f"{new_fr_params[2]:.2f}")
            
            self.rl_kp_entry.setText(f"{new_rl_params[0]:.2f}")
            self.rl_ki_entry.setText(f"{new_rl_params[1]:.2f}")
            self.rl_kd_entry.setText(f"{new_rl_params[2]:.2f}")
            
            # 发送到STM32
            self.apply_pid_params()
            
            # 清空分析数据以重新开始分析
            for motor_key in ['fr', 'rl']:
                for data_key in ['errors', 'outputs', 'targets', 'actuals', 'timestamps']:
                    self.pid_analysis_data[motor_key][data_key].clear()
            
            QMessageBox.information(self, "成功", "自动调参完成，开始重新分析性能")
            
    def calculate_auto_tune_params(self, stats, current_kp, current_ki, current_kd):
        """计算自动调参参数"""
        new_kp = current_kp
        new_ki = current_ki  
        new_kd = current_kd
        
        target = abs(stats.get('target', 1))
        if target < 0.1:  # 避免除零错误
            return (new_kp, new_ki, new_kd)
            
        steady_error_pct = (stats.get('steady_state_error', 0) / target) * 100
        overshoot = stats.get('overshoot', 0)
        oscillations = stats.get('oscillations', 0)
        
        # 调整Ki - 基于稳态误差
        if steady_error_pct > 15:
            new_ki = min(current_ki * 1.3, 2.0)  # 增大Ki
        elif steady_error_pct < 2:
            new_ki = max(current_ki * 0.9, 0.05)  # 适量减小Ki
            
        # 调整Kp - 基于超调和响应速度
        if overshoot > 15:
            new_kp = max(current_kp * 0.8, 0.5)  # 减小Kp
        elif overshoot < 2 and steady_error_pct > 10:
            new_kp = min(current_kp * 1.2, 5.0)  # 增大Kp
            
        # 调整Kd - 基于振荡
        if oscillations >= 4:
            new_kd = min(current_kd * 1.4, 1.0)  # 增大Kd
        elif oscillations == 0 and overshoot > 5:
            new_kd = min(current_kd * 1.1, 0.5)  # 适量增大Kd
            
        return (new_kp, new_ki, new_kd)
        
    def closeEvent(self, event):
        """关闭窗口时的处理"""
        if self.serial_thread:
            self.serial_thread.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    window = RobotControlGUI()
    window.show()
    
    sys.exit(app.exec())