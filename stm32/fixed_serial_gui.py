import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QFrame, QLabel, QPushButton, QComboBox, QSlider,
    QLineEdit, QCheckBox, QGroupBox, QScrollArea, QSplitter,
    QMessageBox, QFormLayout, QSpacerItem, QSizePolicy, QProgressBar,
    QTextEdit
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QObject, QMutex, QMutexLocker
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
from enum import Enum
from datetime import datetime
import threading

# 忽略字体警告
warnings.filterwarnings('ignore', category=UserWarning)

# Ubuntu系统串口设备配置
SERIAL_DEVICE = "/dev/ttyS3"  # 硬编码的串口设备，可根据实际情况修改


class TuningStatus(Enum):
    """调参状态枚举"""
    IDLE = "idle"
    COLLECTING_DATA = "collecting_data"
    ANALYZING = "analyzing"
    APPLYING_PARAMS = "applying_params"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"


class AutoTuningStateMachine:
    """自动调参状态机"""
    def __init__(self):
        self.current_state = TuningStatus.IDLE
        self.state_mutex = QMutex()
        self.progress = 0
        self.total_steps = 0
        self.current_step = 0
        self.state_history = []
        self.start_time = None
        self.error_count = 0
        self.max_errors = 3
        
    def reset(self):
        """重置状态机"""
        with QMutexLocker(self.state_mutex):
            self.current_state = TuningStatus.IDLE
            self.progress = 0
            self.total_steps = 0
            self.current_step = 0
            self.state_history.clear()
            self.start_time = None
            self.error_count = 0
            
    def transition_to(self, new_state):
        """状态转换"""
        with QMutexLocker(self.state_mutex):
            old_state = self.current_state
            self.current_state = new_state
            self.state_history.append({
                'from': old_state,
                'to': new_state,
                'timestamp': datetime.now()
            })
            return True
            
    def get_state(self):
        """获取当前状态"""
        with QMutexLocker(self.state_mutex):
            return self.current_state
            
    def update_progress(self, step, total):
        """更新进度"""
        with QMutexLocker(self.state_mutex):
            self.current_step = step
            self.total_steps = total
            self.progress = int((step / total) * 100) if total > 0 else 0
            return self.progress
            
    def increment_error(self):
        """增加错误计数"""
        with QMutexLocker(self.state_mutex):
            self.error_count += 1
            return self.error_count >= self.max_errors


class EnhancedPIDTuner:
    """增强的PID调参器"""
    
    def __init__(self):
        # 调参算法参数
        self.ziegler_nichols_enabled = True
        self.cohen_coon_enabled = False
        self.lambda_tuning_enabled = False
        
        # 性能指标权重
        self.weights = {
            'overshoot': 0.3,
            'settling_time': 0.3,
            'steady_error': 0.2,
            'oscillation': 0.2
        }
        
        # 调参约束
        self.param_constraints = {
            'kp': {'min': 0.1, 'max': 10.0, 'step': 0.1},
            'ki': {'min': 0.0, 'max': 5.0, 'step': 0.05},
            'kd': {'min': 0.0, 'max': 2.0, 'step': 0.02}
        }
        
        # 调参历史
        self.tuning_history = []
        self.performance_history = []
        
    def calculate_performance_score(self, stats):
        """计算性能得分"""
        score = 100.0
        
        # 超调惩罚
        overshoot = stats.get('overshoot', 0)
        if overshoot > 20:
            score -= self.weights['overshoot'] * 50
        elif overshoot > 10:
            score -= self.weights['overshoot'] * 30
        elif overshoot > 5:
            score -= self.weights['overshoot'] * 10
            
        # 稳态误差惩罚
        steady_error = stats.get('steady_state_error', 0)
        target = abs(stats.get('target', 1.0))
        if target > 0.1:
            error_pct = (steady_error / target) * 100
            if error_pct > 10:
                score -= self.weights['steady_error'] * 40
            elif error_pct > 5:
                score -= self.weights['steady_error'] * 20
            elif error_pct > 2:
                score -= self.weights['steady_error'] * 10
                
        # 振荡惩罚
        oscillation = stats.get('oscillation_intensity', 0)
        if oscillation > 3.0:
            score -= self.weights['oscillation'] * 40
        elif oscillation > 2.0:
            score -= self.weights['oscillation'] * 25
        elif oscillation > 1.0:
            score -= self.weights['oscillation'] * 10
            
        # 建立时间惩罚（简化评估）
        if not stats.get('is_steady_state', False):
            score -= self.weights['settling_time'] * 20
            
        return max(0, score)
        
    def apply_ziegler_nichols(self, ku, tu):
        """Ziegler-Nichols方法"""
        # PID控制器参数
        kp = 0.6 * ku
        ki = 2.0 * kp / tu
        kd = kp * tu / 8.0
        
        return self._constrain_params(kp, ki, kd)
        
    def apply_cohen_coon(self, k, tau, theta):
        """Cohen-Coon方法"""
        a = k * theta / tau
        
        kp = (1.35 / k) * ((tau / theta) + 0.185)
        ki = kp / (2.5 * theta * (1 + 0.185 * (theta / tau)))
        kd = kp * 0.37 * theta * (1 / (1 + 0.185 * (theta / tau)))
        
        return self._constrain_params(kp, ki, kd)
        
    def iterative_tuning(self, current_params, stats, iteration):
        """迭代调参方法"""
        kp, ki, kd = current_params
        
        # 基于性能指标的调整策略
        overshoot = stats.get('overshoot', 0)
        steady_error = stats.get('steady_state_error', 0)
        oscillation = stats.get('oscillation_intensity', 0)
        target = abs(stats.get('target', 1.0))
        
        # 动态调整因子（随迭代次数递减）
        base_factor = 0.2
        adjustment_factor = base_factor * (0.8 ** iteration)
        
        # 优先级调整策略
        if oscillation > 2.5:
            # 严重振荡：大幅降低Kp，增加Kd
            kp *= (1 - adjustment_factor * 1.5)
            ki *= (1 - adjustment_factor)
            kd *= (1 + adjustment_factor * 2)
        elif overshoot > 20:
            # 严重超调：降低Kp，大幅增加Kd
            kp *= (1 - adjustment_factor)
            kd *= (1 + adjustment_factor * 1.8)
        elif target > 0.1 and (steady_error / target) > 0.1:
            # 稳态误差大：增加Ki，适度增加Kp
            if overshoot < 5:
                kp *= (1 + adjustment_factor * 0.5)
            ki *= (1 + adjustment_factor * 1.2)
        elif overshoot < 3 and oscillation < 0.5:
            # 性能良好，可以尝试提高响应速度
            kp *= (1 + adjustment_factor * 0.3)
            
        return self._constrain_params(kp, ki, kd)
        
    def _constrain_params(self, kp, ki, kd):
        """约束参数在合理范围内"""
        constraints = self.param_constraints
        
        kp = max(constraints['kp']['min'], min(kp, constraints['kp']['max']))
        ki = max(constraints['ki']['min'], min(ki, constraints['ki']['max']))
        kd = max(constraints['kd']['min'], min(kd, constraints['kd']['max']))
        
        # 四舍五入到指定步长
        kp = round(kp / constraints['kp']['step']) * constraints['kp']['step']
        ki = round(ki / constraints['ki']['step']) * constraints['ki']['step']
        kd = round(kd / constraints['kd']['step']) * constraints['kd']['step']
        
        return (kp, ki, kd)
        
    def suggest_next_params(self, current_params, stats, method='iterative'):
        """建议下一组参数"""
        if method == 'iterative':
            iteration = len(self.tuning_history)
            return self.iterative_tuning(current_params, stats, iteration)
        elif method == 'ziegler_nichols':
            # 需要从振荡测试中获取ku和tu
            # 这里简化处理
            ku = current_params[0] * 2.0  # 估算
            tu = 2.0  # 估算
            return self.apply_ziegler_nichols(ku, tu)
        else:
            return current_params
            
    def record_result(self, params, stats, score):
        """记录调参结果"""
        self.tuning_history.append({
            'params': params,
            'stats': stats,
            'score': score,
            'timestamp': datetime.now()
        })
        
        self.performance_history.append(score)
        
        # 保持历史记录在合理长度
        if len(self.tuning_history) > 50:
            self.tuning_history = self.tuning_history[-50:]
            self.performance_history = self.performance_history[-50:]


class AutoTuningWorker(QThread):
    """自动调参工作线程"""
    progress_updated = Signal(int)
    status_updated = Signal(str)
    params_calculated = Signal(dict)  # {'fr': (kp, ki, kd), 'rl': (kp, ki, kd)}
    tuning_completed = Signal(bool, str)  # success, message
    log_message = Signal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.state_machine = AutoTuningStateMachine()
        self.pid_tuner = EnhancedPIDTuner()
        self.parent_window = parent
        self.running = False
        self.data_collection_time = 5.0  # 数据收集时间
        self.verification_time = 3.0  # 验证时间
        self.max_iterations = 5  # 最大迭代次数
        
    def run(self):
        """运行自动调参流程"""
        self.running = True
        self.state_machine.reset()
        
        try:
            # 阶段1：收集初始数据
            if not self._collect_initial_data():
                return
                
            # 阶段2：迭代优化
            for iteration in range(self.max_iterations):
                if not self.running:
                    break
                    
                self.log_message.emit(f"=== 第 {iteration + 1}/{self.max_iterations} 轮调参 ===")
                
                # 分析当前性能
                if not self._analyze_performance():
                    break
                    
                # 计算新参数
                if not self._calculate_new_params(iteration):
                    break
                    
                # 应用新参数
                if not self._apply_params():
                    break
                    
                # 验证效果
                if not self._verify_performance():
                    break
                    
                # 检查是否达到目标
                if self._check_target_reached():
                    self.log_message.emit("已达到性能目标，提前结束调参")
                    break
                    
            # 完成调参
            self._complete_tuning()
            
        except Exception as e:
            self.log_message.emit(f"调参过程出错: {str(e)}")
            self.state_machine.transition_to(TuningStatus.FAILED)
            self.tuning_completed.emit(False, f"调参失败: {str(e)}")
            
    def stop(self):
        """停止调参"""
        self.running = False
        self.state_machine.transition_to(TuningStatus.STOPPED)
        
    def _collect_initial_data(self):
        """收集初始数据"""
        self.state_machine.transition_to(TuningStatus.COLLECTING_DATA)
        self.status_updated.emit("正在收集初始数据...")
        self.log_message.emit("开始收集初始性能数据")
        
        # 清空历史数据
        if self.parent_window:
            self.parent_window.clear_analysis_data()
        
        # 等待数据收集
        for i in range(int(self.data_collection_time * 10)):
            if not self.running:
                return False
                
            progress = self.state_machine.update_progress(i, int(self.data_collection_time * 10))
            self.progress_updated.emit(progress)
            self.msleep(100)
            
        return True
        
    def _analyze_performance(self):
        """分析当前性能"""
        self.state_machine.transition_to(TuningStatus.ANALYZING)
        self.status_updated.emit("正在分析系统性能...")
        
        if not self.parent_window:
            return False
            
        # 获取分析结果
        fr_recommendation, fr_stats, fr_status = self.parent_window.pid_analyzer.analyze_pid_performance_improved(
            self.parent_window.pid_analysis_data['fr']
        )
        rl_recommendation, rl_stats, rl_status = self.parent_window.pid_analyzer.analyze_pid_performance_improved(
            self.parent_window.pid_analysis_data['rl']
        )
        
        # 计算性能得分
        fr_score = self.pid_tuner.calculate_performance_score(fr_stats)
        rl_score = self.pid_tuner.calculate_performance_score(rl_stats)
        
        self.log_message.emit(f"前右电机性能得分: {fr_score:.1f}/100")
        self.log_message.emit(f"后左电机性能得分: {rl_score:.1f}/100")
        
        # 记录结果
        current_fr_params = (
            float(self.parent_window.fr_kp_entry.text()),
            float(self.parent_window.fr_ki_entry.text()),
            float(self.parent_window.fr_kd_entry.text())
        )
        current_rl_params = (
            float(self.parent_window.rl_kp_entry.text()),
            float(self.parent_window.rl_ki_entry.text()),
            float(self.parent_window.rl_kd_entry.text())
        )
        
        self.pid_tuner.record_result(current_fr_params, fr_stats, fr_score)
        self.pid_tuner.record_result(current_rl_params, rl_stats, rl_score)
        
        return True
        
    def _calculate_new_params(self, iteration):
        """计算新参数"""
        self.status_updated.emit("正在计算优化参数...")
        
        if not self.parent_window:
            return False
            
        # 获取当前参数
        current_fr_params = (
            float(self.parent_window.fr_kp_entry.text()),
            float(self.parent_window.fr_ki_entry.text()),
            float(self.parent_window.fr_kd_entry.text())
        )
        current_rl_params = (
            float(self.parent_window.rl_kp_entry.text()),
            float(self.parent_window.rl_ki_entry.text()),
            float(self.parent_window.rl_kd_entry.text())
        )
        
        # 获取最新的统计数据
        fr_stats = self.pid_tuner.tuning_history[-1]['stats'] if self.pid_tuner.tuning_history else {}
        rl_stats = self.pid_tuner.tuning_history[-2]['stats'] if len(self.pid_tuner.tuning_history) > 1 else {}
        
        # 计算新参数
        new_fr_params = self.pid_tuner.suggest_next_params(current_fr_params, fr_stats)
        new_rl_params = self.pid_tuner.suggest_next_params(current_rl_params, rl_stats)
        
        self.log_message.emit(f"FR新参数: Kp={new_fr_params[0]:.3f}, Ki={new_fr_params[1]:.3f}, Kd={new_fr_params[2]:.3f}")
        self.log_message.emit(f"RL新参数: Kp={new_rl_params[0]:.3f}, Ki={new_rl_params[1]:.3f}, Kd={new_rl_params[2]:.3f}")
        
        # 发送新参数
        self.params_calculated.emit({
            'fr': new_fr_params,
            'rl': new_rl_params
        })
        
        return True
        
    def _apply_params(self):
        """应用参数（由主线程执行）"""
        self.state_machine.transition_to(TuningStatus.APPLYING_PARAMS)
        self.status_updated.emit("正在应用新参数...")
        
        # 等待参数应用
        self.msleep(1000)
        
        # 清空数据准备下一轮测试
        if self.parent_window:
            self.parent_window.clear_analysis_data()
            
        return True
        
    def _verify_performance(self):
        """验证性能"""
        self.state_machine.transition_to(TuningStatus.VERIFYING)
        self.status_updated.emit("正在验证调参效果...")
        
        # 等待系统稳定并收集数据
        for i in range(int(self.verification_time * 10)):
            if not self.running:
                return False
                
            progress = self.state_machine.update_progress(
                50 + int(50 * i / (self.verification_time * 10)),
                100
            )
            self.progress_updated.emit(progress)
            self.msleep(100)
            
        return True
        
    def _check_target_reached(self):
        """检查是否达到目标性能"""
        if len(self.pid_tuner.performance_history) < 2:
            return False
            
        # 检查最近的性能得分
        latest_score = self.pid_tuner.performance_history[-1]
        previous_score = self.pid_tuner.performance_history[-2]
        
        # 如果性能得分高于90分，或者改善小于2%，认为达到目标
        if latest_score > 90 or abs(latest_score - previous_score) < 2:
            return True
            
        return False
        
    def _complete_tuning(self):
        """完成调参"""
        self.state_machine.transition_to(TuningStatus.COMPLETED)
        self.status_updated.emit("调参完成")
        
        # 生成调参报告
        if self.pid_tuner.tuning_history:
            best_result = max(self.pid_tuner.tuning_history, key=lambda x: x['score'])
            message = f"调参完成！最佳性能得分: {best_result['score']:.1f}/100"
        else:
            message = "调参完成"
            
        self.tuning_completed.emit(True, message)


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
            QPushButton:disabled {{
                background-color: #BDBDBD;
                color: #757575;
            }}
        """)
        
        self.setCursor(Qt.PointingHandCursor)


class SerialThread(QThread):
    """串口接收线程"""
    packet_received = Signal(int, list)  # cmd, data
    packet_count_updated = Signal(int)
    connection_lost = Signal(str)  # 连接丢失信号
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
        self.packet_count = 0
        
    def run(self):
        packet_state = 'WAIT_HEADER1'
        packet_data = []
        packet_cmd = 0
        packet_len = 0
        packet_checksum = 0
        last_receive_time = time.time()
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    last_receive_time = time.time()
                    
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
                                self.packet_count += 1
                                self.packet_received.emit(packet_cmd, packet_data)
                                self.packet_count_updated.emit(self.packet_count)
                            packet_state = 'WAIT_HEADER1'
                
                # 检查连接超时
                if time.time() - last_receive_time > 5.0:
                    self.connection_lost.emit("数据接收超时")
                    
            except Exception as e:
                self.connection_lost.emit(f"接收错误: {e}")
                break
                
            self.msleep(1)
            
    def stop(self):
        self.running = False
        self.wait()


class SimpleChartWidget(QWidget):
    """简化的图表组件，避免matplotlib Qt后端问题"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 300)  # 减小最小高度
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
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


class ImprovedPIDAnalyzer:
    """改进的PID控制器分析和调试建议类"""
    
    def __init__(self):
        self.analysis_interval = 2.0
        self.settling_tolerance = 0.05
        self.overshoot_threshold = 0.1
        self.oscillation_threshold = 3
        self.min_data_points = 30  # 最少数据点要求
        
        # 数据质量阈值
        self.max_time_gap = 1.0  # 最大时间间隔
        self.min_target_changes = 2  # 最少目标变化次数
        
        # 调参历史记录
        self.tuning_history = []
        
    def validate_data_quality(self, motor_data):
        """验证数据质量"""
        errors = list(motor_data['errors'])
        targets = list(motor_data['targets'])
        timestamps = list(motor_data['timestamps'])
        
        # 检查数据量
        if len(errors) < self.min_data_points:
            return False, f"数据不足，需要至少{self.min_data_points}个数据点，当前{len(errors)}个"
        
        # 检查时间连续性
        if len(timestamps) >= 2:
            time_gaps = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
            max_gap = max(time_gaps) if time_gaps else 0
            if max_gap > self.max_time_gap:
                return False, f"数据不连续，最大间隔{max_gap:.2f}秒"
        
        # 检查目标值变化
        recent_targets = targets[-30:] if len(targets) >= 30 else targets
        target_changes = 0
        for i in range(1, len(recent_targets)):
            if abs(recent_targets[i] - recent_targets[i-1]) > 1.0:
                target_changes += 1
        
        if target_changes < self.min_target_changes:
            return False, f"目标变化不足，需要至少{self.min_target_changes}次变化，当前{target_changes}次"
        
        return True, "数据质量良好"
    
    def detect_oscillation_improved(self, errors, timestamps):
        """改进的振荡检测算法"""
        if len(errors) < 20:
            return 0, 0, "数据不足"
        
        recent_errors = errors[-20:]
        
        # 方法1：基于方差的振荡强度
        error_variance = np.var(recent_errors)
        mean_error = np.mean([abs(e) for e in recent_errors])
        oscillation_intensity = error_variance / (mean_error + 0.001)
        
        # 方法2：改进的符号变化检测（加入幅度阈值）
        threshold = max(0.5, mean_error * 0.2)  # 动态阈值
        significant_changes = 0
        
        for i in range(len(recent_errors) - 1):
            current_error = recent_errors[i]
            next_error = recent_errors[i + 1]
            
            # 符号变化且幅度显著
            if (current_error * next_error < 0 and 
                abs(current_error) > threshold and 
                abs(next_error) > threshold):
                significant_changes += 1
        
        # 方法3：频率分析（如果数据足够）
        dominant_frequency = 0
        if len(errors) >= 50:
            try:
                # 简单的频域分析
                fft_result = np.fft.fft(errors[-50:])
                power_spectrum = np.abs(fft_result[1:25])  # 排除DC分量
                dominant_frequency = np.argmax(power_spectrum)
            except:
                pass
        
        analysis_result = f"振荡强度: {oscillation_intensity:.3f}, 符号变化: {significant_changes}, 主频: {dominant_frequency}"
        return oscillation_intensity, significant_changes, analysis_result
    
    def detect_overshoot_improved(self, actuals, targets, timestamps):
        """改进的超调检测"""
        if len(targets) < 10:
            return 0, "数据不足"
        
        # 检测目标值显著变化
        target_changes = []
        for i in range(1, len(targets)):
            if abs(targets[i] - targets[i-1]) > 1.0:
                target_changes.append({
                    'index': i,
                    'time': timestamps[i],
                    'old_target': targets[i-1],
                    'new_target': targets[i]
                })
        
        if not target_changes:
            return 0, "无目标变化"
        
        # 分析最近一次目标变化后的响应
        last_change = target_changes[-1]
        change_idx = last_change['index']
        change_time = last_change['time']
        new_target = last_change['new_target']
        old_target = last_change['old_target']
        
        # 确定是阶跃上升还是下降
        is_step_up = new_target > old_target
        
        # 寻找变化后的响应数据（5秒内）
        response_data = []
        response_times = []
        
        for i in range(change_idx, len(actuals)):
            if i < len(timestamps) and timestamps[i] - change_time < 5.0:
                response_data.append(actuals[i])
                response_times.append(timestamps[i] - change_time)
        
        if len(response_data) < 5:
            return 0, "响应数据不足"
        
        # 计算超调
        if is_step_up:
            max_response = max(response_data)
            overshoot = max(0, (max_response - new_target) / abs(new_target) * 100) if new_target != 0 else 0
            analysis = f"阶跃上升，最大值: {max_response:.1f}, 目标: {new_target:.1f}"
        else:
            min_response = min(response_data)
            overshoot = max(0, (new_target - min_response) / abs(new_target) * 100) if new_target != 0 else 0
            analysis = f"阶跃下降，最小值: {min_response:.1f}, 目标: {new_target:.1f}"
        
        return overshoot, analysis
    
    def is_steady_state(self, errors, timestamps, settling_time=3.0):
        """判断是否达到稳态"""
        if len(errors) < 20 or len(timestamps) < 20:
            return False, "数据不足"
        
        # 检查最近的数据是否满足稳态条件
        current_time = timestamps[-1]
        steady_data = []
        
        for i in range(len(timestamps)):
            if current_time - timestamps[i] <= settling_time:
                steady_data.append(errors[i])
        
        if len(steady_data) < 10:
            return False, "稳态时间内数据不足"
        
        # 稳态条件检查
        max_error = max([abs(e) for e in steady_data])
        error_variance = np.var(steady_data)
        mean_error = np.mean([abs(e) for e in steady_data])
        
        is_steady = (max_error < self.settling_tolerance * 10 and 
                    error_variance < 1.0 and 
                    mean_error < self.settling_tolerance * 5)
        
        analysis = f"最大误差: {max_error:.3f}, 方差: {error_variance:.3f}, 平均误差: {mean_error:.3f}"
        return is_steady, analysis
    
    def analyze_pid_performance_improved(self, motor_data):
        """改进的PID性能分析"""
        # 数据质量检查
        quality_ok, quality_msg = self.validate_data_quality(motor_data)
        if not quality_ok:
            return quality_msg, {}, "warning"
        
        # 基础统计
        errors = list(motor_data['errors'])
        outputs = list(motor_data['outputs'])
        targets = list(motor_data['targets'])
        actuals = list(motor_data['actuals'])
        timestamps = list(motor_data['timestamps'])
        
        # 计算改进的分析指标
        current_error = abs(errors[-1]) if errors else 0
        avg_error = np.mean([abs(e) for e in errors[-10:]]) if len(errors) >= 10 else 0
        
        # 稳态分析
        is_steady, steady_analysis = self.is_steady_state(errors, timestamps)
        steady_state_error = np.mean([abs(e) for e in errors[-10:]]) if is_steady else avg_error
        
        # 改进的振荡检测
        oscillation_intensity, significant_changes, osc_analysis = self.detect_oscillation_improved(
            errors, timestamps)
        
        # 改进的超调检测
        overshoot, overshoot_analysis = self.detect_overshoot_improved(
            actuals, targets, timestamps)
        
        # 构建统计结果
        stats = {
            'current_error': current_error,
            'avg_error': avg_error,
            'steady_state_error': steady_state_error,
            'overshoot': overshoot,
            'oscillation_intensity': oscillation_intensity,
            'significant_changes': significant_changes,
            'target': targets[-1] if targets else 0,
            'actual': actuals[-1] if actuals else 0,
            'output': outputs[-1] if outputs else 0,
            'is_steady_state': is_steady,
            'data_quality': quality_msg,
            'oscillation_analysis': osc_analysis,
            'overshoot_analysis': overshoot_analysis,
            'steady_analysis': steady_analysis
        }
        
        # 生成建议
        recommendation, status = self.generate_improved_recommendation(stats)
        
        return recommendation, stats, status
    
    def generate_improved_recommendation(self, stats):
        """生成改进的调参建议"""
        recommendations = []
        status = "info"
        
        target = stats.get('target', 0)
        if abs(target) < 1.0:
            return "目标速度接近零，暂停分析", "info"
        
        # 数据质量提示
        recommendations.append(f"📊 {stats['data_quality']}")
        
        # 稳态分析
        if stats['is_steady_state']:
            recommendations.append("✅ 系统已达到稳态")
        else:
            recommendations.append("⏳ 系统未达到稳态，分析结果仅供参考")
        
        # 性能分析
        steady_error_pct = (stats['steady_state_error'] / abs(target)) * 100
        overshoot = stats['overshoot']
        osc_intensity = stats['oscillation_intensity']
        
        # 稳态误差分析
        if steady_error_pct > 10:
            recommendations.append(f"⚠️ 稳态误差过大({steady_error_pct:.1f}%)")
            status = "warning"
        elif steady_error_pct > 5:
            recommendations.append(f"💡 稳态误差偏大({steady_error_pct:.1f}%)")
        else:
            recommendations.append(f"✅ 稳态误差良好({steady_error_pct:.1f}%)")
        
        # 超调分析
        if overshoot > 15:
            recommendations.append(f"⚠️ 超调严重({overshoot:.1f}%)")
            status = "warning"
        elif overshoot > 8:
            recommendations.append(f"💡 有轻微超调({overshoot:.1f}%)")
        else:
            recommendations.append(f"✅ 超调控制良好({overshoot:.1f}%)")
        
        # 振荡分析
        if osc_intensity > 2.0:
            recommendations.append("⚠️ 检测到明显振荡")
            status = "warning"
        elif osc_intensity > 1.0:
            recommendations.append("💡 有轻微振荡趋势")
        else:
            recommendations.append("✅ 系统稳定，无振荡")
        
        # 详细分析信息
        recommendations.append(f"\n📈 详细分析:")
        recommendations.append(f"• {stats['oscillation_analysis']}")
        recommendations.append(f"• {stats['overshoot_analysis']}")
        recommendations.append(f"• {stats['steady_analysis']}")
        
        return "\n".join(recommendations), status


class RobotControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Robot Control System - Ubuntu (Optimized)")
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
        self.connection_stable = True
        
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
                'errors': deque(maxlen=100),
                'outputs': deque(maxlen=100), 
                'targets': deque(maxlen=100),
                'actuals': deque(maxlen=100),
                'timestamps': deque(maxlen=100),
                'last_target_change': time.time(),
                'settling_started': None,
                'overshoot_detected': False,
                'oscillation_count': 0,
                'last_recommendation': '',
                'analysis_stats': {}
            },
            'rl': {
                'errors': deque(maxlen=100),
                'outputs': deque(maxlen=100),
                'targets': deque(maxlen=100),
                'actuals': deque(maxlen=100),
                'timestamps': deque(maxlen=100),
                'last_target_change': time.time(),
                'settling_started': None,
                'overshoot_detected': False,
                'oscillation_count': 0,
                'last_recommendation': '',
                'analysis_stats': {}
            }
        }
        
        # 改进的PID分析器
        self.pid_analyzer = ImprovedPIDAnalyzer()
        
        # 自动调参相关
        self.auto_tuning_worker = None
        self.backup_params = None
        
        # 初始化UI组件为None，避免访问未创建的对象
        self.fr_stats_label = None
        self.fr_recommendation_label = None
        self.rl_stats_label = None
        self.rl_recommendation_label = None
        self.global_advice_label = None
        self.tuning_progress_bar = None
        self.tuning_log_text = None
        
        # 创建主界面
        self.create_ui()
        
        # 定时器
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)
        
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.request_pid_status)
        
        # PID分析定时器
        self.analysis_timer = QTimer()
        self.analysis_timer.timeout.connect(self.update_pid_analysis)
        self.analysis_timer.start(1000)  # 每秒更新一次分析
        
        # 连接状态检查定时器
        self.connection_check_timer = QTimer()
        self.connection_check_timer.timeout.connect(self.check_connection_status)
        self.connection_check_timer.start(5000)  # 每5秒检查一次连接状态
        
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
        
        # 右侧内容（添加滚动区域）
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
        title_label = QLabel("STM32 Robot Control System - Ubuntu (Optimized)")
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
        """创建右侧面板（带滚动）"""
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
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
        
        # 实时图表卡片
        self.create_chart_card(scroll_layout)
        
        # PID调试建议卡片
        self.create_pid_analysis_card(scroll_layout)
        
        # 状态显示卡片
        self.create_status_card(scroll_layout)
        
        # 添加底部空间
        scroll_layout.addSpacing(20)
        
        scroll_area.setWidget(scroll_content)
        
        return scroll_area
        
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
        
        # 连接状态显示
        self.connection_status_label = QLabel("未连接")
        self.connection_status_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_secondary']};
                font-size: 10pt;
                padding: 4px;
                background-color: #F5F5F5;
                border-radius: 4px;
            }}
        """)
        layout.addWidget(self.connection_status_label)
        
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
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
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
        self.auto_tune_btn = MaterialButton("AUTO TUNE", "secondary")
        self.auto_tune_btn.clicked.connect(self.auto_tune_pid)
        header_layout.addWidget(self.auto_tune_btn)
        
        # 停止调参按钮
        self.stop_tune_btn = MaterialButton("STOP", "danger")
        self.stop_tune_btn.clicked.connect(self.stop_auto_tune)
        self.stop_tune_btn.setVisible(False)
        header_layout.addWidget(self.stop_tune_btn)
        
        layout.addLayout(header_layout)
        
        # 调参进度条
        self.tuning_progress_bar = QProgressBar()
        self.tuning_progress_bar.setVisible(False)
        self.tuning_progress_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid {self.colors['divider']};
                border-radius: 4px;
                background-color: #F5F5F5;
                height: 20px;
                text-align: center;
            }}
            QProgressBar::chunk {{
                background-color: {self.colors['primary']};
                border-radius: 3px;
            }}
        """)
        layout.addWidget(self.tuning_progress_bar)
        
        # 分析结果显示区域（使用垂直布局以适应小屏幕）
        analysis_layout = QVBoxLayout()
        
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
        self.fr_stats_label.setWordWrap(True)
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
        self.rl_stats_label.setWordWrap(True)
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
        
        # 调参日志
        log_label = QLabel("🔧 调参日志")
        log_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_primary']};
                font-size: 12pt;
                font-weight: bold;
                margin-top: 10px;
            }}
        """)
        layout.addWidget(log_label)
        
        self.tuning_log_text = QTextEdit()
        self.tuning_log_text.setMaximumHeight(150)
        self.tuning_log_text.setReadOnly(True)
        self.tuning_log_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: #F5F5F5;
                border: 1px solid {self.colors['divider']};
                border-radius: 4px;
                font-family: monospace;
                font-size: 9pt;
                color: {self.colors['text_secondary']};
            }}
        """)
        self.tuning_log_text.setPlainText("调参日志将在这里显示...")
        layout.addWidget(self.tuning_log_text)
        
        parent_layout.addWidget(card)
        
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
            QPushButton:disabled {{
                background-color: #BDBDBD;
                color: #757575;
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
            QPushButton:disabled {{
                background-color: #BDBDBD;
                color: #757575;
            }}
        """
        
        # 前进
        self.forward_btn = QPushButton("▲")
        self.forward_btn.setStyleSheet(button_style)
        self.forward_btn.clicked.connect(lambda: self.send_direction_cmd(0x00))
        control_layout.addWidget(self.forward_btn, 0, 1)
        
        # 左转
        self.left_btn = QPushButton("◄")
        self.left_btn.setStyleSheet(button_style)
        self.left_btn.clicked.connect(lambda: self.send_direction_cmd(0x02))
        control_layout.addWidget(self.left_btn, 1, 0)
        
        # 停止
        self.stop_btn = QPushButton("■")
        self.stop_btn.setStyleSheet(stop_button_style)
        self.stop_btn.clicked.connect(lambda: self.send_direction_cmd(0x04))
        control_layout.addWidget(self.stop_btn, 1, 1)
        
        # 右转
        self.right_btn = QPushButton("►")
        self.right_btn.setStyleSheet(button_style)
        self.right_btn.clicked.connect(lambda: self.send_direction_cmd(0x03))
        control_layout.addWidget(self.right_btn, 1, 2)
        
        # 后退
        self.backward_btn = QPushButton("▼")
        self.backward_btn.setStyleSheet(button_style)
        self.backward_btn.clicked.connect(lambda: self.send_direction_cmd(0x01))
        control_layout.addWidget(self.backward_btn, 2, 1)
        
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
        self.apply_btn = MaterialButton("APPLY PARAMETERS", "primary")
        self.apply_btn.clicked.connect(self.apply_pid_params)
        layout.addWidget(self.apply_btn)
        
        button_layout = QHBoxLayout()
        self.reset_btn = MaterialButton("RESET", "secondary")
        self.reset_btn.clicked.connect(self.reset_pid)
        self.test_btn = MaterialButton("TEST", "secondary")
        self.test_btn.clicked.connect(self.test_communication)
        
        button_layout.addWidget(self.reset_btn)
        button_layout.addWidget(self.test_btn)
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
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
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
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
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
        self.data_rate_label = self.create_status_item(comm_group, "Rate", "0 Hz")
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
            self.serial_thread.connection_lost.connect(self.on_connection_lost)
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
            
            self.connection_status_label.setText("已连接")
            self.connection_status_label.setStyleSheet(f"""
                QLabel {{
                    color: {self.colors['success']};
                    font-size: 10pt;
                    font-weight: bold;
                    padding: 4px;
                    background-color: #E8F5E8;
                    border-radius: 4px;
                }}
            """)
            
            self.connection_stable = True
            self.enable_motion_controls(True)
            
            # 清除图表
            self.clear_chart()
            
            # 启动定时器
            self.status_timer.start(100)
            
            self.add_log("串口连接成功")
            
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
        
        self.connection_status_label.setText("未连接")
        self.connection_status_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['text_secondary']};
                font-size: 10pt;
                padding: 4px;
                background-color: #F5F5F5;
                border-radius: 4px;
            }}
        """)
        
        self.enable_motion_controls(False)
        self.add_log("串口连接断开")
        
    def on_connection_lost(self, reason):
        """连接丢失处理"""
        self.connection_stable = False
        self.connection_status_label.setText(f"连接异常: {reason}")
        self.connection_status_label.setStyleSheet(f"""
            QLabel {{
                color: {self.colors['error']};
                font-size: 10pt;
                font-weight: bold;
                padding: 4px;
                background-color: #FFEBEE;
                border-radius: 4px;
            }}
        """)
        self.add_log(f"连接异常: {reason}")
        
    def check_connection_status(self):
        """检查连接状态"""
        if self.serial_port and self.serial_port.is_open:
            if not self.connection_stable:
                # 尝试恢复连接
                self.add_log("尝试恢复连接...")
        
    def enable_motion_controls(self, enabled):
        """启用/禁用运动控制按钮"""
        self.forward_btn.setEnabled(enabled)
        self.backward_btn.setEnabled(enabled)
        self.left_btn.setEnabled(enabled)
        self.right_btn.setEnabled(enabled)
        self.stop_btn.setEnabled(enabled)
        self.speed_slider.setEnabled(enabled)
        self.apply_btn.setEnabled(enabled)
        self.reset_btn.setEnabled(enabled)
        self.test_btn.setEnabled(enabled)
        
    def process_packet(self, cmd, data):
        """处理接收到的数据包"""
        self.connection_stable = True
        
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
            return False
            
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
            return True
        except Exception as e:
            QMessageBox.critical(self, "Send Failed", str(e))
            return False
            
    # ========== 控制命令相关方法 ==========
    
    def send_direction_cmd(self, direction):
        """发送方向控制命令"""
        speed = self.speed_slider.value()
        data = [direction, speed]
        if self.send_packet(0x01, data):
            direction_names = {0x00: "前进", 0x01: "后退", 0x02: "左转", 0x03: "右转", 0x04: "停止"}
            self.add_log(f"发送运动命令: {direction_names.get(direction, '未知')} - 速度: {speed}%")
        
    def on_speed_change(self, value):
        """速度滑块变化时的回调"""
        self.speed_value_label.setText(f"{value}%")
        
        if self.serial_port and self.serial_port.is_open:
            if self.send_packet(0x02, [value]):
                self.add_log(f"设置速度: {value}%")
            
    def toggle_pid(self, state):
        """切换PID控制状态"""
        enable = 1 if state == Qt.Checked else 0
        if self.send_packet(0x06, [enable]):
            self.add_log(f"PID控制: {'启用' if enable else '禁用'}")
            
    def auto_tune_pid(self):
        """启动自动调参功能"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "请先连接串口")
            return
        
        # 检查是否已有调参任务在运行
        if self.auto_tuning_worker and self.auto_tuning_worker.isRunning():
            QMessageBox.information(self, "Info", "自动调参正在进行中")
            return
            
        # 检查数据质量
        fr_quality_ok, fr_quality_msg = self.pid_analyzer.validate_data_quality(self.pid_analysis_data['fr'])
        rl_quality_ok, rl_quality_msg = self.pid_analyzer.validate_data_quality(self.pid_analysis_data['rl'])
        
        if not fr_quality_ok or not rl_quality_ok:
            QMessageBox.warning(self, "Data Quality", 
                              f"数据质量不足，无法进行自动调参\n\n"
                              f"前右电机: {fr_quality_msg}\n"
                              f"后左电机: {rl_quality_msg}\n\n"
                              f"请先进行运动控制，让系统运行一段时间后再尝试")
            return
        
        # 确认开始调参
        reply = QMessageBox.question(self, "自动调参确认", 
                                   "即将开始自动调参，该过程可能需要几分钟时间。\n\n"
                                   "调参过程中将会：\n"
                                   "1. 备份当前参数\n"
                                   "2. 分析系统性能\n"
                                   "3. 迭代优化参数\n"
                                   "4. 验证调参效果\n\n"
                                   "调参期间请保持系统正常运行\n\n"
                                   "是否继续？", 
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply != QMessageBox.Yes:
            return
        
        # 开始自动调参
        self.start_auto_tuning()
    
    def start_auto_tuning(self):
        """开始自动调参过程"""
        # 备份当前参数
        self.backup_params = {
            'fr': {
                'kp': float(self.fr_kp_entry.text()),
                'ki': float(self.fr_ki_entry.text()),
                'kd': float(self.fr_kd_entry.text())
            },
            'rl': {
                'kp': float(self.rl_kp_entry.text()),
                'ki': float(self.rl_ki_entry.text()),
                'kd': float(self.rl_kd_entry.text())
            }
        }
        
        # 更新UI
        self.auto_tune_btn.setVisible(False)
        self.stop_tune_btn.setVisible(True)
        self.tuning_progress_bar.setVisible(True)
        self.tuning_progress_bar.setValue(0)
        
        # 禁用参数输入
        self.fr_kp_entry.setEnabled(False)
        self.fr_ki_entry.setEnabled(False)
        self.fr_kd_entry.setEnabled(False)
        self.rl_kp_entry.setEnabled(False)
        self.rl_ki_entry.setEnabled(False)
        self.rl_kd_entry.setEnabled(False)
        self.apply_btn.setEnabled(False)
        
        # 创建并启动调参工作线程
        self.auto_tuning_worker = AutoTuningWorker(self)
        self.auto_tuning_worker.progress_updated.connect(self.update_tuning_progress)
        self.auto_tuning_worker.status_updated.connect(self.update_tuning_status)
        self.auto_tuning_worker.params_calculated.connect(self.apply_calculated_params)
        self.auto_tuning_worker.tuning_completed.connect(self.on_tuning_completed)
        self.auto_tuning_worker.log_message.connect(self.add_log)
        
        self.add_log("="*50)
        self.add_log("开始自动调参...")
        self.add_log(f"备份参数 - FR: Kp={self.backup_params['fr']['kp']:.2f}, "
                    f"Ki={self.backup_params['fr']['ki']:.2f}, "
                    f"Kd={self.backup_params['fr']['kd']:.2f}")
        self.add_log(f"备份参数 - RL: Kp={self.backup_params['rl']['kp']:.2f}, "
                    f"Ki={self.backup_params['rl']['ki']:.2f}, "
                    f"Kd={self.backup_params['rl']['kd']:.2f}")
        
        # 启动工作线程
        self.auto_tuning_worker.start()
    
    def update_tuning_progress(self, progress):
        """更新调参进度"""
        self.tuning_progress_bar.setValue(progress)
    
    def update_tuning_status(self, status):
        """更新调参状态"""
        self.tuning_progress_bar.setFormat(f"{status} - {self.tuning_progress_bar.value()}%")
    
    def apply_calculated_params(self, params_dict):
        """应用计算出的新参数"""
        fr_params = params_dict['fr']
        rl_params = params_dict['rl']
        
        # 更新UI输入框
        self.fr_kp_entry.setText(f"{fr_params[0]:.3f}")
        self.fr_ki_entry.setText(f"{fr_params[1]:.3f}")
        self.fr_kd_entry.setText(f"{fr_params[2]:.3f}")
        
        self.rl_kp_entry.setText(f"{rl_params[0]:.3f}")
        self.rl_ki_entry.setText(f"{rl_params[1]:.3f}")
        self.rl_kd_entry.setText(f"{rl_params[2]:.3f}")
        
        # 应用到STM32
        self._apply_pid_params_internal()
    
    def _apply_pid_params_internal(self):
        """内部方法：应用PID参数到STM32"""
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
                      
            if self.send_packet(0x07, data_fr):
                # 延时后发送后左电机参数
                QTimer.singleShot(100, lambda: self.send_rl_pid_params(rl_kp, rl_ki, rl_kd))
                
        except ValueError:
            self.add_log("参数应用失败：无效的数值")
    
    def on_tuning_completed(self, success, message):
        """调参完成的处理"""
        # 恢复UI状态
        self.auto_tune_btn.setVisible(True)
        self.stop_tune_btn.setVisible(False)
        self.tuning_progress_bar.setVisible(False)
        
        # 重新启用参数输入
        self.fr_kp_entry.setEnabled(True)
        self.fr_ki_entry.setEnabled(True)
        self.fr_kd_entry.setEnabled(True)
        self.rl_kp_entry.setEnabled(True)
        self.rl_ki_entry.setEnabled(True)
        self.rl_kd_entry.setEnabled(True)
        self.apply_btn.setEnabled(True)
        
        if success:
            # 获取最终参数
            final_fr_params = (
                float(self.fr_kp_entry.text()),
                float(self.fr_ki_entry.text()),
                float(self.fr_kd_entry.text())
            )
            
            final_rl_params = (
                float(self.rl_kp_entry.text()),
                float(self.rl_ki_entry.text()),
                float(self.rl_kd_entry.text())
            )
            
            # 计算参数变化
            fr_kp_change = ((final_fr_params[0] - self.backup_params['fr']['kp']) / 
                           self.backup_params['fr']['kp'] * 100)
            fr_ki_change = ((final_fr_params[1] - self.backup_params['fr']['ki']) / 
                           self.backup_params['fr']['ki'] * 100) if self.backup_params['fr']['ki'] > 0 else 0
            fr_kd_change = ((final_fr_params[2] - self.backup_params['fr']['kd']) / 
                           self.backup_params['fr']['kd'] * 100) if self.backup_params['fr']['kd'] > 0 else 0
            
            rl_kp_change = ((final_rl_params[0] - self.backup_params['rl']['kp']) / 
                           self.backup_params['rl']['kp'] * 100)
            rl_ki_change = ((final_rl_params[1] - self.backup_params['rl']['ki']) / 
                           self.backup_params['rl']['ki'] * 100) if self.backup_params['rl']['ki'] > 0 else 0
            rl_kd_change = ((final_rl_params[2] - self.backup_params['rl']['kd']) / 
                           self.backup_params['rl']['kd'] * 100) if self.backup_params['rl']['kd'] > 0 else 0
            
            result_text = f"""自动调参完成！

{message}

参数变化对比：

前右电机：
原始参数: Kp={self.backup_params['fr']['kp']:.3f}, Ki={self.backup_params['fr']['ki']:.3f}, Kd={self.backup_params['fr']['kd']:.3f}
调整后: Kp={final_fr_params[0]:.3f} ({fr_kp_change:+.1f}%), Ki={final_fr_params[1]:.3f} ({fr_ki_change:+.1f}%), Kd={final_fr_params[2]:.3f} ({fr_kd_change:+.1f}%)

后左电机：
原始参数: Kp={self.backup_params['rl']['kp']:.3f}, Ki={self.backup_params['rl']['ki']:.3f}, Kd={self.backup_params['rl']['kd']:.3f}
调整后: Kp={final_rl_params[0]:.3f} ({rl_kp_change:+.1f}%), Ki={final_rl_params[1]:.3f} ({rl_ki_change:+.1f}%), Kd={final_rl_params[2]:.3f} ({rl_kd_change:+.1f}%)

建议继续运行一段时间以验证调参效果。
如果效果不理想，可以点击"恢复备份"按钮恢复原始参数。"""
            
            # 添加恢复备份按钮的消息框
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle("自动调参完成")
            msg_box.setText(result_text)
            msg_box.setStandardButtons(QMessageBox.Ok)
            
            restore_btn = msg_box.addButton("恢复备份参数", QMessageBox.ActionRole)
            msg_box.exec()
            
            if msg_box.clickedButton() == restore_btn:
                self.restore_backup_params()
            
            self.add_log("="*50)
            self.add_log("自动调参完成")
            
        else:
            # 调参失败
            QMessageBox.warning(self, "调参失败", message)
            
            # 询问是否恢复备份参数
            if self.backup_params:
                restore_reply = QMessageBox.question(self, "恢复参数", 
                                                   "调参失败，是否恢复到调参前的参数？", 
                                                   QMessageBox.Yes | QMessageBox.No)
                if restore_reply == QMessageBox.Yes:
                    self.restore_backup_params()
        
        # 清空分析数据以重新开始分析
        self.clear_analysis_data()
        
        # 清理工作线程
        if self.auto_tuning_worker:
            self.auto_tuning_worker.deleteLater()
            self.auto_tuning_worker = None
    
    def stop_auto_tune(self):
        """停止自动调参"""
        if not self.auto_tuning_worker or not self.auto_tuning_worker.isRunning():
            return
        
        reply = QMessageBox.question(self, "停止调参", 
                                   "确定要停止自动调参吗？\n\n"
                                   "停止后可以选择恢复到调参前的参数。", 
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply != QMessageBox.Yes:
            return
        
        # 停止工作线程
        self.auto_tuning_worker.stop()
        self.auto_tuning_worker.wait()  # 等待线程结束
        
        # 恢复UI状态
        self.auto_tune_btn.setVisible(True)
        self.stop_tune_btn.setVisible(False)
        self.tuning_progress_bar.setVisible(False)
        
        # 重新启用参数输入
        self.fr_kp_entry.setEnabled(True)
        self.fr_ki_entry.setEnabled(True)
        self.fr_kd_entry.setEnabled(True)
        self.rl_kp_entry.setEnabled(True)
        self.rl_ki_entry.setEnabled(True)
        self.rl_kd_entry.setEnabled(True)
        self.apply_btn.setEnabled(True)
        
        # 询问是否恢复备份参数
        if self.backup_params:
            restore_reply = QMessageBox.question(self, "恢复参数", 
                                               "是否恢复到调参前的参数？", 
                                               QMessageBox.Yes | QMessageBox.No)
            if restore_reply == QMessageBox.Yes:
                self.restore_backup_params()
        
        self.add_log("自动调参已停止")
        
        # 清理工作线程
        if self.auto_tuning_worker:
            self.auto_tuning_worker.deleteLater()
            self.auto_tuning_worker = None
    
    def restore_backup_params(self):
        """恢复备份的参数"""
        if not self.backup_params:
            QMessageBox.warning(self, "Warning", "没有可恢复的备份参数")
            return
        
        # 恢复参数到输入框
        self.fr_kp_entry.setText(str(self.backup_params['fr']['kp']))
        self.fr_ki_entry.setText(str(self.backup_params['fr']['ki']))
        self.fr_kd_entry.setText(str(self.backup_params['fr']['kd']))
        
        self.rl_kp_entry.setText(str(self.backup_params['rl']['kp']))
        self.rl_ki_entry.setText(str(self.backup_params['rl']['ki']))
        self.rl_kd_entry.setText(str(self.backup_params['rl']['kd']))
        
        # 应用到STM32
        self.apply_pid_params()
        
        self.add_log("已恢复备份参数")
        QMessageBox.information(self, "Success", "参数已恢复到调参前的状态")
    
    def clear_analysis_data(self):
        """清空分析数据"""
        for motor_key in ['fr', 'rl']:
            for data_key in ['errors', 'outputs', 'targets', 'actuals', 'timestamps']:
                self.pid_analysis_data[motor_key][data_key].clear()
        self.add_log("清空分析数据，重新开始分析")
    
    def add_log(self, message):
        """添加日志信息"""
        if self.tuning_log_text:
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_entry = f"[{timestamp}] {message}"
            self.tuning_log_text.append(log_entry)
            
            # 滚动到底部
            scrollbar = self.tuning_log_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            
            # 限制日志行数
            document = self.tuning_log_text.document()
            if document.blockCount() > 200:
                cursor = self.tuning_log_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deletePreviousChar()  # 删除换行符
    
    def apply_pid_params(self):
        """应用PID参数"""
        # 检查是否有调参任务在运行
        if self.auto_tuning_worker and self.auto_tuning_worker.isRunning():
            QMessageBox.warning(self, "Warning", "自动调参正在进行中，请等待完成或停止调参")
            return
            
        try:
            # 获取前右电机参数
            fr_kp = float(self.fr_kp_entry.text())
            fr_ki = float(self.fr_ki_entry.text())
            fr_kd = float(self.fr_kd_entry.text())
            
            # 获取后左电机参数
            rl_kp = float(self.rl_kp_entry.text())
            rl_ki = float(self.rl_ki_entry.text())
            rl_kd = float(self.rl_kd_entry.text())
            
            # 参数合理性检查
            if not self.validate_pid_params(fr_kp, fr_ki, fr_kd) or not self.validate_pid_params(rl_kp, rl_ki, rl_kd):
                return
            
            # 发送前右电机PID参数
            kp_fr = int(fr_kp * 100)
            ki_fr = int(fr_ki * 100)
            kd_fr = int(fr_kd * 100)
            
            data_fr = [0,  # Motor ID
                      (kp_fr >> 8) & 0xFF, kp_fr & 0xFF,
                      (ki_fr >> 8) & 0xFF, ki_fr & 0xFF,
                      (kd_fr >> 8) & 0xFF, kd_fr & 0xFF]
                      
            if self.send_packet(0x07, data_fr):
                self.add_log(f"设置前右电机PID: Kp={fr_kp:.2f}, Ki={fr_ki:.2f}, Kd={fr_kd:.2f}")
                
                # 延时后发送后左电机参数
                QTimer.singleShot(100, lambda: self.send_rl_pid_params(rl_kp, rl_ki, rl_kd))
            
        except ValueError:
            QMessageBox.critical(self, "Error", "Please enter valid numeric values")
            
    def validate_pid_params(self, kp, ki, kd):
        """验证PID参数合理性"""
        if kp < 0 or kp > 10:
            QMessageBox.warning(self, "Parameter Error", f"Kp值应在0-10范围内，当前值: {kp}")
            return False
        if ki < 0 or ki > 5:
            QMessageBox.warning(self, "Parameter Error", f"Ki值应在0-5范围内，当前值: {ki}")
            return False
        if kd < 0 or kd > 2:
            QMessageBox.warning(self, "Parameter Error", f"Kd值应在0-2范围内，当前值: {kd}")
            return False
        return True
        
    def send_rl_pid_params(self, kp, ki, kd):
        """发送后左电机PID参数"""
        kp_rl = int(kp * 100)
        ki_rl = int(ki * 100)
        kd_rl = int(kd * 100)
        
        data_rl = [1,  # Motor ID
                  (kp_rl >> 8) & 0xFF, kp_rl & 0xFF,
                  (ki_rl >> 8) & 0xFF, ki_rl & 0xFF,
                  (kd_rl >> 8) & 0xFF, kd_rl & 0xFF]
                  
        if self.send_packet(0x07, data_rl):
            self.add_log(f"设置后左电机PID: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
            QMessageBox.information(self, "Success", "PID parameters updated successfully")
        
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
        self.add_log("重置PID参数为默认值")
        
    def test_communication(self):
        """测试与STM32的通信"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "Please connect serial port first")
            return
            
        if self.send_packet(0x06, [1]):  # 启用PID
            QTimer.singleShot(100, lambda: self.send_packet(0x08, [0]))  # 请求前右电机状态
            QTimer.singleShot(200, lambda: self.send_packet(0x08, [1]))  # 请求后左电机状态
            self.add_log("发送通信测试命令")
        
    def request_pid_status(self):
        """请求PID状态"""
        if self.serial_port and self.serial_port.is_open and self.connection_stable:
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
        self.add_log("清除图表数据")
        
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
        
        # 计算数据速率
        current_time = time.time()
        if hasattr(self, 'last_packet_time'):
            time_diff = current_time - self.last_packet_time
            if time_diff > 0:
                rate = 1.0 / time_diff
                self.data_rate_label.setText(f"{rate:.1f} Hz")
        self.last_packet_time = current_time
        
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
        # 检查UI组件是否存在，避免访问已删除的对象
        if (self.fr_recommendation_label is None or 
            self.rl_recommendation_label is None or 
            self.global_advice_label is None or
            self.fr_stats_label is None or
            self.rl_stats_label is None):
            return
            
        try:
            # 分析前右电机
            fr_recommendation, fr_stats, fr_status = self.pid_analyzer.analyze_pid_performance_improved(
                self.pid_analysis_data['fr']
            )
            
            # 分析后左电机
            rl_recommendation, rl_stats, rl_status = self.pid_analyzer.analyze_pid_performance_improved(
                self.pid_analysis_data['rl']
            )
            
            # 更新前右电机显示
            if fr_stats:
                fr_stats_text = f"""误差: {fr_stats.get('current_error', 0):.2f}
稳态误差: {fr_stats.get('steady_state_error', 0):.2f}
超调: {fr_stats.get('overshoot', 0):.1f}%
振荡强度: {fr_stats.get('oscillation_intensity', 0):.3f}
符号变化: {fr_stats.get('significant_changes', 0)}次
目标: {fr_stats.get('target', 0):.1f}
实际: {fr_stats.get('actual', 0):.1f}
稳态: {'是' if fr_stats.get('is_steady_state', False) else '否'}"""
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
振荡强度: {rl_stats.get('oscillation_intensity', 0):.3f}
符号变化: {rl_stats.get('significant_changes', 0)}次
目标: {rl_stats.get('target', 0):.1f}
实际: {rl_stats.get('actual', 0):.1f}
稳态: {'是' if rl_stats.get('is_steady_state', False) else '否'}"""
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
            
        except RuntimeError as e:
            # 捕获RuntimeError，UI对象可能已经被删除
            print(f"UI object access error: {e}")
        except Exception as e:
            print(f"Unexpected error in update_pid_analysis: {e}")
        
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
        if self.global_advice_label is None:
            return
            
        try:
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
                
                fr_oscillation = fr_stats.get('oscillation_intensity', 0)
                rl_oscillation = rl_stats.get('oscillation_intensity', 0)
                
                if fr_oscillation > 2.0 and rl_oscillation > 2.0:
                    advice_parts.append("〰️ 检测到系统振荡，建议降低Kp和Ki参数")
                
                # 调参准备状态检查
                fr_ready = fr_stats.get('data_quality', '').startswith('数据质量良好')
                rl_ready = rl_stats.get('data_quality', '').startswith('数据质量良好')
                
                if fr_ready and rl_ready and not (self.auto_tuning_worker and self.auto_tuning_worker.isRunning()):
                    advice_parts.append("✅ 数据质量良好，可以开始自动调参")
            
            # 通用建议
            advice_parts.append("💡 调参原则：\n• Kp影响响应速度和稳态误差\n• Ki消除稳态误差但可能引起振荡\n• Kd减少超调和振荡\n• 建议先调Kp，再调Ki，最后调Kd")
            
            if not advice_parts:
                advice_parts.append("✅ 系统运行良好，PID参数调节合适")
            
            self.global_advice_label.setText("\n\n".join(advice_parts))
            
        except RuntimeError as e:
            print(f"Global advice update error: {e}")
    
    def closeEvent(self, event):
        """关闭窗口时的处理"""
        # 如果正在自动调参，询问是否停止
        if self.auto_tuning_worker and self.auto_tuning_worker.isRunning():
            reply = QMessageBox.question(self, "确认关闭", 
                                       "自动调参正在进行中，确定要关闭程序吗？", 
                                       QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                event.ignore()
                return
            
            # 停止调参工作线程
            self.auto_tuning_worker.stop()
            self.auto_tuning_worker.wait()
        
        # 停止所有定时器
        if hasattr(self, 'analysis_timer'):
            self.analysis_timer.stop()
        if hasattr(self, 'plot_timer'):
            self.plot_timer.stop()
        if hasattr(self, 'status_timer'):
            self.status_timer.stop()
        if hasattr(self, 'connection_check_timer'):
            self.connection_check_timer.stop()
            
        # 停止串口线程
        if self.serial_thread:
            self.serial_thread.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    # 设置应用信息
    app.setApplicationName("STM32 Robot Control System")
    app.setApplicationVersion("2.0")
    app.setOrganizationName("Robot Control Systems")
    
    window = RobotControlGUI()
    window.show()
    
    sys.exit(app.exec())