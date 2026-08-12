#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ANO_DT高频振动数据采集工具 - 主程序
监听真实串口，解析ANO_DT协议，保存高频振动数据（0xF2帧）为CSV格式
"""

import os
import sys
import time
import signal
import logging
import argparse
import threading
import serial
import serial.tools.list_ports
from datetime import datetime
from typing import Optional, Dict, List, Tuple
import yaml

# 导入协议解析模块
from ano_protocol import (
    ANOProtocolParser,
    VibrationData,
    VibrationDataParser,
    ANOFunctionCode
)


class Forwarder:
    """数据转发器（用于将数据转发给ANO地面站）"""
    
    def __init__(self, config: Dict):
        """
        初始化转发器
        
        Args:
            config: 转发配置字典
        """
        self.config = config
        self.enabled = config.get('enabled', False)
        self.forward_type = config.get('type', 'virtual')
        
        # 目标串口（用于虚拟串口转发）
        self.target_serial = None
        self.target_port = config.get('virtual_port')
        self.target_baudrate = config.get('virtual_baudrate', 500000)
        
        # 过滤规则
        self.filter_config = config.get('filter', {})
        
        # 统计信息
        self.stats = {
            'frames_forwarded': 0,
            'bytes_forwarded': 0
        }
        
        # 初始化转发目标
        if self.enabled:
            self._initialize_target()
    
    def _initialize_target(self) -> bool:
        """初始化转发目标"""
        if self.forward_type == 'virtual' and self.target_port:
            try:
                import serial
                self.target_serial = serial.Serial(
                    port=self.target_port,
                    baudrate=self.target_baudrate,
                    timeout=0.1
                )
                logging.getLogger(__name__).info(f"转发目标串口已连接: {self.target_port}")
                return True
            except Exception as e:
                logging.getLogger(__name__).error(f"连接转发目标串口失败: {e}")
                return False
        elif self.forward_type == 'file':
            # 文件转发（用于调试）
            self.target_file = open(self.config.get('file_path', 'forwarded_data.bin'), 'ab')
            return True
        elif self.forward_type == 'network':
            # 网络转发（未来扩展）
            logging.getLogger(__name__).warning("网络转发尚未实现")
            return False
        else:
            logging.getLogger(__name__).warning(f"未知的转发类型: {self.forward_type}")
            return False
    
    def should_forward(self, frame) -> bool:
        """检查帧是否需要转发"""
        if not self.enabled:
            return False
        
        # 检查过滤规则
        if self.filter_config.get('include_all', True):
            return True
        
        # 检查功能码白名单
        func_codes = self.filter_config.get('function_codes', [])
        if func_codes and frame.function_code in func_codes:
            return True
        
        return False
    
    def forward(self, frame) -> bool:
        """
        转发帧数据
        
        Args:
            frame: ANOFrame对象
            
        Returns:
            转发是否成功
        """
        if not self.enabled:
            return False
        
        try:
            if self.forward_type == 'virtual' and self.target_serial:
                self.target_serial.write(frame.raw_frame)
                self.stats['frames_forwarded'] += 1
                self.stats['bytes_forwarded'] += len(frame.raw_frame)
                return True
            elif self.forward_type == 'file' and hasattr(self, 'target_file'):
                self.target_file.write(frame.raw_frame)
                self.target_file.flush()
                self.stats['frames_forwarded'] += 1
                self.stats['bytes_forwarded'] += len(frame.raw_frame)
                return True
            else:
                return False
        except Exception as e:
            logging.getLogger(__name__).error(f"转发数据失败: {e}")
            return False
    
    def close(self) -> None:
        """关闭转发器"""
        if self.target_serial:
            self.target_serial.close()
        if hasattr(self, 'target_file'):
            self.target_file.close()


class VibrationCapture:
    """高频振动数据采集器"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化采集器
        
        Args:
            config_path: 配置文件路径，如果为None则使用默认配置
        """
        # 默认配置
        self.config = {
            'serial': {
                'port': 'auto',
                'baudrate': 500000,
                'timeout': 0.1,
                'bytesize': 8,
                'stopbits': 1,
                'parity': 'N'
            },
            'data': {
                'save_dir': './data',
                'filename_format': 'vibration_{timestamp}.csv',
                'max_file_size_mb': 50,
                'max_duration_min': 60,
                'csv': {
                    'delimiter': ',',
                    'newline': '\n',
                    'write_header': True
                }
            },
            'forward': {
                'enabled': False,
                'type': 'virtual',
                'virtual_port': 'COM4',
                'virtual_baudrate': 500000,
                'filter': {
                    'include_all': True
                }
            },
            'logging': {
                'level': 'INFO',
                'console': True,
                'file': './logs/capture.log'
            },
            'monitor': {
                'update_interval_sec': 1,
                'display_stats': True
            }
        }
        
        # 加载配置文件（如果存在）
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r', encoding='utf-8') as f:
                user_config = yaml.safe_load(f)
                self._deep_update(self.config, user_config)
        
        # 初始化日志
        self._setup_logging()
        
        # 初始化组件
        self.serial_port = None
        self.parser = ANOProtocolParser()
        self.forwarder = Forwarder(self.config.get('forward', {}))
        self.running = False
        self.stats = {
            'total_frames': 0,
            'vibration_frames': 0,
            'invalid_frames': 0,
            'bytes_received': 0,
            'start_time': time.time(),
            'last_update_time': time.time()
        }
        
        # 文件相关
        self.csv_file = None
        self.csv_writer = None
        self.current_file_size = 0
        self.file_start_time = time.time()
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info("高频振动数据采集器初始化完成")
        
    def _deep_update(self, base: Dict, update: Dict) -> None:
        """递归更新字典"""
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._deep_update(base[key], value)
            else:
                base[key] = value
    
    def _setup_logging(self) -> None:
        """配置日志系统"""
        log_config = self.config['logging']
        log_level = getattr(logging, log_config['level'].upper())
        
        # 创建日志目录
        log_file = log_config.get('file')
        if log_file:
            log_dir = os.path.dirname(log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
        
        # 配置日志处理器
        handlers = []
        if log_config.get('console', True):
            handlers.append(logging.StreamHandler(sys.stdout))
        
        if log_file:
            handlers.append(logging.FileHandler(log_file, encoding='utf-8'))
        
        # 配置日志格式
        log_format = '%(asctime)s [%(levelname)s] %(message)s'
        date_format = '%Y-%m-%d %H:%M:%S'
        
        logging.basicConfig(
            level=log_level,
            format=log_format,
            datefmt=date_format,
            handlers=handlers
        )
        
        self.logger = logging.getLogger(__name__)
    
    def _signal_handler(self, signum, frame) -> None:
        """信号处理函数"""
        self.logger.info(f"收到信号 {signum}，正在停止采集...")
        self.stop()
    
    def auto_detect_serial_port(self) -> Optional[str]:
        """
        自动检测串口设备
        
        Returns:
            串口设备路径，如果未找到则返回None
        """
        self.logger.info("正在自动检测串口设备...")
        
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            self.logger.warning("未找到可用串口设备")
            return None
        
        self.logger.info(f"找到 {len(ports)} 个串口设备:")
        for port, desc, hwid in ports:
            self.logger.info(f"  - {port}: {desc} [{hwid}]")
        
        # 优先选择包含 "USB" 或 "ACM" 的设备
        preferred_keywords = ['USB', 'ACM', 'Serial', 'COM']
        for port, desc, _ in ports:
            desc_upper = desc.upper()
            if any(keyword in desc_upper for keyword in preferred_keywords):
                self.logger.info(f"选择设备: {port} ({desc})")
                return port
        
        # 如果没有匹配的，选择第一个
        first_port = ports[0][0]
        self.logger.info(f"选择第一个设备: {first_port}")
        return first_port
    
    def connect_serial(self, port: Optional[str] = None) -> bool:
        """
        连接串口
        
        Args:
            port: 串口设备路径，如果为None则使用配置中的port
        
        Returns:
            连接是否成功
        """
        serial_config = self.config['serial']
        
        # 确定端口
        if port is None:
            port = serial_config['port']
        
        if port == 'auto':
            port = self.auto_detect_serial_port()
            if port is None:
                self.logger.error("自动检测串口失败")
                return False
        
        # 串口参数
        baudrate = serial_config['baudrate']
        timeout = serial_config['timeout']
        bytesize = serial_config.get('bytesize', 8)
        stopbits = serial_config.get('stopbits', 1)
        parity = serial_config.get('parity', 'N')
        
        try:
            self.logger.info(f"正在连接串口 {port}，波特率 {baudrate}...")
            
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout
            )
            
            # 清空缓冲区
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.logger.info(f"串口连接成功: {port}")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"串口连接失败: {e}")
            return False
        except Exception as e:
            self.logger.error(f"连接串口时发生未知错误: {e}")
            return False
    
    def disconnect_serial(self) -> None:
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.logger.info("串口连接已关闭")
    
    def _generate_filename(self) -> str:
        """生成文件名"""
        filename_format = self.config['data']['filename_format']
        timestamp = int(time.time())
        now = datetime.now()
        
        # 替换变量
        filename = filename_format
        filename = filename.replace('{timestamp}', str(timestamp))
        filename = filename.replace('{date}', now.strftime('%Y%m%d'))
        filename = filename.replace('{time}', now.strftime('%H%M%S'))
        
        return filename
    
    def _open_csv_file(self) -> bool:
        """打开CSV文件"""
        data_config = self.config['data']
        save_dir = data_config['save_dir']
        
        # 创建保存目录
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        # 生成文件名
        filename = self._generate_filename()
        filepath = os.path.join(save_dir, filename)
        
        try:
            # 关闭现有文件（如果有）
            if self.csv_file:
                self.csv_file.close()
            
            # 打开新文件
            self.csv_file = open(filepath, 'w', encoding='utf-8')
            self.current_file_size = 0
            self.file_start_time = time.time()
            
            # 写入CSV头部（如果需要）
            if data_config['csv'].get('write_header', True):
                header = VibrationData.get_csv_header()
                self.csv_file.write(header + '\n')
                self.current_file_size += len(header) + 1
            
            self.logger.info(f"数据文件已创建: {filepath}")
            return True
            
        except IOError as e:
            self.logger.error(f"创建数据文件失败: {e}")
            return False
    
    def _close_csv_file(self) -> None:
        """关闭CSV文件"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.logger.info("数据文件已关闭")
    
    def _should_rotate_file(self) -> bool:
        """检查是否需要轮转文件"""
        data_config = self.config['data']
        
        # 检查文件大小
        max_size_mb = data_config.get('max_file_size_mb', 50)
        if self.current_file_size > max_size_mb * 1024 * 1024:
            self.logger.info(f"文件大小超过 {max_size_mb}MB，需要轮转")
            return True
        
        # 检查记录时长
        max_duration_min = data_config.get('max_duration_min', 60)
        if max_duration_min > 0:
            duration_sec = time.time() - self.file_start_time
            if duration_sec > max_duration_min * 60:
                self.logger.info(f"记录时长超过 {max_duration_min} 分钟，需要轮转")
                return True
        
        return False
    
    def _save_vibration_data(self, vibration_data: VibrationData) -> None:
        """
        保存振动数据到CSV文件
        
        Args:
            vibration_data: 振动数据对象
        """
        with self.lock:
            # 检查是否需要轮转文件
            if self.csv_file is None or self._should_rotate_file():
                if not self._open_csv_file():
                    self.logger.error("无法打开CSV文件，数据将丢失")
                    return
            
            try:
                # 写入CSV行
                csv_line = vibration_data.to_csv_row() + '\n'
                self.csv_file.write(csv_line)
                self.csv_file.flush()
                
                # 更新文件大小
                self.current_file_size += len(csv_line)
                
                # 更新统计信息
                self.stats['vibration_frames'] += 1
                
            except IOError as e:
                self.logger.error(f"写入数据文件失败: {e}")
    
    def _process_frame(self, frame) -> None:
        """
        处理解析完成的帧
        
        Args:
            frame: ANOFrame对象
        """
        # 更新总帧数统计
        self.stats['total_frames'] += 1
        
        # 检查是否为振动数据帧
        if frame.function_code == ANOFunctionCode.VIBRATION:
            vibration_data = VibrationDataParser.parse(frame)
            if vibration_data:
                self._save_vibration_data(vibration_data)
            else:
                self.logger.warning("振动数据帧解析失败")
                self.stats['invalid_frames'] += 1
        else:
            # 其他类型帧：转发给ANO地面站（如果启用）
            if self.forwarder.should_forward(frame):
                self.forwarder.forward(frame)
    
    def _update_stats_display(self) -> None:
        """更新并显示统计信息"""
        current_time = time.time()
        elapsed = current_time - self.stats['last_update_time']
        
        if elapsed < self.config['monitor']['update_interval_sec']:
            return
        
        total_elapsed = current_time - self.stats['start_time']
        
        # 计算速率
        total_frames = self.stats['total_frames']
        vibration_frames = self.stats['vibration_frames']
        bytes_received = self.stats['bytes_received']
        
        total_rate = total_frames / total_elapsed if total_elapsed > 0 else 0
        vibration_rate = vibration_frames / total_elapsed if total_elapsed > 0 else 0
        data_rate = bytes_received / total_elapsed if total_elapsed > 0 else 0
        
        # 显示统计信息
        if self.config['monitor']['display_stats']:
            print("\n" + "=" * 60)
            print("高频振动数据采集统计信息")
            print("=" * 60)
            print(f"运行时间: {total_elapsed:.1f} 秒")
            print(f"总帧数: {total_frames} 帧")
            print(f"振动帧数: {vibration_frames} 帧")
            print(f"无效帧数: {self.stats['invalid_frames']} 帧")
            print(f"转发帧数: {self.forwarder.stats['frames_forwarded']} 帧")
            print(f"总帧率: {total_rate:.1f} Hz")
            print(f"振动帧率: {vibration_rate:.1f} Hz")
            print(f"数据速率: {data_rate/1024:.1f} KB/s")
            print(f"数据文件大小: {self.current_file_size/1024/1024:.2f} MB")
            print("=" * 60 + "\n")
        
        # 重置上一周期统计
        self.stats['last_update_time'] = current_time
    
    def run(self) -> None:
        """运行数据采集主循环"""
        self.logger.info("开始数据采集...")
        
        # 打开CSV文件
        if not self._open_csv_file():
            self.logger.error("无法创建数据文件，采集终止")
            return
        
        self.running = True
        
        try:
            while self.running:
                # 更新统计显示
                self._update_stats_display()
                
                # 读取串口数据
                if self.serial_port and self.serial_port.is_open:
                    try:
                        # 读取可用数据
                        bytes_to_read = self.serial_port.in_waiting
                        if bytes_to_read > 0:
                            data = self.serial_port.read(bytes_to_read)
                            self.stats['bytes_received'] += len(data)
                            
                            # 解析数据帧
                            frames = self.parser.feed_bytes(data)
                            for frame in frames:
                                self._process_frame(frame)
                        
                        # 短暂休眠以减少CPU占用
                        time.sleep(0.001)
                        
                    except serial.SerialException as e:
                        self.logger.error(f"串口读取错误: {e}")
                        break
                    except Exception as e:
                        self.logger.error(f"处理数据时发生错误: {e}")
                        break
                else:
                    self.logger.error("串口未连接，采集终止")
                    break
        
        except KeyboardInterrupt:
            self.logger.info("用户中断采集")
        finally:
            self.stop()
    
    def stop(self) -> None:
        """停止数据采集"""
        self.running = False
        self._close_csv_file()
        self.disconnect_serial()
        self.forwarder.close()
        self.logger.info("数据采集已停止")


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='ANO_DT高频振动数据采集工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python vibration_capture.py                     # 自动检测串口
  python vibration_capture.py --port COM3         # 指定串口
  python vibration_capture.py --baudrate 500000   # 指定波特率
  python vibration_capture.py --config config.yaml # 使用配置文件
        """
    )
    
    parser.add_argument(
        '--port', '-p',
        type=str,
        help='串口设备路径 (例如: COM3, /dev/ttyUSB0)'
    )
    
    parser.add_argument(
        '--baudrate', '-b',
        type=int,
        default=500000,
        help='串口波特率 (默认: 500000)'
    )
    
    parser.add_argument(
        '--config', '-c',
        type=str,
        help='配置文件路径'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        help='数据保存目录 (覆盖配置文件中的设置)'
    )
    
    parser.add_argument(
        '--log-level',
        type=str,
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        default='INFO',
        help='日志级别 (默认: INFO)'
    )
    
    return parser.parse_args()


def main():
    """主函数"""
    args = parse_arguments()
    
    # 创建采集器实例
    try:
        capture = VibrationCapture(config_path=args.config)
    except Exception as e:
        print(f"初始化采集器失败: {e}")
        return 1
    
    # 覆盖命令行参数
    if args.port:
        capture.config['serial']['port'] = args.port
    
    if args.baudrate:
        capture.config['serial']['baudrate'] = args.baudrate
    
    if args.output:
        capture.config['data']['save_dir'] = args.output
    
    if args.log_level:
        capture.config['logging']['level'] = args.log_level
    
    # 连接串口
    if not capture.connect_serial(args.port):
        return 1
    
    # 运行采集
    capture.run()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())