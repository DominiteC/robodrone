#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
振动数据采集工具测试脚本
测试协议解析、数据保存和转发功能
"""

import os
import sys
import time
import tempfile
import shutil
from unittest.mock import Mock, patch
import serial

# 添加当前目录到模块搜索路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ano_protocol import ANOProtocolParser, VibrationDataParser, ANOFunctionCode, VibrationData


class MockSerial:
    """模拟串口类，用于测试"""
    
    def __init__(self, test_frames, port='COM_TEST', baudrate=500000):
        self.port = port
        self.baudrate = baudrate
        self.test_frames = test_frames
        self.frame_index = 0
        self.is_open = True
        self.timeout = 0.1
        
    def read(self, size=1):
        """模拟读取数据"""
        if self.frame_index >= len(self.test_frames):
            return b''
        
        frame = self.test_frames[self.frame_index]
        self.frame_index += 1
        return frame
    
    @property
    def in_waiting(self):
        """模拟等待读取的字节数"""
        if self.frame_index < len(self.test_frames):
            return len(self.test_frames[self.frame_index])
        return 0
    
    def reset_input_buffer(self):
        """模拟清空输入缓冲区"""
        pass
    
    def reset_output_buffer(self):
        """模拟清空输出缓冲区"""
        pass
    
    def close(self):
        self.is_open = False


def test_protocol_parser():
    """测试协议解析器"""
    print("测试协议解析器...")
    
    # 使用ano_protocol中的测试帧
    from ano_protocol import test_frame
    
    parser = ANOProtocolParser()
    frames = parser.feed_bytes(test_frame)
    
    assert len(frames) == 1, f"应解析出1帧，实际解析出{len(frames)}帧"
    
    frame = frames[0]
    assert frame.function_code == ANOFunctionCode.VIBRATION, f"功能码应为0xF2，实际为0x{frame.function_code:02X}"
    assert frame.is_valid, "帧校验和无效"
    
    # 解析振动数据
    vib_data = VibrationDataParser.parse(frame)
    assert vib_data is not None, "振动数据解析失败"
    
    print(f"  时间戳: {vib_data.timestamp_ms}")
    print(f"  原始陀螺仪: ({vib_data.raw_gx}, {vib_data.raw_gy}, {vib_data.raw_gz})")
    print(f"  滤波角速度: ({vib_data.filt_gx:.2f}, {vib_data.filt_gy:.2f}, {vib_data.filt_gz:.2f})")
    
    print("[PASS] 协议解析器测试通过")


def test_vibration_data_csv():
    """测试振动数据CSV保存"""
    print("测试振动数据CSV保存...")
    
    from ano_protocol import test_frame, VibrationData
    
    parser = ANOProtocolParser()
    frames = parser.feed_bytes(test_frame)
    frame = frames[0]
    vib_data = VibrationDataParser.parse(frame)
    
    # 测试CSV头部
    header = vib_data.get_csv_header()
    expected_header = 'timestamp_ms,raw_gx,raw_gy,raw_gz,filt_gx,filt_gy,filt_gz,' \
                      'pid_roll_out,pid_pitch_out,pid_yaw_out,' \
                      'pid_roll_err,pid_pitch_err,pid_yaw_err'
    assert header == expected_header, f"CSV头部不匹配:\n期望: {expected_header}\n实际: {header}"
    
    # 测试CSV行
    csv_row = vib_data.to_csv_row()
    # 简单检查行长度和字段数
    fields = csv_row.split(',')
    assert len(fields) == 13, f"CSV行应有13个字段，实际有{len(fields)}个"
    
    print(f"  CSV头部: {header}")
    print(f"  CSV行示例: {csv_row}")
    print("[PASS] 振动数据CSV保存测试通过")


def test_capture_with_mock_serial():
    """使用模拟串口测试采集器"""
    print("使用模拟串口测试采集器...")
    
    # 创建临时目录用于测试
    temp_dir = tempfile.mkdtemp(prefix='vibration_test_')
    print(f"  使用临时目录: {temp_dir}")
    
    try:
        # 导入采集器（需要在模拟串口之后）
        from vibration_capture import VibrationCapture
        
        # 创建测试配置
        test_config = {
            'serial': {
                'port': 'COM_TEST',
                'baudrate': 500000,
                'timeout': 0.1
            },
            'data': {
                'save_dir': temp_dir,
                'filename_format': 'test_{timestamp}.csv',
                'max_file_size_mb': 1,  # 小文件便于测试
                'csv': {
                    'write_header': True
                }
            },
            'logging': {
                'level': 'WARNING',  # 减少日志输出
                'console': False
            },
            'monitor': {
                'display_stats': False
            }
        }
        
        # 创建测试帧（来自ano_protocol）
        from ano_protocol import test_frame
        
        # 创建模拟串口对象
        mock_serial = MockSerial([test_frame] * 10)  # 10个测试帧
        
        # 创建采集器实例（不加载配置文件）
        with patch('serial.Serial') as mock_serial_class:
            mock_serial_class.return_value = mock_serial
            
            # 创建采集器
            capture = VibrationCapture(config_path=None)
            capture.config = test_config
            
            # 手动设置模拟串口
            capture.serial_port = mock_serial
            
            # 运行短暂时间
            capture.running = True
            capture._open_csv_file()
            
            # 手动处理一些数据
            for _ in range(5):
                if mock_serial.in_waiting > 0:
                    data = mock_serial.read()
                    frames = capture.parser.feed_bytes(data)
                    for frame in frames:
                        capture._process_frame(frame)
                time.sleep(0.01)
            
            # 停止采集
            capture.stop()
            
            # 检查是否生成了数据文件
            csv_files = [f for f in os.listdir(temp_dir) if f.endswith('.csv')]
            assert len(csv_files) > 0, "未生成CSV数据文件"
            
            # 读取CSV文件内容
            csv_file = os.path.join(temp_dir, csv_files[0])
            with open(csv_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 检查CSV文件格式
            lines = content.strip().split('\n')
            assert len(lines) >= 2, f"CSV文件应至少包含头部和数据行，实际行数: {len(lines)}"
            
            # 检查头部
            assert lines[0] == VibrationData.get_csv_header(), "CSV头部不匹配"
            
            # 检查数据行数（应至少有一行数据）
            data_lines = lines[1:]
            assert len(data_lines) > 0, "CSV文件中没有数据行"
            
            print(f"  生成CSV文件: {csv_file}")
            print(f"  数据行数: {len(data_lines)}")
            print("[PASS] 模拟串口采集测试通过")
            
    finally:
        # 清理临时目录
        shutil.rmtree(temp_dir, ignore_errors=True)
        print(f"  清理临时目录: {temp_dir}")


def test_forwarder():
    """测试转发功能"""
    print("测试转发功能...")
    
    from vibration_capture import Forwarder
    
    # 测试配置
    config = {
        'enabled': True,
        'type': 'file',
        'file_path': 'test_forward.bin'
    }
    
    # 创建转发器
    forwarder = Forwarder(config)
    assert forwarder.enabled, "转发器应启用"
    
    # 创建一个模拟帧
    from ano_protocol import test_frame, ANOFrame
    parser = ANOProtocolParser()
    frames = parser.feed_bytes(test_frame)
    test_frame_obj = frames[0]
    
    # 测试转发
    success = forwarder.forward(test_frame_obj)
    assert success, "转发失败"
    
    # 检查统计信息
    assert forwarder.stats['frames_forwarded'] == 1, f"转发帧数统计错误: {forwarder.stats['frames_forwarded']}"
    
    # 关闭转发器以释放文件句柄
    forwarder.close()
    
    # 清理测试文件
    if os.path.exists('test_forward.bin'):
        os.remove('test_forward.bin')
    
    print("[PASS] 转发功能测试通过")


def main():
    """运行所有测试"""
    print("=" * 60)
    print("高频振动数据采集工具测试套件")
    print("=" * 60)
    
    tests = [
        test_protocol_parser,
        test_vibration_data_csv,
        test_forwarder,
        test_capture_with_mock_serial,
    ]
    
    passed = 0
    failed = 0
    
    for test_func in tests:
        try:
            test_func()
            passed += 1
        except Exception as e:
            failed += 1
            print(f"[FAIL] {test_func.__name__} 测试失败: {e}")
            import traceback
            traceback.print_exc()
    
    print("=" * 60)
    print(f"测试完成: {passed} 通过, {failed} 失败")
    print("=" * 60)
    
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())