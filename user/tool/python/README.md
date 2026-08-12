# ANO_DT高频振动数据采集工具

## 概述
这是一个Python代理中间件，用于从基于STM32+FreeRTOS的陆空两用无人机采集高频（200Hz）振动数据。工具监听真实串口，解析ANO_DT协议，提取振动数据（功能码0xF2）并保存为CSV格式，同时可透明转发其他数据帧给ANO地面站。

## 功能特性
- ✅ 自动检测和连接串口（Windows/Linux）
- ✅ ANO_DT协议解析（支持0x00-0xF2功能码）
- ✅ 高频振动数据（0xF2帧）CSV保存
- ✅ 实时统计信息（帧率、丢帧数）
- 🔄 数据转发给ANO地面站（可选）
- 🔄 实时可视化（可选扩展）

## 快速开始

### 1. 环境配置
```bash
# 安装Python 3.8+（如果未安装）
# 安装依赖
pip install -r requirements.txt
```

### 2. 基础使用
```bash
# 启动数据采集（自动检测串口）
python vibration_capture.py

# 指定串口和波特率
python vibration_capture.py --port COM3 --baudrate 500000

# 使用配置文件
python vibration_capture.py --config config.yaml
```

### 3. 配置文件说明
复制并修改`config.yaml.example`为`config.yaml`：
```yaml
serial:
  port: "auto"           # 自动检测或指定如"COM3"
  baudrate: 500000
  timeout: 0.1
  
data:
  save_dir: "./data"     # 数据保存目录
  filename_format: "vibration_{timestamp}.csv"
  max_file_size_mb: 50   # 文件轮转大小
  
logging:
  level: "INFO"
  file: "./logs/capture.log"
```

## 数据格式

### CSV列说明
| 列名 | 类型 | 描述 |
|------|------|------|
| timestamp_ms | uint32 | 时间戳（毫秒） |
| raw_gx | int16 | 原始陀螺仪X轴（°/s） |
| raw_gy | int16 | 原始陀螺仪Y轴（°/s） |
| raw_gz | int16 | 原始陀螺仪Z轴（°/s） |
| filt_gx | float | 滤波后角速度X轴（°/s） |
| filt_gy | float | 滤波后角速度Y轴（°/s） |
| filt_gz | float | 滤波后角速度Z轴（°/s） |
| pid_roll_out | float | 横滚环PID输出 |
| pid_pitch_out | float | 俯仰环PID输出 |
| pid_yaw_out | float | 航向环PID输出 |
| pid_roll_err | float | 横滚环PID误差 |
| pid_pitch_err | float | 俯仰环PID误差 |
| pid_yaw_err | float | 航向环PID误差 |

### ANO_DT协议扩展
- 功能码：0xF2（高频振动数据）
- 帧长度：51字节
- 发送频率：200Hz（5ms间隔）

## 模块说明

### ano_protocol.py
ANO_DT协议解析模块，包含：
- 帧状态机解析
- 校验和验证
- 功能码映射
- 数据类型转换（int16, float, uint32）

### vibration_capture.py
主采集脚本，包含：
- 串口管理（自动检测、连接、重连）
- 数据解析与保存
- 统计信息显示
- 错误处理与日志记录

## 开发计划

### 已完成
- ✅ 基础数据采集框架
- ✅ ANO_DT协议解析
- ✅ CSV数据保存

### 计划中
- 🔄 透明转发给ANO地面站
- 🔄 实时可视化（matplotlib）
- 🔄 频域分析（FFT）
- 🔄 Web界面

## 故障排除

### 常见问题
1. **串口连接失败**
   - 检查设备管理器中的COM端口
   - 确认波特率设置为500000
   - 确保ANO地面站未占用同一端口

2. **数据解析错误**
   - 检查校验和是否正确
   - 确认帧头为0xAA 0xAA
   - 验证功能码映射

3. **性能问题**
   - 降低日志级别（INFO→WARNING）
   - 减少实时显示更新频率
   - 使用二进制格式替代CSV

## 许可证
本项目基于MIT许可证开源。

## 联系方式
如有问题或建议，请通过项目仓库提交Issue。