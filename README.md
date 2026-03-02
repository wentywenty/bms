# TWS BMS ROS 2 驱动程序

这是一个基于 ROS 2 的高性能、工业级电池管理系统 (BMS) 驱动程序，专门为广州明美新能源 (TWS) 的动力电池组设计。该驱动通过 Modbus RTU 协议与 BMS 进行串口通信。

## 🌟 核心特性

- **实时监控**：发布电压、电流、电量 (SOC)、最高温度等核心指标。
- **Python 接口**：集成 `pybind11`，支持在 Python 中直接调用 C++ 驱动逻辑。
- **静态信息提取**：启动时自动获取并打印序列号 (SN)、软件版本、硬件版本、健康度 (SOH) 和充放电循环次数。
- **软件关断功能**：通过 ROS 2 Service 远程控制放电 MOS 管。
- **二级安全防护**：内置过温保护逻辑（默认 55℃ 自动关断）。
- **诊断系统**：彩色终端日志告警。
- **高鲁棒性**：具备串口异常断连后的自动检测与重连机制。

## 🛠️ 硬件准备

1. **电池组**：TWS 13串锂电池组。
2. **连接线**：USB 转 TTL/RS485/RS232 转接头。
3. **波特率**：默认为 `115200`。

## 📥 安装与编译

### 1. 依赖安装
```bash
sudo apt install ros-$ROS_DISTRO-pybind11-vendor
```

### 2. 编译步骤
```bash
colcon build --packages-select bms
source install/setup.bash
```

## 🐍 Python API 使用示例

编译后，你可以在 Python 中直接操作 BMS：

```python
import tws_bms_api

# 初始化
bms = tws_bms_api.TwsBms("/dev/bms", 115200)
if bms.open():
    status = tws_bms_api.BatteryStatus()
    if bms.read_basic_info(status):
        print(f"Voltage: {status.voltage} V")
        print(f"Current: {status.current} A")
    
    # 获取序列号
    sn = bms.read_serial_number()
    print(f"SN: {sn}")
    
    # 软件关断
    # bms.set_discharge_output(False)
    bms.close_port()
```

## 🚀 ROS 2 部署

### 1. 配置 udev 规则 (推荐)
编辑 `99-bms.rules` 填入 ID 后：
```bash
sudo cp 99-bms.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. 启动驱动
```bash
ros2 run bms bms_node --ros-args --params-file src/bms/config/bms_config.yaml
```

## ⚙️ 参数配置 (`bms_config.yaml`)

| 参数名 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `port_name` | `/dev/ttyUSB0` | 串口设备路径 |
| `publish_rate` | `1.0` | 消息发布频率 (Hz) |
| `over_temp_threshold` | `55.0` | 二级过温保护阈值 (℃) |

## ⚠️ 故障排除

- **电压读数离谱**：当前代码针对 V2.5 协议（小端字序 32 位数据）进行了校准。
- **Python 模块找不到**：确保已 `source install/setup.bash`。
