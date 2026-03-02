# TWS BMS ROS 2 驱动程序

这是一个高性能、工业级的电池管理系统 (BMS) 驱动程序，专为广州明美新能源 (TWS) 13串动力电池组设计。采用 C++ 核心库、ROS 2 封装层和 Python API 的三层解耦架构。

## 🌟 核心特性

- **架构解耦**：底层 `BmsProtocol` 库完全独立，可在不带 ROS 的纯 C++ 或 Python 环境中使用。
- **高鲁棒性**：
    - **非阻塞读取**：基于 `poll` 的 100ms 超时控制，不卡死主线程。
    - **帧跳过机制**：偶发通信错误（CRC/超时）会自动跳过当前帧，保持数据流连续。
    - **自愈重连**：连续 5 帧失败会自动触发串口重置与重新初始化。
- **全方位监控**：
    - **实时数据**：电压、电流、SOC、最高单体温度。
    - **一致性分析**：实时监控最高/最低电芯极差 (Cell Gap)，识别电池老化。
    - **健康度日志**：启动时自动提取 SOH、充放电循环次数、版本号。
- **安全防护**：
    - **二级过温保护**：软件层级监控，超温自动关断放电 MOS。
    - **彩色诊断**：终端实时输出彩色状态日志与故障码翻译。

## 🛠️ 安装与编译

### 1. 依赖安装
```bash
sudo apt install ros-$ROS_DISTRO-pybind11-vendor
```

### 2. 编译
```bash
colcon build --packages-select bms
source install/setup.bash
```

## 🐍 Python API 使用

若在纯 Python 环境中使用，需将库路径加入 `PYTHONPATH`：

```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/bms/lib
python3
```

**示例脚本：**
```python
import tws_bms_api
bms = tws_bms_api.BmsProtocol("/dev/bms", 115200)
if bms.open():
    status = tws_bms_api.BatteryStatus()
    if bms.read_basic_info(status):
        print(f"电压: {status.voltage}V, SOC: {status.percentage*100}%")
    bms.close_port()
```

## 🚀 ROS 2 部署

### 1. 运行节点
```bash
ros2 run bms bms_node --ros-args --params-file src/bms/config/bms_config.yaml
```

### 2. 核心服务 (软件关断)
```bash
# 紧急切断负载输出
ros2 service call /set_discharge_output std_srvs/srv/SetBool "{data: false}"
```

## ⚙️ 关键配置 (`bms_config.yaml`)

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `publish_rate` | `1.0` | 消息发布与轮询频率 (Hz) |
| `cell_gap_warn_threshold` | `0.1` | 电芯极差报警阈值 (V)，建议 0.05-0.15 |
| `over_temp_threshold` | `55.0` | 软件二级过温保护阈值 (℃) |

## 📦 项目结构

- `bms_protocol.cpp`：底层 Modbus 协议实现（查表法 CRC）。
- `bms_driver.cpp`：ROS 2 节点封装。
- `pybind_module.cpp`：Python 接口定义。
- `bms_driver.hpp`：统一定义的数据结构。

# TWS BMS ROS 2 驱动 TODO 列表

## 🟢 已完成 (当前状态)
- [x] **底层解耦**: 核心协议封装为 `BmsProtocol` 类，不依赖 ROS 环境。
- [x] **Python 接口**: 集成 `pybind11` 生成 `tws_bms_api` 模块。
- [x] **高鲁棒性机制**:
    - 引入非阻塞 `poll` 读取，支持 100ms 超时控制。
    - 实现“帧跳过”逻辑：单次读取失败不中断 Topic 发布。
    - 实现“连续失败重置”：连续 5 次失败自动触发串口重连。
- [x] **电芯极差监控**: 实时计算 `Max - Min` 电压，并提供黄色阈值告警。
- [x] **彩色诊断系统**:
    - 绿色：存活状态（每 5s 概览）。
    - 黄色：跳过帧、低电量、极差大。
    - 红色：硬件故障、严重高温、串口重置。
- [x] **静态信息日志**: 启动时打印软件/硬件版本、SOH 及循环次数。
- [x] **软件关断服务**: 远程控制放电 MOS 管输出。
- [x] **参数化配置**: 支持发布频率、设计容量、保护阈值等动态调整。

## 🟡 近期优化 (待处理)
- [ ] **Python 示例**: 编写更完善的 Python 调试脚本示例。
