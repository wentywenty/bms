#!/usr/bin/env python3
import sys
import os
import time
import glob

def setup_path():
    current_dir = os.path.abspath(os.path.dirname(__file__))
    ws_root = current_dir
    while ws_root != "/" and not os.path.exists(os.path.join(ws_root, "install")):
        ws_root = os.path.dirname(ws_root)
    if ws_root == "/": return None
    search_pattern = os.path.join(ws_root, "install/bms/**/tws_bms_api*.so")
    found_files = glob.glob(search_pattern, recursive=True)
    if not found_files: return None
    lib_dir = os.path.dirname(found_files[0])
    if lib_dir not in sys.path: sys.path.insert(0, lib_dir)
    return lib_dir

lib_path = setup_path()
try:
    import tws_bms_api
except ImportError as e:
    print(f"\033[1;31m[错误] 导入失败: {e}\033[0m")
    sys.exit(1)

def main():
    PORT = "/dev/ttyUSB0" 
    BAUD = 115200
    print(f"\033[1;32m=== TWS BMS Python API 测试工具 (库路径: {lib_path}) ===\033[0m")
    bms = tws_bms_api.BmsProtocol(PORT, BAUD)

    if not bms.open():
        print("\033[1;31m错误: 无法打开串口！\033[0m")
        return

    try:
        sn = bms.read_serial_number()
        print(f"电池序列号: {sn}")
        status = tws_bms_api.BatteryStatus()
        
        if bms.read_version_info(status):
            print(f"固件版本: 0x{status.sw_version:04X} | 硬件版本: 0x{status.hw_version:04X}")
            print(f"健康度 (SOH): {status.soh}% | 循环次数: {status.cycles}")

        count = 0
        while True:
            count += 1
            if bms.read_basic_info(status) and bms.read_capacity_info(status):
                print(f"\n--- 第 {count} 次采样 ---")
                print(f"总电压  : {status.voltage:.2f} V")
                print(f"总电流  : {status.current:.2f} A")
                print(f"最高温度: {status.temperature:.1f} °C")
                print(f"当前电量: {status.percentage * 100:.1f} %")
                print(f"容量统计: 剩余 {status.charge:.2f} Ah / 当前满充 {status.capacity:.2f} Ah")
                print(f"电芯极值: Max {status.max_cell_voltage:.3f}V / Min {status.min_cell_voltage:.3f}V (极差: {status.max_cell_voltage - status.min_cell_voltage:.3f}V)")
            else:
                print("\033[1;33m数据读取失败，跳过...\033[0m")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        bms.close_port()
        print("\n\033[1;32m测试完成，串口已关闭\033[0m")

if __name__ == "__main__":
    main()
