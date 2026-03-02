#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tws_bms_api  # 导入 pybind11 生成的 C++ 模块
import time

def main():
    # 1. 初始化 BmsProtocol (串口路径, 波特率)
    # 根据文档，UART 默认波特率为 115200 [cite: 132]
    port_name = "/dev/ttyUSB0"
    baud_rate = 115200
    bms = tws_bms_api.BmsProtocol(port_name, baud_rate)

    print(f"正在尝试打开串口 {port_name}...")
    if not bms.open():
        print(f"错误: 无法打开串口 {port_name}！请检查权限或接线。")
        return

    print("串口打开成功！")

    try:
        # 2. 获取序列号 (SN)
        # 对应寄存器地址 9016，长度 32 字节 ASCII 
        sn = bms.read_serial_number()
        if sn:
            print(f"电池序列号: {sn}")
        else:
            print("警告: 无法读取序列号")

        # 3. 创建状态结构体对象并循环读取
        status = tws_bms_api.BatteryStatus()

        for i in range(5):
            print(f"\n--- 第 {i+1} 次采样 ---")
            
            # 读取基础信息 (寄存器 9000-9014) 
            # 包含：工作状态、总电压、电流、极值电压、极值温度等
            if bms.read_basic_info(status):
                # 协议规定电压单位为 mV, 电流为 mA 
                # 建议在 C++ 接口或此处进行单位转换
                print(f"工作状态: {status.work_state}") # 0:IDLE, 1:CHG, 2:DSG 
                print(f"总电压  : {status.voltage:.2f} V")
                print(f"总电流  : {status.current:.2f} A")
                
                # 温度换算逻辑：实际值 = 寄存器值 - 40 
                print(f"最高温度: {status.temperature:.1f} °C")
                print(f"单体电压: Max {status.max_cell_voltage:.3f}V / Min {status.min_cell_voltage:.3f}V")

                # 读取容量与健康度 (寄存器 9028-902A) 
                if bms.read_capacity_info(status):
                    # 文档定义 BasSoc 单位为 1% 
                    print(f"电量(SOC): {status.percentage:.1f} %")
                    print(f"健康(SOH): {status.soh:.1f} %")
            else:
                print("读取失败：未收到回复或 CRC 校验错误")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户终止测试")
    except Exception as e:
        print(f"运行异常: {e}")
    finally:
        # 4. 关闭串口
        bms.close_port()
        print("测试完成，串口已关闭")

if __name__ == "__main__":
    main()