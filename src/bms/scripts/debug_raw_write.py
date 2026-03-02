import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

def send(name, raw_hex):
    data = bytes.fromhex(raw_hex)
    full_data = data + calculate_crc(data)
    print(f"\n>>> {name}")
    print(f"发送: {full_data.hex(' ')}")
    
    try:
        with serial.Serial(PORT, BAUD, timeout=0.3) as ser: 
            ser.write(full_data)
            resp = ser.read(100)
            if resp:
                print(f"收到: {resp.hex(' ')}")
                return resp
            else:
                print("结果: 超时")
                return None
    except Exception as e:
        print(f"串口错误: {e}")
        return None

if __name__ == "__main__":
    print("=== TWS BMS 终极写指令探测工具 ===")
    
    # 策略 A: 尝试不同的功能码写入 9014
    send("A1: 0x06 写 0x9014 数据 00 00", "01 06 90 14 00 00")
    send("A2: 0x06 写 0x9015 (32位的高字) 数据 00 00", "01 06 90 15 00 00")
    
    # 策略 B: 尝试线圈控制 (Function 0x05) - 地址尝试 0 到 31
    for addr in [0, 1, 2, 3, 20]: # 20 是 0x14 的十进制
        send(f"B: 0x05 写线圈 {addr} (OFF)", f"01 05 00 {addr:02x} 00 00")
    
    # 策略 C: 尝试标准大端序 0x10 写入 32 位全零
    # [ID] [10] [90 14] [00 02] [04] [00 00 00 00]
    send("C: 0x10 写 32位 9014 (全零)", "01 10 90 14 00 02 04 00 00 00 00")

    # 策略 D: 探测是否有“写使能”寄存器 (尝试写入通用解锁码)
    # 很多 BMS 在 0x0000 或 0xFFFF 处有锁
    send("D: 尝试解锁写权限 (0x55AA to 0x0000)", "01 06 00 00 55 AA")

    print("\n[分析提示] 如果以上全部超时，说明 BMS 当前处于‘硬锁定’状态。")
    print("请确认：BMS 物理面板上是否有开关？或者是否有上位机软件正在占用串口？")
