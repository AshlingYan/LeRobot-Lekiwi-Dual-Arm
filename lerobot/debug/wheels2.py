#!/usr/bin/env python3
import sys
import time
import serial
from pynput import keyboard

# 硬件配置
PORT = "/dev/ttyACM0"
MOTOR_IDS = [8, 9, 10]

class MotorDriver:
    def __init__(self):
        print("正在初始化串口...")
        self.ser = serial.Serial(
            port=PORT,
            baudrate=1000000,  # 降为常用波特率
            timeout=0.2,
            bytesize=8,
            parity='N',
            stopbits=1
        )
        time.sleep(2)  # 等待串口稳定
        self._test_connection()

    def _test_connection(self):
        """基础通信测试"""
        print("\n=== 通信测试 ===")
        test_cmds = [
            b"\xff\xff\x01\x02\x01\xfb",  # SCS Ping指令
            b"\xff\xff\x01\x02\x15\xe7",  # SCS 读位置指令
            b"#1 P\r\n"                   # STS 指令
        ]
        
        for cmd in test_cmds:
            print(f"发送: {cmd.hex()}")
            self.ser.write(cmd)
            time.sleep(0.5)
            resp = self.ser.read_all()
            print(f"接收: {resp.hex() if resp else '无响应'}")

    def send_velocity(self, motor_id, speed):
        """发送速度指令 (单位: RPM)"""
        # 速度值处理 (-1000 ~ 1000)
        speed = max(-1000, min(1000, speed))
        speed_bytes = speed.to_bytes(2, 'little', signed=True)
        
        # SCS协议速度指令
        cmd = bytearray([
            0xFF, 0xFF,        # 帧头
            motor_id,          # 电机ID
            0x07,              # 指令长度
            0x20,              # 速度指令
            speed_bytes[0],    # 速度低字节
            speed_bytes[1],    # 速度高字节
            0                  # 临时CRC
        ])
        cmd[-1] = 0xFF - (sum(cmd[2:-1]) % 0x100)
        
        print(f"[CMD] 电机{motor_id} 速度:{speed} 指令:{cmd.hex()}")
        self.ser.write(cmd)
        time.sleep(0.1)
        return self.ser.read_all()

def main():
    print("\n=== 电机调试工具 ===")
    print("正在初始化...")
    driver = MotorDriver()
    
    # 测试速度控制
    print("\n=== 速度测试 ===")
    for speed in [300, 0, -300, 0]:
        print(f"\n设置速度: {speed}")
        for mid in MOTOR_IDS:
            resp = driver.send_velocity(mid, speed)
            print(f"电机{mid} 响应: {resp.hex() if resp else '无响应'}")
        time.sleep(1)
    
    print("\n测试完成，请检查：")
    print("1. 电机电源指示灯状态")
    print("2. 电机是否发出声音")
    print("3. USB转串口模块的TX/RX指示灯")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序终止")
    except Exception as e:
        print(f"\n错误发生: {str(e)}")
