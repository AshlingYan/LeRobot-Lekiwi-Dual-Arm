import sys
import os
import argparse
import time
import numpy as np
import serial
from pynput import keyboard

# 添加模块路径
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.join(current_dir, '../common/robot_devices/motors')
sys.path.append(os.path.abspath(target_dir))

from feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# 常量定义
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1000000  # 1Mbps

# ---------------------- 运动控制函数 ----------------------
def degps_to_raw(degps: float) -> int:
    steps_per_deg = 4096.0 / 360.0
    speed_in_steps = abs(degps) * steps_per_deg
    speed_int = int(round(speed_in_steps))
    if speed_int > 0x7FFF:
        speed_int = 0x7FFF
    return speed_int | 0x8000 if degps < 0 else speed_int & 0x7FFF

def body_to_wheel_raw(x_cmd, y_cmd, theta_cmd, max_raw=3000):
    theta_rad = np.radians(theta_cmd)
    velocity_vector = np.array([x_cmd, y_cmd, theta_rad])
    angles = np.radians([300, 180, 60])
    m = np.array([[np.cos(a), np.sin(a), 0.125] for a in angles])
    wheel_degps = (m @ velocity_vector) / 0.05 * (180/np.pi)
    wheel_raw = [degps_to_raw(deg) for deg in wheel_degps]
    return {
        "left_wheel": wheel_raw[0],
        "back_wheel": wheel_raw[1], 
        "right_wheel": wheel_raw[2]
    }

# ---------------------- 键盘控制 ----------------------
teleop_keys = {
    "forward": "w", "backward": "s",
    "rotate_left": "a", "rotate_right": "d",
    "left": "q", "right": "e"
}
pressed_keys = {k: False for k in teleop_keys}

def on_press(key):
    try:
        for action, ch in teleop_keys.items():
            if key.char == ch:
                pressed_keys[action] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        for action, ch in teleop_keys.items():
            if key.char == ch:
                pressed_keys[action] = False
    except AttributeError:
        pass

# ---------------------- 主程序 ----------------------
if __name__ == "__main__":
    # 1. 初始化配置
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default=DEFAULT_PORT)
    args = parser.parse_args()

    motors = {
        "left_wheel": (8, "sts3215"),
        "back_wheel": (9, "sts3215"),
        "right_wheel": (10, "sts3215"),
    }

    # 2. 手动创建串口连接
    try:
        print(f"\n正在初始化串口 {args.port} @ {DEFAULT_BAUDRATE}...")
        ser = serial.Serial(
            port=args.port,
            baudrate=DEFAULT_BAUDRATE,
            timeout=0.1,
            bytesize=8,
            parity='N',
            stopbits=1
        )
        print("[SUCCESS] 串口已创建")
    except Exception as e:
        print(f"[ERROR] 串口初始化失败: {str(e)}")
        exit(1)

    # 3. 初始化电机总线并注入串口对象
    config = FeetechMotorsBusConfig(
        port=args.port,
        motors=motors
    )
    
    motors_bus = FeetechMotorsBus(config)
    
    # 4. 通过反射强制注入串口对象
    if not hasattr(motors_bus, 'serial'):
        print("通过反射注入串口对象...")
        motors_bus.serial = ser
        motors_bus.port_handler = ser  # 有些库使用不同名称
        motors_bus._serial = ser      # 有些库使用私有变量

    try:
        # 5. 连接电机
        print("\n尝试连接电机...")
        if hasattr(motors_bus, 'connect'):
            motors_bus.connect()
        else:
            print("⚠️ 电机总线没有connect方法，使用直接通信")
        
        # 6. 设置为速度模式
        motors_bus.write("Lock", 0)
        motors_bus.write("Mode", [1, 1, 1], motors)
        motors_bus.write("Lock", 1)
        print("电机模式: 速度控制")

        # 7. 启动键盘监听
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        # 8. 主控制循环
        print("\n控制指令:")
        print("W:前进 S:后退")
        print("A:左转 D:右转")
        print("Q:左移 E:右移")
        print("Ctrl+C 退出\n")
        
        while True:
            # 计算目标速度
            y_cmd = 0.2 if pressed_keys["forward"] else -0.2 if pressed_keys["backward"] else 0
            x_cmd = 0.2 if pressed_keys["left"] else -0.2 if pressed_keys["right"] else 0
            theta_cmd = 30 if pressed_keys["rotate_left"] else -30 if pressed_keys["rotate_right"] else 0

            # 发送速度命令
            wheel_cmds = body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
            raw_values = [wheel_cmds[n] for n in ["left_wheel", "back_wheel", "right_wheel"]]
            
            # 直接使用串口发送数据（备用方案）
            try:
                motors_bus.write("Goal_Speed", raw_values, list(motors.keys()))
            except Exception as e:
                print(f"通过总线写入失败，尝试直接写入: {str(e)}")
                for motor_id, value in zip([8,9,10], raw_values):
                    cmd = f"#{motor_id} P{value}\r\n".encode()
                    ser.write(cmd)
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n正在停止电机...")
        motors_bus.write("Goal_Speed", [0, 0, 0], list(motors.keys()))
    except Exception as e:
        print(f"\n[ERROR] 发生错误: {str(e)}")
    finally:
        listener.stop()
        ser.close()
        print("程序已安全退出")
