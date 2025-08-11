import sys
import os
import argparse
import time
import json
from pynput import keyboard
import numpy as np

# 设置路径
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.join(current_dir, '../common/robot_devices/motors')
sys.path.append(os.path.abspath(target_dir))

from feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# 常量
DEFAULT_PORT = "/dev/ttyACM0"
HALF_TURN_DEGREE = 180

# 舵机配置
MOTORS = {
    "left_wheel": (8, "sts3215"),
    "back_wheel": (9, "sts3215"),
    "right_wheel": (10, "sts3215"),
}

# 键盘控制映射
TELEOP_KEYS = {
    "forward": "w",
    "backward": "s",
    "rotate_left": "a",
    "rotate_right": "d",
    "left": "q",
    "right": "e",
}

# 全局变量
pressed_keys = {k: False for k in TELEOP_KEYS}
running = True

def degps_to_raw(degps: float) -> int:
    """将 deg/s 转换为舵机原始速度值"""
    steps_per_deg = 4096.0 / 360.0
    speed_in_steps = abs(degps) * steps_per_deg
    speed_int = int(round(speed_in_steps))
    if speed_int > 0x7FFF:
        speed_int = 0x7FFF
    return speed_int | 0x8000 if degps < 0 else speed_int

def body_to_wheel_raw(x_cmd: float, y_cmd: float, theta_cmd: float) -> dict:
    """
    将机身速度转换为舵机速度指令
    """
    wheel_radius = 0.05  # 轮子半径 (m)
    base_radius = 0.125  # 底盘半径 (m)
    max_raw = 3000  # 最大速度限制

    # 计算各轮速度 (m/s)
    angles = np.radians([300, 180, 60])  # 轮子安装角度
    velocity_vector = np.array([x_cmd, y_cmd, np.radians(theta_cmd)])
    m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])
    wheel_speeds = m.dot(velocity_vector) / wheel_radius  # rad/s
    wheel_degps = np.degrees(wheel_speeds)  # deg/s

    # 限制速度范围
    raw_values = [degps_to_raw(degps) for degps in wheel_degps]
    max_speed = max(abs(v) for v in raw_values)
    if max_speed > max_raw:
        scale = max_raw / max_speed
        raw_values = [int(v * scale) for v in raw_values]

    return {
        "left_wheel": raw_values[0],
        "back_wheel": raw_values[1],
        "right_wheel": raw_values[2],
    }

def on_press(key):
    """键盘按下事件"""
    try:
        key_char = key.char.lower()
        for action, ch in TELEOP_KEYS.items():
            if key_char == ch:
                pressed_keys[action] = True
    except AttributeError:
        if key == keyboard.Key.esc:
            global running
            running = False
            return False

def on_release(key):
    """键盘释放事件"""
    try:
        key_char = key.char.lower()
        for action, ch in TELEOP_KEYS.items():
            if key_char == ch:
                pressed_keys[action] = False
    except AttributeError:
        pass

def main(port: str):
    """主控制循环"""
    # 初始化舵机总线
    config = FeetechMotorsBusConfig(port=port, motors=MOTORS)
    motors_bus = FeetechMotorsBus(config)

    try:
        motors_bus.connect()
        print(f"Connected to port {port}")

        # 设置舵机模式
        motors_bus.write("Lock", 0)  # 解锁配置
        motors_bus.write("Mode", [1, 1, 1], MOTORS)  # 速度模式
        motors_bus.write("Torque_Enable", [1, 1, 1], MOTORS)  # 启用扭矩
        motors_bus.write("Maximum_Acceleration", [254, 254, 254], MOTORS)  # 最大加速度
        motors_bus.write("Lock", 1)  # 锁定配置
        print("Motors initialized in velocity mode.")

        # 启动键盘监听
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        print("Use WSAD to control, ESC to exit.")

        # 主控制循环
        while running:
            # 计算机身速度
            lin_speed = 0.2  # m/s
            ang_speed = 30.0  # deg/s
            y_cmd = lin_speed if pressed_keys["forward"] else -lin_speed if pressed_keys["backward"] else 0.0
            x_cmd = lin_speed if pressed_keys["left"] else -lin_speed if pressed_keys["right"] else 0.0
            theta_cmd = ang_speed if pressed_keys["rotate_left"] else -ang_speed if pressed_keys["rotate_right"] else 0.0

            # 转换为舵机速度并发送
            wheel_cmds = body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
            motors_bus.write("Goal_Speed", [
                wheel_cmds["left_wheel"],
                wheel_cmds["back_wheel"],
                wheel_cmds["right_wheel"],
            ], MOTORS)

            time.sleep(0.05)  # 控制频率 ~20Hz

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # 停止舵机并断开连接
        motors_bus.write("Goal_Speed", [0, 0, 0], MOTORS)
        motors_bus.disconnect()
        listener.stop()
        print("Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Feetech motors.")
    parser.add_argument("--port", type=str, default=DEFAULT_PORT, help="Serial port (default: /dev/ttyACM0)")
    args = parser.parse_args()
    main(args.port)
