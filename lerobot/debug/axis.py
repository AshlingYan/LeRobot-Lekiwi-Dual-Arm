#!/usr/bin/env python3
# lift_axis_teleop.py
"""
Keyboard tele-op for a single lift axis driven by a Feetech servo.
 *  Z  – lift up     (positive speed)
 *  X  – lift down   (negative speed)
Release key → motor stops.
"""

import os, sys, time, argparse
from pynput import keyboard

# ---------- 1.  把 feetech 驱动所在路径加入搜索路径 ----------
current_dir = os.path.dirname(os.path.abspath(__file__))
motors_dir  = os.path.join(current_dir, "../common/robot_devices/motors")
sys.path.append(os.path.abspath(motors_dir))

from feetech import FeetechMotorsBus                      # noqa
from lerobot.common.robot_devices.motors.configs import (
    FeetechMotorsBusConfig,
)

# ---------- 2.  与 wheels.py 保持一致的 Raw ↔︎ deg/s 编码 ----------
STEPS_PER_DEG = 4096.0 / 360.0            # 约 11.377 ticks / deg

def degps_to_raw(degps: float) -> int:     # :contentReference[oaicite:0]{index=0}
    speed  = abs(degps) * STEPS_PER_DEG
    speed  = int(round(min(speed, 0x7FFF)))   # 饱和
    return (speed | 0x8000) if degps < 0 else speed

# ---------- 3.  键盘状态 ----------
TELEOP_KEYS  = {"up": "z", "down": "x"}    # 可自行改键
pressed      = {"up": False, "down": False}

def on_press(key):
    try:
        if key.char == TELEOP_KEYS["up"]:
            pressed["up"] = True
        elif key.char == TELEOP_KEYS["down"]:
            pressed["down"] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char == TELEOP_KEYS["up"]:
            pressed["up"] = False
        elif key.char == TELEOP_KEYS["down"]:
            pressed["down"] = False
    except AttributeError:
        pass

# ---------- 4.  主程序 ----------
def main():
    parser = argparse.ArgumentParser(description="Lift-axis tele-op")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="USB-TTL port, e.g. /dev/ttyACM0")
    parser.add_argument("--speed", type=float, default=180.0,
                        help="Lift speed in deg/s (default 90)")
    args = parser.parse_args()

    motor_id  = 11                          # 升降轴舵机 ID
    config    = FeetechMotorsBusConfig(
        port   = args.port,
        motors = { "lift_axis": (motor_id, "sts3215") },
    )
    bus       = FeetechMotorsBus(config)

    # -- 连接 & 设成轮模式(速度模式, value=1) --
    bus.connect()
    print(f"[INFO] Connected on {args.port}")
    bus.write("Lock", 0)
    bus.write("Mode", 1, ["lift_axis"])    # 1 = wheel / velocity
    bus.write("Lock", 1)

    # -- 启动键盘监听 --
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    print("[INFO] Ready.  Z ↑ / X ↓ / Ctrl-C quit.")

    try:
        while True:
            # 根据按键决定速度符号
            if pressed["up"] and not pressed["down"]:
                raw = degps_to_raw( args.speed)
            elif pressed["down"] and not pressed["up"]:
                raw = degps_to_raw(-args.speed)
            else:
                raw = 0

            bus.write("Goal_Speed", raw, ["lift_axis"])
            time.sleep(0.03)               # ~33 Hz 更新

    except KeyboardInterrupt:
        print("\n[INFO] Stopping motor...")
        bus.write("Goal_Speed", 0, ["lift_axis"])
        listener.stop()

if __name__ == "__main__":
    main()
