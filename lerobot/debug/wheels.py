# test_motors.py
import sys
import os
import argparse
import time
import ast
import json

current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.join(current_dir, '../common/robot_devices/motors')
sys.path.append(os.path.abspath(target_dir))

from feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from pynput import keyboard
import numpy as np


DEFAULT_PORT = "/dev/ttyACM0"
HALF_TURN_DEGREE = 180


@staticmethod
def degps_to_raw(degps: float) -> int:
    steps_per_deg = 4096.0 / 360.0
    speed_in_steps = abs(degps) * steps_per_deg
    speed_int = int(round(speed_in_steps))
    if speed_int > 0x7FFF:
        speed_int = 0x7FFF
    if degps < 0:
        return speed_int | 0x8000
    else:
        return speed_int & 0x7FFF

@staticmethod
def raw_to_degps(raw_speed: int) -> float:
    steps_per_deg = 4096.0 / 360.0
    magnitude = raw_speed & 0x7FFF
    degps = magnitude / steps_per_deg
    if raw_speed & 0x8000:
        degps = -degps
    return degps

def body_to_wheel_raw(
    self,
    x_cmd: float,
    y_cmd: float,
    theta_cmd: float,
    wheel_radius: float = 0.05,
    base_radius: float = 0.125,
    max_raw: int = 3000,
) -> dict:
    """
    Convert desired body-frame velocities into wheel raw commands.

    Parameters:
        x_cmd      : Linear velocity in x (m/s).
        y_cmd      : Linear velocity in y (m/s).
        theta_cmd  : Rotational velocity (deg/s).
        wheel_radius: Radius of each wheel (meters).
        base_radius : Distance from the center of rotation to each wheel (meters).
        max_raw    : Maximum allowed raw command (ticks) per wheel.

    Returns:
        A dictionary with wheel raw commands:
            {"left_wheel": value, "back_wheel": value, "right_wheel": value}.

    Notes:
        - Internally, the method converts theta_cmd to rad/s for the kinematics.
        - The raw command is computed from the wheels angular speed in deg/s
        using degps_to_raw(). If any command exceeds max_raw, all commands
        are scaled down proportionally.
    """
    # Convert rotational velocity from deg/s to rad/s.
    theta_rad = theta_cmd * (np.pi / 180.0)
    # Create the body velocity vector [x, y, theta_rad].
    velocity_vector = np.array([x_cmd, y_cmd, theta_rad])

    # Define the wheel mounting angles (defined from y axis cw)
    angles = np.radians(np.array([300, 180, 60]))
    # Build the kinematic matrix: each row maps body velocities to a wheel’s linear speed.
    # The third column (base_radius) accounts for the effect of rotation.
    m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

    # Compute each wheel’s linear speed (m/s) and then its angular speed (rad/s).
    wheel_linear_speeds = m.dot(velocity_vector)
    wheel_angular_speeds = wheel_linear_speeds / wheel_radius

    # Convert wheel angular speeds from rad/s to deg/s.
    wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

    # Scaling
    steps_per_deg = 4096.0 / 360.0
    raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
    max_raw_computed = max(raw_floats)
    if max_raw_computed > max_raw:
        scale = max_raw / max_raw_computed
        wheel_degps = wheel_degps * scale

    # Convert each wheel’s angular speed (deg/s) to a raw integer.
    wheel_raw = [degps_to_raw(deg) for deg in wheel_degps]

    return {"left_wheel": wheel_raw[0], "back_wheel": wheel_raw[1], "right_wheel": wheel_raw[2]}

def wheel_raw_to_body(
    self, wheel_raw: dict, wheel_radius: float = 0.05, base_radius: float = 0.125
) -> tuple:
    """
    Convert wheel raw command feedback back into body-frame velocities.

    Parameters:
        wheel_raw   : Dictionary with raw wheel commands (keys: "left_wheel", "back_wheel", "right_wheel").
        wheel_radius: Radius of each wheel (meters).
        base_radius : Distance from the robot center to each wheel (meters).

    Returns:
        A tuple (x_cmd, y_cmd, theta_cmd) where:
            x_cmd      : Linear velocity in x (m/s).
            y_cmd      : Linear velocity in y (m/s).
            theta_cmd  : Rotational velocity in deg/s.
    """
    # Extract the raw values in order.
    raw_list = [
        int(wheel_raw.get("left_wheel", 0)),
        int(wheel_raw.get("back_wheel", 0)),
        int(wheel_raw.get("right_wheel", 0)),
    ]

    # Convert each raw command back to an angular speed in deg/s.
    wheel_degps = np.array([raw_to_degps(r) for r in raw_list])
    # Convert from deg/s to rad/s.
    wheel_radps = wheel_degps * (np.pi / 180.0)
    # Compute each wheel’s linear speed (m/s) from its angular speed.
    wheel_linear_speeds = wheel_radps * wheel_radius

    # Define the wheel mounting angles (defined from y axis cw)
    angles = np.radians(np.array([300, 180, 60]))
    m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

    # Solve the inverse kinematics: body_velocity = M⁻¹ · wheel_linear_speeds.
    m_inv = np.linalg.inv(m)
    velocity_vector = m_inv.dot(wheel_linear_speeds)
    x_cmd, y_cmd, theta_rad = velocity_vector
    theta_cmd = theta_rad * (180.0 / np.pi)
    return (x_cmd, y_cmd, theta_cmd)

def read_velocity(self):
    """
    Reads the raw speeds for all wheels. Returns a dictionary with motor names:
    """
    raw_speeds = self.motor_bus.read("Present_Speed", self.motor_ids)
    print(f"Raw speeds: {raw_speeds}")

    return {
        "left_wheel": int(raw_speeds[0]),
        "back_wheel": int(raw_speeds[1]),
        "right_wheel": int(raw_speeds[2]),
    }

def set_velocity(self, command_speeds):
    """
    Sends raw velocity commands (16-bit encoded values) directly to the motor bus.
    The order of speeds must correspond to self.motor_ids.
    """
    self.motor_bus.write("Goal_Speed", command_speeds, self.motor_ids)


# def on_press(self, key):
#     try:
#         # Movement
#         if key.char == self.teleop_keys["forward"]:
#             self.pressed_keys["forward"] = True
#         elif key.char == self.teleop_keys["backward"]:
#             self.pressed_keys["backward"] = True
#         elif key.char == self.teleop_keys["left"]:
#             self.pressed_keys["left"] = True
#         elif key.char == self.teleop_keys["right"]:
#             self.pressed_keys["right"] = True
#         elif key.char == self.teleop_keys["rotate_left"]:
#             self.pressed_keys["rotate_left"] = True
#         elif key.char == self.teleop_keys["rotate_right"]:
#             self.pressed_keys["rotate_right"] = True

#         # Quit teleoperation
#         elif key.char == self.teleop_keys["quit"]:
#             self.running = False
#             return False

#         # Speed control
#         elif key.char == self.teleop_keys["speed_up"]:
#             self.speed_index = min(self.speed_index + 1, 2)
#             print(f"Speed index increased to {self.speed_index}")
#         elif key.char == self.teleop_keys["speed_down"]:
#             self.speed_index = max(self.speed_index - 1, 0)
#             print(f"Speed index decreased to {self.speed_index}")

#     except AttributeError:
#         # e.g., if key is special like Key.esc
#         if key == keyboard.Key.esc:
#             self.running = False
#             return False

# def on_release(self, key):
#     try:
#         if hasattr(key, "char"):
#             if key.char == self.teleop_keys["forward"]:
#                 self.pressed_keys["forward"] = False
#             elif key.char == self.teleop_keys["backward"]:
#                 self.pressed_keys["backward"] = False
#             elif key.char == self.teleop_keys["left"]:
#                 self.pressed_keys["left"] = False
#             elif key.char == self.teleop_keys["right"]:
#                 self.pressed_keys["right"] = False
#             elif key.char == self.teleop_keys["rotate_left"]:
#                 self.pressed_keys["rotate_left"] = False
#             elif key.char == self.teleop_keys["rotate_right"]:
#                 self.pressed_keys["rotate_right"] = False
#     except AttributeError:
#         pass

def on_press(key):
    try:
        for action, ch in teleop_keys.items():
            if key.char == ch:
                pressed_keys[action] = True
    except AttributeError:
        pass  # 忽略特殊键

def on_release(key):
    try:
        for action, ch in teleop_keys.items():
            if key.char == ch:
                pressed_keys[action] = False
    except AttributeError:
        pass



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Feetech motors.")
    parser.add_argument(
        "--port",
        type=str,
        default=DEFAULT_PORT,
        help="Serial port for the motor bus (default: %s)" % DEFAULT_PORT,
    )

    args = parser.parse_args()
    port = args.port
    motors = {
        # "shoulder_pan": [1, "sts3215"],
        # "shoulder_lift": [2, "sts3215"],
        # "elbow_flex": [3, "sts3215"],
        # "wrist_flex": [4, "sts3215"],
        # "wrist_roll": [5, "sts3215"],
        # "gripper": [6, "sts3215"],
        "left_wheel": (7, "sts3215"),
        "back_wheel": (8, "sts3215"),
        "right_wheel": (9, "sts3215"),
    }

    # teleop_keys: dict[str, str] = field(
    #     default_factory=lambda: {
    #         # Movement
    #         "forward": "w",
    #         "backward": "s",
    #         "left": "q",
    #         "right": "e",
    #         "rotate_left": "a",
    #         "rotate_right": "d",
    #         # Speed control
    #         "speed_up": "r",
    #         "speed_down": "f",
    #         # quit teleop
    #         "quit": "x",
    #     }
    # )

    teleop_keys = {
    "forward": "w", 
    "backward": "s",
    "rotate_left": "a", 
    "rotate_right": "d",
    "left": "q",
    "right": "e",
}

    pressed_keys = {k: False for k in teleop_keys}
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    config = FeetechMotorsBusConfig(
        port=port,
        motors=motors,
    )

    motors_bus = FeetechMotorsBus(config)

    try:
        motors_bus.connect()
        print(f"Connected on port {motors_bus.port}")
    except OSError as e:
        print(f"Error occurred when connecting to the motor bus: {e}")
        #return


    # Initialize motors in velocity mode.
    motors_bus.write("Lock", 0)
    motors_bus.write("Mode", [1, 1, 1], motors)
    motors_bus.write("Lock", 1)
    print("Motors set to velocity mode.")

    # Set the velocity to 0.
    # motors_bus.write("Goal_Speed", [0, 0, 0], motors)   
    # print("Motors set to 0 speed.")
    # time.sleep(1)
    # Set the velocity to 1000.
    # motors_bus.write("Goal_Speed", [1000, 1000, 1000], motors)
    # print("Motors set to 1000 speed.")
    # time.sleep(1)

    try:
        while True:
            # 3.1) 设定线速度和角速度大小
            lin_speed = 0.2   # m/s，你可根据 speed_index 调整
            ang_speed = 30.0  # deg/s
            
            # 3.2) 根据按键拼身体速度
            y_cmd = lin_speed if pressed_keys["forward"] else -lin_speed if pressed_keys["backward"] else 0.0
            x_cmd = 0.0  # 如果你不做侧向平移就保持 0
            theta_cmd = ang_speed if pressed_keys["rotate_left"] else -ang_speed if pressed_keys["rotate_right"] else 0.0

            print(f"Body commands: x={x_cmd}, y={y_cmd}, theta={theta_cmd}")

            # 3.3) 转成各轮 raw 命令
            wheel_cmds: dict = body_to_wheel_raw(None, x_cmd, y_cmd, theta_cmd)
            print(f"Wheel commands: {wheel_cmds}")

            # 3.4) 按固定的轮序发送（跟你构造 motors 字典顺序一致）
            wheel_names = ["left_wheel", "back_wheel", "right_wheel"]
            raw_values = [wheel_cmds[n] for n in wheel_names]
            motors_bus.write("Goal_Speed", raw_values, wheel_names)
            print(f"Sent raw speeds: {raw_values}")

            # 3.5) 小憩一下，频率 ~20Hz
            time.sleep(0.05)

    except KeyboardInterrupt:
        # Ctrl-C 退出
        listener.stop()
        print("Teleop stopped.")
