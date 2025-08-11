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

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

DEFAULT_PORT = "/dev/ttyACM0"
HALF_TURN_DEGREE = 180


GENERAL_ACTIONS = {'sleep', 'print'}

def parse_actions(file_path):
    actions = []
    with open(file_path, 'r') as f:
        for line in f:
            
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [part.strip() for part in line.split(',')]
            actions.append(parts)
    return actions

def evaluate_expression(expression, variables):
    """
    Simple expression evaluation, supporting {variables} and addition and subtraction operations.
    For example: "{Present_Position} + 500"
    """
    for var, value in variables.items():
        expression = expression.replace(f'{{{var}}}', str(value))
    try:
        return eval(expression)
    except Exception as e:
        print(f"Error evaluating expression '{expression}': {e}")
        return None
    

def move_motors_by_script(script_path, port):

    # Get the directory where the current script file is located and construct an absolute path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, script_path)
    print(f"script_path:{script_path}")
    print(f"script_dir:{script_dir}")
    print(f"file_path:{file_path}")

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
        return

    variables = {}

    # Read and parse the action file
    actions = parse_actions(file_path)

    print(f"actions: {actions}")  

    try:
        for action in actions:
            if not action:
                continue 

            first_field = action[0]

            if first_field in GENERAL_ACTIONS:
                action_type = first_field

                if action_type == 'sleep':
                    if len(action) < 2:
                        print(f"Invalid sleep action: {action}")
                        continue
                    try:
                        duration = float(action[1])
                        print(f"Sleeping for {duration} seconds")
                        time.sleep(duration)
                    except ValueError:
                        print(f"Invalid duration for sleep action: {action[1]}")
                        continue

                elif action_type == 'print':
                    if len(action) < 2:
                        print(f"Invalid print action: {action}")
                        continue
                    message = action[1].format(**variables)
                    print(message)

                else:
                    print(f"Unknown general action type '{action_type}'. Skipping.")
                    continue

            else:
                
                motor_name = action[0]
                action_type = action[1]

                if motor_name not in motors:
                    print(f"Motor '{motor_name}' not found. Skipping action.")
                    continue

                print(f"motor_name:{motor_name},action_type:{action_type}")

                motor_id, motor_model = motors[motor_name]
                try:
                    if action_type == 'read':
                        if len(action) < 3:
                            print(f"Invalid read action: {action}")
                            continue
                        register = action[2]
                        value = motors_bus.read_with_motor_ids(
                            motors_bus.motor_models, motor_id, register
                        )
                        print(f"{motor_name} (ID: {motor_id}) - {register}: {value}")
                        
                        variables[register] = value

                    elif action_type == 'write':
                        if len(action) < 4:
                            print(f"Invalid write action: {action}")
                            continue
                        register = action[2]
                        value_expr = action[3]
                        value = evaluate_expression(value_expr, variables)
                        if value is not None:
                            motors_bus.write_with_motor_ids(
                                motors_bus.motor_models, motor_id, register, value
                            )
                            print(f"Set {motor_name} (ID: {motor_id}) - {register} to {value}")

                    else:
                        print(f"Unknown action type '{action_type}' for motor '{motor_name}'. Skipping.")

                except Exception as motor_e:
                    print(f"Error performing '{action_type}' on motor '{motor_name}' (ID: {motor_id}): {motor_e}")
                    continue  

    except Exception as e:
        print(f"Error occurred during executing actions: {e}")

    finally:
        motors_bus.disconnect()
        print(f"Disconnected from port {port}")

    print(f"Completed executing actions from {script_path}")


def move_motors_by_code(port):
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
        return
    

    try:
        
        for motor_name, (motor_id, motor_model) in motors.items():
            try:
                
                if motor_name=='gripper':
                    ID = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "ID")
                    position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")

                    print(f"------- {motor_name} (ID: {motor_id}) ------")
                    print(f"position:{position}")

                    motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Goal_Position", position + 500)
                    time.sleep(1)
                    position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")
                    print(f'+500  posiston:{position}')
                    time.sleep(1)

                    motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Goal_Position", position - 1000)
                    time.sleep(1)
                    position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")
                    print(f'-1000  posiston:{position}')
                    time.sleep(1)

                    motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Goal_Position", position + 500)
                    time.sleep(1)
                    position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")
                    print(f'+500  posiston:{position}')
                    time.sleep(1)


                    print(f"start_reset")
                    motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Goal_Position", 2048)
                    time.sleep(2)
                    position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")
                    print(f"reset_position:{position}")


                    print("\n")
            except Exception as motor_e:
                print(f"Error reading state for motor '{motor_name}' (ID: {motor_id}): {motor_e}")
                continue  

    except Exception as e:
        print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()


    print(f"Reading motor ID from port {port}")

def move_motor_to_position(port,motor_id,position):
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
        return
    

    position = int(position)
    try:
        ID = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "ID")
        pre_position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")
        motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Lock", 0)
        motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Mode", 0)

        
        motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Goal_Position", position)
        
        
        
        print(f"ID{ID}:Present_Position:{pre_position}")
        print(f"ID{ID}:Move to Goal_Position:{position}")
        print("\n")

    except Exception as e:
        print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()






def reset_motors_to_midpoint(port):
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
        return
    
    motor_states = {}

    try:
        
        for motor_name, (motor_id, motor_model) in motors.items():
            try:

                state = {
                    "ID": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "ID"),
                    "Present_Position": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position"),
                    "Maximum_Acceleration": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Maximum_Acceleration"),
                    "Offset": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Offset"),

                }


                motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Torque_Enable", 128)

                time.sleep(0.1)
                motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Lock", 0)
                motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Maximum_Acceleration", 254)
                motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Acceleration", 254)

                motor_states[motor_name] = state
                print(f"------- {motor_name} (ID: {motor_id}) to midpoint complete!------")


            except Exception as motor_e:
                print(f"Error reading state for motor '{motor_name}' (ID: {motor_id}): {motor_e}")
                continue  

    # when done, consider disconnecting
    except Exception as e:
        print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()


    print(f"Reading motor ID from port {port}")



def reset_motors_torque(port):
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
        return
    
    motor_states = {}

    try:
        
        for motor_name, (motor_id, motor_model) in motors.items():
            try:
                # Read the motor status
                state = {
                    "ID": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "ID"),

                }

                motors_bus.write_with_motor_ids(motors_bus.motor_models, motor_id, "Torque_Enable", 0)

                motor_states[motor_name] = state

                print(f"------- {motor_name} (ID: {motor_id}) reset torque complete!------")


            except Exception as motor_e:
                print(f"Error reading state for motor '{motor_name}' (ID: {motor_id}): {motor_e}")
                continue  # Ignore the disconnected motor and continue to process the next one.


    # when done, consider disconnecting
    except Exception as e:
        print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()



def get_motors_states(port,simple):

    # motors_bus = FeetechMotorsBus(
    #     port=port,
    #     motors=motors,
    # )
    
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
        return
    
    motor_ids = {}
    motor_ids = motors_bus.find_motor_indices(list(range(1, 50)))

    motor_states = {}
    
    try:
        while True:
            for motor_id in motor_ids:
                try:
                    state = {
                        "ID": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "ID"),
                        #"Baud_Rate": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Baud_Rate"),
                        "Position": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position"),
                        "Load": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Load"),
                        "Acceleration": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Maximum_Acceleration"),
                        "Offset": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Offset"),
                        "Voltage": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Voltage"),
                        "Current(mA)": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Current")* 6.5 ,
                        "Temperature": motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Temperature"),
                    }
                    position = state["Position"]
                    angle = position / (4096 // 2) * HALF_TURN_DEGREE
                    state["Angle"] = round(angle, 1)
                    state["Port"] = port

                    #motor_states[motor_id] = state

                    if simple == 'true':
                        print(f"motor_id:{motor_id}")
                    
                    else:
                        for key, value in state.items():
                            print(f"{key}: {value}", end="  ")
                        print("\n")
                except Exception as motor_e:
                    #print(f"Error reading state for motor '{motor_name}' (ID: {motor_id}): {motor_e}")
                    continue 

            time.sleep(2)
            

    # when done, consider disconnecting
    #except Exception as e:
        #print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()

# position_to_angle_with_offset


def position_to_angle_with_offset(port, calibration_file):
    if not os.path.exists(calibration_file):
        raise FileNotFoundError(f"Calibration file not found: {calibration_file}")
    
    with open(calibration_file, "r") as f:
        calib_data = json.load(f)

    offsets = calib_data["homing_offset"]
    motor_names = calib_data["motor_names"]
    drive_modes = calib_data["drive_mode"]

    if not (len(offsets) == len(motor_names) == len(drive_modes)):
        raise ValueError("homing_offset, motor_names, and drive_mode must all be the same length.")

    num_motors = len(offsets)

    motors = {
        name: [i + 1, "sts3215"] for i, name in enumerate(motor_names)
    }

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
        return

    try:
        while True:
            for i, name in enumerate(motor_names):
                motor_id = i + 1
                try:
                    position = motors_bus.read_with_motor_ids(motors_bus.motor_models, motor_id, "Present_Position")
                except Exception as e:
                    print(f"[{name}] Read failed: {e}")
                    continue

                if drive_modes[i] == 1:
                    corrected_angle = (-position + offsets[i]) / (4096 // 2) * HALF_TURN_DEGREE
                else:
                    corrected_angle = (position + offsets[i]) / (4096 // 2) * HALF_TURN_DEGREE

                print(f"{name}: position={position}, angle={round(corrected_angle, 1)}")

            print(f"port: {port}\n")
            time.sleep(2)

    except Exception as e:
        print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()



def configure_motor_id(port,motor_index,motor_idx_des):
    motor_name = "motor"
    motor_idx_des = motor_idx_des  # Use the motor ID passed via argument
    motor_model = 'sts3215'  # Use the motor model passed via argument
    #motors={motor_name: (motor_idx_des, motor_model)}

    #motor_bus = FeetechMotorsBus(port=port, motors={motor_name: (motor_idx_des, motor_model)})
    config = FeetechMotorsBusConfig(
        port=port,
        motors=motors,
    )

    motor_bus = FeetechMotorsBus(config)
    try:
        motor_bus.connect()
        print(f"Connected on port {motor_bus.port}")
    except OSError as e:
        print(f"Error occurred when connecting to the motor bus: {e}")
        return
    

    try:

        present_ids = motor_bus.find_motor_indices(list(range(1, 50)))


        print(f"Motor index found : {present_ids}")

        # Allows ID to be written in memory
        motor_bus.write_with_motor_ids(motor_bus.motor_models, motor_index, "Lock", 0)
        motor_bus.write_with_motor_ids(motor_bus.motor_models, motor_index, "ID", motor_idx_des)
        print(f"Setting its {motor_index} to desired index {motor_idx_des}")

        time.sleep(0.5)
        present_idx = motor_bus.read_with_motor_ids(motor_bus.motor_models, motor_idx_des, "ID", num_retry=2)
        if present_idx != motor_idx_des:
            raise OSError("Failed to write index.")
        
        # Set Maximum_Acceleration
        # Note: this configuration is not in the official STS3215 Memory Table
        print(f"present_idx: {present_idx}")

        motor_bus.write_with_motor_ids(motor_bus.motor_models, present_idx, "Lock", 0)
        motor_bus.write_with_motor_ids(motor_bus.motor_models, present_idx, "Maximum_Acceleration", 306)
        #motor_bus.write_with_motor_ids(motor_bus.motor_models, present_idx, "Goal_Position", 2048)
        #motor_bus.write_with_motor_ids(motor_bus.motor_models, present_idx, "Offset", 0)

        time.sleep(0.5)
        present_ids = motor_bus.find_motor_indices(list(range(1, 12)))
        print(f"Retrying motor index search : {present_ids}")

        print("Complete!")


    except Exception as e:
        print(f"Error occurred during motor configuration: {e}")

    finally:
        motor_bus.disconnect()
        print("Disconnected from motor bus.")
    #motor_bus.write_with_motor_ids(motor_bus.motor_models, motor_index_arbitrary, "Lock", 0)


def get_motors_id(port):
    motor_name = "motor"
    motor_index= -1  
    motor_model = 'sts3215'  

    #motors={motor_name: (motor_idx_des, motor_model)}
    #motor_bus = FeetechMotorsBus(port=port, motors={motor_name: (motor_index, motor_model)})
    config = FeetechMotorsBusConfig(
        port=port,
        motors=motors,
    )

    motor_bus = FeetechMotorsBus(config)

    try:
        motor_bus.connect()
        print(f"Connected on port {motor_bus.port}")
    except OSError as e:
        print(f"Error occurred when connecting to the motor bus: {e}")
        return
    
    
    present_ids = motor_bus.find_motor_indices(list(range(1, 12)))
    print(present_ids)
    print('get_all_motors_id_complete')
    motor_bus.disconnect()


def debug_function(port):
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
        return
    
    motor_states = {}

    try:
        motors_bus.write_with_motor_ids(motors_bus.motor_models, 7, "Lock", 0)
        motors_bus.write_with_motor_ids(motors_bus.motor_models, 7, "Torque_Enable", 128)
        print(f"Complete!")


    # when done, consider disconnecting
    except Exception as e:
        print(f"Error occurred during motor get state: {e}")
    
    finally:
        motors_bus.disconnect()



if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Motor configuration")

    parser.add_argument('command', choices=['reset_motors_to_midpoint', 'reset_motors_torque','get_motors_states', 'position_to_angle_with_offset',
                                            'configure_motor_id','get_motors_id','move_motors_by_code','move_motors_by_script','move_motor_to_position','debug_function'],
                    help="Command to execute")
    
    parser.add_argument('--port', type=str, default=DEFAULT_PORT, 
                        help=f"Set the port (default: {DEFAULT_PORT})")
    parser.add_argument("--model", type=str,  default='sts3215', help="Motor model (e.g. xl330-m077,sts3215)")
    parser.add_argument("--id", type=int, help="Desired ID of the current motor (e.g. 1,2,3)")
    parser.add_argument("--set_id", type=int, help="Targeted ID of the current motor (e.g. 1,2,3)")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Desired baudrate for the motor (default: 1000000)")
    parser.add_argument("--simple", type=str, default= 'flase', help="")
    parser.add_argument("--offset_str", type=str, help="")
    parser.add_argument("--position", type=str, help="")
    parser.add_argument("--script_path", type=str, help="")
    parser.add_argument("--file", type=str, help="Path to calibration JSON file")
    
    args = parser.parse_args()

    motors = {
        "gripper": (7, "sts3215"),
        "wrist_roll": (6, "sts3215"),
        "wrist_yaw": (5, "sts3215"),
        "wrist_flex": (4, "sts3215"),
        "elbow_flex": (3, "sts3215"),
        "shoulder_lift": (2, "sts3215"),
        "shoulder_pan": (1, "sts3215"),
    }


    # set parameter to funciton
    if args.command == 'reset_motors_to_midpoint':
        reset_motors_to_midpoint(args.port)
    elif args.command == 'reset_motors_torque':
        reset_motors_torque(args.port)
    elif args.command == 'get_motors_states':
        get_motors_states(args.port,args.simple)
    elif args.command == 'position_to_angle_with_offset':
        position_to_angle_with_offset(args.port,args.file)        
    elif args.command == 'configure_motor_id':
        configure_motor_id(args.port,args.id,args.set_id)
    elif args.command == 'get_motors_id':
        get_motors_id(args.port)
    elif args.command == 'move_motors_by_code':
        move_motors_by_code(args.port)
    elif args.command == 'move_motors_by_script':
        move_motors_by_script(args.script_path,args.port)
    elif args.command == 'move_motor_to_position':
        move_motor_to_position(args.port,args.id,args.position)
    elif args.command == 'debug_function':
        debug_function(args.port)