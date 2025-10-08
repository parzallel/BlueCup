import logging
from . import message_builder
from .button_filter import ButtonFilter
from .mavlink import mavlink, client
from . import command_handler, connection
from .controller import Controller
from robot_core import robot
K_MODE_MANUAL = 0b0000000000000010


async def command_recv_handler(msg: mavlink.MAVLink_command_long_message | mavlink.MAVLink_command_int_message):
    if msg.command in command_handler.handlers:
        await command_handler.handlers[msg.command](msg)


async def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


button_filter = ButtonFilter(delay=0.3)
connection.start_serial_thread()
saved_yaw_int = None
command = None
mode = "stabilize"  # default mode
is_input_hold = False
looped_command = None
speed = 0


def toggle_mode(button_code: int):
    """Toggle between stabilize and manual mode on triangle button."""
    global mode
    if button_code == 32768:
        mode = "manual"
    elif button_code == 10:
        mode = "stabilize"
    message_builder.change_mode(mode)

def input_hold(button_code: int, data):
    """Toggle between stabilize and manual mode on triangle button."""
    global is_input_hold
    global looped_command
    looped_command = data
    if button_code == 256:
        is_input_hold = False if is_input_hold  else not is_input_hold

        # Choose color: green for stabilize, cyan for manual
        color = "\033[92m" if mode == "stabilize" else "\033[96m"
        reset = "\033[0m"

        print(f"{color}>>> Input hold : {is_input_hold }{reset}")
        return True  # mode toggled
    return False

def process_stabilize(msg, command):
    """Process control in stabilize mode."""
    global saved_yaw_int , speed
    speed = command.acc()
    has_input = any([msg.x, msg.y, msg.z, msg.r, msg.buttons])
    if has_input:
        thruster_command = command.in_action()
        saved_yaw_int = connection.save_yaw()
    else:
        thruster_command = connection.sensor_handler(saved_yaw_int)
    return thruster_command

def process_manual(msg, command):
    """Process control in manual mode."""
    global saved_yaw_int , speed
    saved_yaw_int = connection.save_yaw()
    speed = command.acc()
    return command.in_action()  # placeholder




async def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    """Handle manual control messages from MAVLink."""
    global command
    # Create command object
    if robot.is_armed :
        command = Controller(msg)
        # Apply button delay filter
        if not button_filter.allow(msg.buttons) and msg.buttons != 0:
            return

        # Handle mode toggle
        if msg.buttons == 32768 or msg.buttons == 10:
            toggle_mode(msg.buttons)
        # Handle input hold mode
        if input_hold(msg.buttons , command):
            return

        if is_input_hold :
            thruster_command = looped_command
            return
        # Process based on current mode
        if mode == "stabilize":
            thruster_command = process_stabilize(msg, command)
        else:  # manual

            thruster_command = process_manual(msg, command)
        # Update shared data safely
        with connection.lock:
            connection.latest_data = thruster_command
    else :
        logging.warning("ROV is not armed")





async def heartbeat_handler(msg: mavlink.MAVLink_heartbeat_message):
    pass


# THis is just for example you should implement your own!

async def param_request_list_handler(msg: mavlink.MAVLink_param_request_list_message):
    param_count = len(parameters)
    for index, param in enumerate(parameters):
        # print(param)
        await client.mav.param_value_send(**param, param_count=param_count, param_index=index)


async def param_request_read_handler(msg: mavlink.MAVLink_param_request_read_message):
    print(f"{msg.target_system} {msg.target_component} requested parameter {msg}")
    param_id = msg.param_id.encode()
    if msg.target_system != client.source_system or (
            msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    for index, param in enumerate(parameters):
        if param['param_id'] == param_id:
            await client.mav.param_value_send(**param, param_count=len(parameters), param_index=index)


async def param_set_handler(msg: mavlink.MAVLink_param_set_message):
    # print(msg)
    param_id = msg.param_id.encode()
    for index, param in enumerate(parameters):
        if param['param_id'] == param_id:
            param['param_value'] = msg.param_value
            param['param_type'] = msg.param_type
            await client.mav.param_value_send(**param, param_count=len(parameters), param_index=index)
            break
    else:
        new_object = {'param_id': msg.param_id.encode(), 'param_value': msg.param_value, 'param_type': msg.param_type}
        parameters.append(new_object)
        await client.mav.param_value_send(**new_object, param_count=len(parameters), parama_index=len(parameters) - 1)


async def mission_request_int_handler(msg: mavlink.MAVLink_mission_request_int_message):
    await client.mav.mission_count_send(target_system=255, target_component=240, count=0, mission_type=0)


async def dc(msg: mavlink.MAVLink_message):
    pass


handlers = {
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: command_ack_handler,
    mavlink.MAVLINK_MSG_ID_HEARTBEAT: heartbeat_handler,
    mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL: manual_control_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST: param_request_list_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ: param_request_read_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_SET: param_set_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST: mission_request_int_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_INT: dc,
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT: dc,
}

parameters = [
    {"param_id": b"BTN0_FUNCTION", "param_value": 1, "param_type": 2},
    {"param_id": b"BTN0_SFUNCTION", "param_value": 1, "param_type": 2},
    {"param_id": b"BTN1_FUNCTION", "param_value": 6, "param_type": 2},
    {"param_id": b"BTN1_SFUNCTION", "param_value": 12, "param_type": 2},
    {"param_id": b"BTN2_FUNCTION", "param_value": 7, "param_type": 2},
    {"param_id": b"BTN2_SFUNCTION", "param_value": 63, "param_type": 2},
    {"param_id": b"BTN3_FUNCTION", "param_value": 6, "param_type": 2},
    {"param_id": b"BTN3_SFUNCTION", "param_value": 64, "param_type": 2},
    {"param_id": b"BTN4_FUNCTION", "param_value": 4, "param_type": 2},
    {"param_id": b"BTN4_SFUNCTION", "param_value": 53, "param_type": 2},
    {"param_id": b"BTN5_FUNCTION", "param_value": 2, "param_type": 2},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 8, "param_type": 2},
    {"param_id": b"BTN6_FUNCTION", "param_value": 3, "param_type": 2},
    {"param_id": b"BTN6_SFUNCTION", "param_value": 0, "param_type": 2},
    {"param_id": b"BTN7_FUNCTION", "param_value": 21, "param_type": 2},
    {"param_id": b"BTN7_SFUNCTION", "param_value": 0, "param_type": 2},
    {"param_id": b"BTN8_FUNCTION", "param_value": 48, "param_type": 2},
    {"param_id": b"BTN8_SFUNCTION", "param_value": 0, "param_type": 2},
    {"param_id": b"BTN9_FUNCTION", "param_value": 23, "param_type": 2},
    {"param_id": b"BTN9_SFUNCTION", "param_value": 27, "param_type": 2},
    {"param_id": b"BTN10_FUNCTION", "param_value": 22, "param_type": 2},
    {"param_id": b"BTN10_SFUNCTION", "param_value": 26, "param_type": 2},
    {"param_id": b"BTN11_FUNCTION", "param_value": 42, "param_type": 2},
    {"param_id": b"BTN11_SFUNCTION", "param_value": 47, "param_type": 2},
    {"param_id": b"BTN12_FUNCTION", "param_value": 43, "param_type": 2},
    {"param_id": b"BTN12_SFUNCTION", "param_value": 46, "param_type": 2},
    {"param_id": b"BTN13_FUNCTION", "param_value": 33, "param_type": 2},
    {"param_id": b"BTN13_SFUNCTION", "param_value": 45, "param_type": 2},
    {"param_id": b"BTN14_FUNCTION", "param_value": 32, "param_type": 2},
    {"param_id": b"BTN14_SFUNCTION", "param_value": 44, "param_type": 2},
    {"param_id": b"BTN15_FUNCTION", "param_value": 0, "param_type": 2},
    {"param_id": b"BTN15_SFUNCTION", "param_value": 0, "param_type": 2},
]
