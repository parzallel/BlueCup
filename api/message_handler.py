import datetime
import time
from .button_filter import ButtonFilter
from .mavlink import mavlink, client
from . import command_handler, connection
from robot_core import robot
from .controller import Controller

import threading

K_MODE_MANUAL = 0b0000000000000010


async def command_recv_handler(msg: mavlink.MAVLink_command_long_message | mavlink.MAVLink_command_int_message):
    if msg.command in command_handler.handlers:
        await command_handler.handlers[msg.command](msg)


async def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


button_filter = ButtonFilter(
    delay=0.5,
    excluded_buttons={}
)
# lock = threading.Lock()
# latest_data = None
#
#
# def mpu_data():
#     try:
#         line = ser.readline().decode('utf-8').strip().split(",")
#
#         mpu = {"roll": line[0], "pitch": line[1], "yaw": line[2]}
#         print(mpu)
#     except Exception as e:
#         print(f"ERROR :  receiving data from MPU as {e}")
#
# def serial_cycle():
#     while True:
#         with lock:
#             if latest_data is not None:
#
#                 ser.write((latest_data + "\n").encode())
#                 print(latest_data)
#                 mpu_data()
#
#         time.sleep(0.11)

# writer_thread = threading.Thread(target=serial_cycle, daemon=True)
# writer_thread.start()
connection.start_serial_thread()


async def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    button_code = msg.buttons

    if not button_filter.allow(button_code):
        return  # Skip due to delay filter
    # TODO : if x or z or y or ... were not 0 the controller should take over or else the sensors
    command = Controller(msg)
    latest_data = command.in_action()
    with connection.lock:
        connection.latest_data = latest_data


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
