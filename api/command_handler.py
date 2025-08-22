from .mavlink import mavlink, client
from robot_core import robot

from typing import Dict, Callable


async def set_camera_zoom(msg: mavlink.MAVLink_command_long_message):
    pass


# msg.param1:Zoom type	CAMERA_ZOOM_TYPE
# ---------------------------------------------------------
# 0	ZOOM_TYPE_STEP	Zoom one step increment (-1 for wide, 1 for tele)
# 1	ZOOM_TYPE_CONTINUOUS	Continuous normalized zoom in/out rate until stopped. Range -1..1, negative: wide, positive: narrow/tele, 0 to stop zooming. Other values should be clipped to the range.
# 2	ZOOM_TYPE_RANGE	Zoom value as proportion of full camera range (a percentage value between 0.0 and 100.0)
# 3	ZOOM_TYPE_FOCAL_LENGTH	Zoom value/variable focal length in millimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)
# 4	ZOOM_TYPE_HORIZONTAL_FOV	Zoom value as horizontal field of view in degrees.
# ---------------------------------------------------------
# msg.param2:Zoom value. The range of valid values depend on the zoom type.
# msg.param3:Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.	min: 0 max: 255 inc: 1
# msg.param4:None


async def set_camera_focus(msg: mavlink.MAVLink_command_long_message):
    pass


# msg.param1:Focus type	SET_FOCUS_TYPE
# --------------------------------------------------
# 0	FOCUS_TYPE_STEP	Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
# 1	FOCUS_TYPE_CONTINUOUS	Continuous normalized focus in/out rate until stopped. Range -1..1, negative: in, positive: out towards infinity, 0 to stop focusing. Other values should be clipped to the range.
# 2	FOCUS_TYPE_RANGE	Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
# 3	FOCUS_TYPE_METERS	Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera).
# 4	FOCUS_TYPE_AUTO	Focus automatically.
# 5	FOCUS_TYPE_AUTO_SINGLE	Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S.
# 6	FOCUS_TYPE_AUTO_CONTINUOUS	Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C.
# --------------------------------------------------
# msg.param2:Focus value
# msg.param3:Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.	min: 0 max: 255 inc: 1
# msg.param4:None


async def component_arm_disarm(msg: mavlink.MAVLink_command_long_message):
    if msg.param1:
        robot.arm()
    else:
        robot.disarm()
    await client.mav.command_ack_send(
        msg.command, mavlink.MAV_RESULT_ACCEPTED)


# msg.param1:Arm (BOOL_FALSE: disarm). Values not equal to 0 or 1 are invalid.	BOOL
# msg.param2:arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)	min: 0 max: 21196 inc: 21196
# msg.param3:None
# msg.param4:None


async def nav_takeoff(msg: mavlink.MAVLink_command_long_message):
    pass

# msg.param1:(Pitch)	Minimum pitch (if airspeed sensor present), desired pitch without sensor	deg
# msg.param2:None
# msg.param3:None
# msg.param4: (Yaw)	Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).	deg
# msg.param5:Latitude
# msg.param6:Longitude
# msg.param7:Altitude


async def do_set_home(msg: mavlink.MAVLink_command_long_message):
    pass


# msg.param1:Use current location (BOOL_FALSE: use specified location). Values not equal to 0 or 1 are invalid.	BOOL
# msg.param2:Roll angle (of surface). Range: -180..180 degrees. NAN or 0 means value not set. 0.01 indicates zero roll.	min: -180 max: 180	deg
# msg.param3:Pitch angle (of surface). Range: -90..90 degrees. NAN or 0 means value not set. 0.01 means zero pitch.	min: -90 max: 90	deg
# msg.param4:Yaw angle. NaN to use default heading. Range: -180..180 degrees.	min: -180 max: 180	deg
# msg.param5:Latitude
# msg.param6:Longitude
# msg.param7:Altitude


async def do_reposition(msg: mavlink.MAVLink_command_int_message):
    pass


# msg.param1:Ground speed, less than 0 (-1) for default	min: -1	m/s
# msg.param2:Bitmask of option flags.	MAV_DO_REPOSITION_FLAGS
# --------------------------------------------------
# 1	MAV_DO_REPOSITION_FLAGS_CHANGE_MODE	The aircraft should immediately transition into guided. This should not be set for follow me applications
# 2	MAV_DO_REPOSITION_FLAGS_RELATIVE_YAW	Yaw relative to the vehicle current heading (if not set, relative to North).
# ---------------------------------------------------
# msg.param3: (Radius)	Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored.		m
# msg.param4: (Yaw)	Yaw heading (heading reference defined in Bitmask field). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)		rad
# msg.param5:Latitude
# msg.param6:Longitude
# msg.param7:Altitude

async def request_message(msg: mavlink.MAVLink_command_long_message):
    pass

# msg.param1:	The MAVLink message ID of the requested message.	min: 0 max: 16777215 inc: 1
# msg.param2: 	Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).
# msg.param3:The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
# msg.param4:The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
# msg.param5:The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
# msg.param6:The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
# msg.param7:Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requester, 2: broadcast.	min: 0 max: 2 inc: 1


async def get_home_position(msg: mavlink.MAVLink_command_int_message):
    await client.mav.command_ack_send(
        msg.command, mavlink.MAV_RESULT_ACCEPTED)
    # This command is used to request the home position from the autopilot.


async def mission_start(msg: mavlink.MAVLink_command_long_message):
    pass
# msg.param1: (First Item)	first_item: the first mission item to run	min: 0 inc: 1
# msg.param2:last_item: the last mission item to run (after this item is run, the mission ends)	min: 0 inc: 1


async def nav_waypoint(msg: mavlink.MAVLink_command_int_message):
    pass
# msg.param1:(Hold)	Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)	min: 0	s
# msg.param2:(Accept Radius)	Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)	min: 0	m
# msg.param3:(Pass Radius)	0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.		m
# msg.param4:(Yaw)	Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).		deg
# msg.param5:Latitude
# msg.param6:Longitude
# msg.param7:Altitude


handlers: Dict[int, Callable] = {
    mavlink.MAV_CMD_COMPONENT_ARM_DISARM: component_arm_disarm,
    mavlink.MAV_CMD_DO_REPOSITION: do_reposition,
    mavlink.MAV_CMD_DO_SET_HOME: do_set_home,
    mavlink.MAV_CMD_MISSION_START: mission_start,
    mavlink.MAV_CMD_NAV_TAKEOFF: nav_takeoff,
    mavlink.MAV_CMD_NAV_WAYPOINT: nav_waypoint,
    mavlink.MAV_CMD_REQUEST_MESSAGE: request_message,
    mavlink.MAV_CMD_SET_CAMERA_FOCUS: set_camera_focus,
    mavlink.MAV_CMD_SET_CAMERA_ZOOM: set_camera_zoom,
}
