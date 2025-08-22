from .mavlink import mavlink, client, VehicleModes
from typing import Dict, Callable
from robot_core import robot
import random


async def send_heartbeat():
    base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
    custom_mode = VehicleModes.MANUAL.value

    if robot.is_armed:
        base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    await client.mav.heartbeat_send(
        client.source_system,
        client.source_component,
        base_mode,
        custom_mode,
        0, 0
    )


# MPU 6050 , ICM
async def send_raw_imu():
    await client.mav.raw_imu_send(
        time_usec=client.boot_time_usec(),
        xacc=0,
        yacc=0,
        zacc=0,
        xgyro=0,
        ygyro=0,
        zgyro=0,
        xmag=0,
        ymag=0,
        zmag=0,
        id=0,
        temperature=0
    )


# xacc	int16_t		X acceleration (raw)
# yacc	int16_t		Y acceleration (raw)
# zacc	int16_t		Z acceleration (raw)
# xgyro	int16_t		Angular speed around X axis (raw)
# ygyro	int16_t		Angular speed around Y axis (raw)
# zgyro	int16_t		Angular speed around Z axis (raw)
# xmag	int16_t		X Magnetic field (raw)
# ymag	int16_t		Y Magnetic field (raw)
# zmag	int16_t		Z Magnetic field (raw)
# id ++	uint8_t		Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
# Messages with same value are from the same source (instance).
# temperature ++	int16_t	cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).

async def send_gps_raw_int():
    await client.mav.gps_raw_int_send(
        time_usec=client.boot_time_usec(),
        fix_type=0,  # change this based on your gps state
        lat=int(robot.lat*1e7),
        lon=int(robot.lon*1e7),
        alt=int(robot.alt*1e3),
        eph=65535,
        epv=65535,
        vel=65535,
        cog=65535,
        satellites_visible=255,
        alt_ellipsoid=0,
        h_acc=0,
        v_acc=0,
        vel_acc=0,
        hdg_acc=0,
        yaw=65535

    )


# fix_type	uint8_t			GPS_FIX_TYPE	GPS fix type.
# -----------------------------------------------------------------------
# 0	GPS_FIX_TYPE_NO_GPS	No GPS connected
# 1	GPS_FIX_TYPE_NO_FIX	No position information, GPS is connected
# 2	GPS_FIX_TYPE_2D_FIX	2D position
# 3	GPS_FIX_TYPE_3D_FIX	3D position
# 4	GPS_FIX_TYPE_DGPS	DGPS/SBAS aided 3D position
# 5	GPS_FIX_TYPE_RTK_FLOAT	RTK float, 3D position
# 6	GPS_FIX_TYPE_RTK_FIXED	RTK Fixed, 3D position
# 7	GPS_FIX_TYPE_STATIC	Static fixed, typically used for base stations
# 8	GPS_FIX_TYPE_PPP	PPP, 3D position.
# -------------------------------------------------------------------------
# lat	int32_t	degE7			Latitude (WGS84, EGM96 ellipsoid)
# lon	int32_t	degE7			Longitude (WGS84, EGM96 ellipsoid)
# alt	int32_t	mm			Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
# eph	uint16_t		1E-2	invalid:UINT16_MAX	GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
# epv	uint16_t		1E-2	invalid:UINT16_MAX	GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
# vel	uint16_t	cm/s		invalid:UINT16_MAX	GPS ground speed. If unknown, set to: UINT16_MAX
# cog	uint16_t	cdeg		invalid:UINT16_MAX	Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
# satellites_visible	uint8_t			invalid:UINT8_MAX	Number of satellites visible. If unknown, set to UINT8_MAX
# alt_ellipsoid ++	int32_t	mm			Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
# h_acc ++	uint32_t	mm			Position uncertainty.
# v_acc ++	uint32_t	mm			Altitude uncertainty.
# vel_acc ++	uint32_t	mm/s			Speed uncertainty.
# hdg_acc ++	uint32_t	degE5			Heading / track uncertainty
# yaw ++	uint16_t	cdeg		invalid:0	Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.


async def send_scaled_pressure():
    await client.mav.scaled_pressure_send(
        time_boot_ms=client.boot_time_ms(),
        press_abs=0,
        press_diff=0,
        temperature=0,
        temperature_press_diff=0

    )


# press_abs	float	hPa	Absolute pressure
# press_diff	float	hPa	Differential pressure 1
# temperature	int16_t	cdegC	Absolute pressure temperature
# temperature_press_diff ++	int16_t	cdegC	Differential pressure te


async def send_scaled_pressure2():
    await client.mav.scaled_pressure2_send(
        time_boot_ms=client.boot_time_ms(),
        press_abs=0,
        press_diff=0,
        temperature=0,
        temperature_press_diff=0

    )


# press_abs	float	hPa	Absolute pressure
# press_diff	float	hPa	Differential pressure 1
# temperature	int16_t	cdegC	Absolute pressure temperature
# temperature_press_diff ++	int16_t	cdegC	Differential pressure te


async def send_scaled_pressure3():
    await client.mav.scaled_pressure3_send(
        time_boot_ms=client.boot_time_ms(),
        press_abs=0,
        press_diff=0,
        temperature=0,
        temperature_press_diff=0
    )


# press_abs	float	hPa	Absolute pressure
# press_diff	float	hPa	Differential pressure 1
# temperature	int16_t	cdegC	Absolute pressure temperature
# temperature_press_diff ++	int16_t	cdegC	Differential pressure te


async def send_ahrs2():
    await client.mav.ahrs2_send(
        roll=0,
        pitch=0,
        yaw=0,
        altitude=0,
        lat=0,
        lng=0

    )


# roll	float	rad	Roll angle.
# pitch	float	rad	Pitch angle.
# yaw	float	rad	Yaw angle.
# altitude	float	m	Altitude (MSL).
# lat	int32_t	degE7	Latitude.
# lng	int32_t	degE7	Longitude.

# ----------------------------------------------------------

async def send_attitude():
    await client.mav.attitude_send(
        time_boot_ms=client.boot_time_ms(),
        # roll=robot.roll,  # Random roll angle in radians
        # pitch=robot.pitch,  # Random pitch angle in radians
        # yaw=robot.yaw,  # Random yaw angle in radians
        roll=(12/180)*3.1415926,  # Random roll angle in radians
        pitch=(12/180)*3.1415926,  # Random pitch angle in radians
        yaw=(12/180)*3.1415926,
        rollspeed=0,
        pitchspeed=0,
        yawspeed=0

    )


# roll	float	rad	Roll angle (-pi..+pi)
# pitch	float	rad	Pitch angle (-pi..+pi)
# yaw	float	rad	Yaw angle (-pi..+pi)
# rollspeed	float	rad/s	Roll angular speed
# pitchspeed	float	rad/s	Pitch angular speed
# yawspeed	float	rad/s	Yaw angular speed

# ---------------------------------------------------------------

async def send_ekf_status_report():
    await client.mav.ekf_status_report_send(
        flags=1,
        velocity_variance=0,
        pos_vert_variance=0,
        pos_horiz_variance=0,
        compass_variance=0,
        terrain_alt_variance=0,
        airspeed_variance=0

    )


# flags	uint16_t	EKF_STATUS_FLAGS	Flags.
# ----------------------------------------------------------
# 1	EKF_ATTITUDE	Set if EKF's attitude estimate is good.
# 2	EKF_VELOCITY_HORIZ	Set if EKF's horizontal velocity estimate is good.
# 4	EKF_VELOCITY_VERT	Set if EKF's vertical velocity estimate is good.
# 8	EKF_POS_HORIZ_REL	Set if EKF's horizontal position (relative) estimate is good.
# 16	EKF_POS_HORIZ_ABS	Set if EKF's horizontal position (absolute) estimate is good.
# 32	EKF_POS_VERT_ABS	Set if EKF's vertical position (absolute) estimate is good.
# 64	EKF_POS_VERT_AGL	Set if EKF's vertical position (above ground) estimate is good.
# 128	EKF_CONST_POS_MODE	EKF is in constant position mode and does not know it's absolute or relative position.
# 256	EKF_PRED_POS_HORIZ_REL	Set if EKF's predicted horizontal position (relative) estimate is good.
# 512	EKF_PRED_POS_HORIZ_ABS	Set if EKF's predicted horizontal position (absolute) estimate is good.
# 1024	EKF_UNINITIALIZED	Set if EKF has never been healthy.
# 32768	EKF_GPS_GLITCHING	Set if EKF believes the GPS input data is faulty.
# -----------------------------------------------------------------------------------------------
# velocity_variance	float		Velocity variance.
# pos_horiz_variance	float		Horizontal Position variance.
# pos_vert_variance	float		Vertical Position variance.
# compass_variance	float		Compass variance.
# terrain_alt_variance	float		Terrain Altitude variance.

# ----------------------------------------------------------------

async def send_global_position_int():
    await client.mav.global_position_int_send(
        time_boot_ms=client.boot_time_ms(),
        lat=int(robot.lat*1e7),
        lon=int(robot.lon*1e7),
        alt=int(robot.alt*1e3),
        relative_alt=0,
        vx=0,
        vy=0,
        vz=0,
        hdg=65535

    )


# lat	int32_t	degE7	Latitude, expressed
# lon	int32_t	degE7	Longitude, expressed
# alt	int32_t	mm	Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
# relative_alt	int32_t	mm	Altitude above home
# vx	int16_t	cm/s	Ground X Speed (Latitude, positive north)
# vy	int16_t	cm/s	Ground Y Speed (Longitude, positive east)
# vz	int16_t	cm/s	Ground Z Speed (Altitude, positive down)
# hdg	uint16_t	cdeg	Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX


async def send_home_position():
    await client.mav.home_position_send(
        time_usec=client.boot_time_usec(),
        latitude=int(robot.lat*1e7),
        longitude=int(robot.lon*1e7),
        altitude=int(robot.alt*1e3),
        x=0,
        y=0,
        z=0,
        q=[0, 0, 0, 0],
        approach_x=0,
        approach_y=0,
        approach_z=0
    )


# latitude	int32_t	degE7	Latitude (WGS84)
# longitude	int32_t	degE7	Longitude (WGS84)
# altitude	int32_t	mm	Altitude (MSL). Positive for up.
# x	float	m	Local X position of this position in the local coordinate frame (NED)
# y	float	m	Local Y position of this position in the local coordinate frame (NED)
# z	float	m	Local Z position of this position in the local coordinate frame (NED: positive "down")
# q	float[4]		Quaternion indicating world-to-surface-normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground. All fields should be set to NaN if an accurate quaternion for both heading and surface slope cannot be supplied.
# approach_x	float	m	Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
# approach_y	float	m	Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
# approach_z	float	m	Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
# time_usec ++	uint64_t	us	Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.


async def send_local_position_ned():
    await client.mav.local_position_ned_send(
        time_boot_ms=client.boot_time_ms(),
        x=0,
        y=0,
        z=0,
        vx=0,
        vy=0,
        vz=0

    )


# x	float	m	X Position
# y	float	m	Y Position
# z	float	m	Z Position
# vx	float	m/s	X Speed
# vy	float	m/s	Y Speed
# vz	float	m/s	Z Speed


async def send_meminfo():
    await client.mav.meminfo_send(
        brkval=0,
        freemem=0,
        freemem32=0
    )


# brkval	uint16_t		Heap top.
# freemem	uint16_t	bytes	Free memory.
# freemem32 ++	uint32_t	bytes	Free memory (32 bit).


async def send_named_value_float():
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"CamTilt",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"CamPan",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"TetherTrn",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"Lights1",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"Lights2",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"PilotGain",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"InputHold",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"RollPitch",
        value=0
    )
    await client.mav.named_value_float_send(
        time_boot_ms=client.boot_time_ms(),
        name=b"RFTarget",
        value=0
    )


# name	char[10]		Name of the debug variable
# Messages with same value are from the same source (instance).
# value	float		Floating point value


async def send_nav_controller_output():
    await client.mav.nav_controller_output_send(
        nav_roll=0,
        nav_pitch=0,
        nav_bearing=0,
        target_bearing=0,
        wp_dist=0,
        alt_error=0,
        aspd_error=0,
        xtrack_error=0

    )


# nav_roll	float	deg	Current desired roll
# nav_pitch	float	deg	Current desired pitch
# nav_bearing	int16_t	deg	Current desired heading
# target_bearing	int16_t	deg	Bearing to current waypoint/target
# wp_dist	uint16_t	m	Distance to active waypoint
# alt_error	float	m	Current altitude error
# aspd_error	float	m/s	Current airspeed error
# xtrack_error	float	m	Current crosstrack error on x-y plane


async def send_power_status():
    await client.mav.power_status_send(
        Vcc=0,
        Vservo=0,
        flags=0

    )


# Vcc	uint16_t	mV		5V rail voltage.
# Vservo	uint16_t	mV		Servo rail voltage.
# flags	uint16_t		MAV_POWER_STATUS	Bitmap of power supply status flags.
# -------------------------------------------------------------------------------
# 1	MAV_POWER_STATUS_BRICK_VALID	main brick power supply valid
# 2	MAV_POWER_STATUS_SERVO_VALID	main servo power supply valid for FMU
# 4	MAV_POWER_STATUS_USB_CONNECTED	USB power is connected
# 8	MAV_POWER_STATUS_PERIPH_OVERCURRENT	peripheral supply is in over-current state
# 16	MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT	hi-power peripheral supply is in over-current state
# 32	MAV_POWER_STATUS_CHANGED	Power status has changed since boot
# -------------------------------------------------------------------------------


async def send_scaled_imu2():
    await client.mav.scaled_imu2_send(
        time_boot_ms=client.boot_time_ms(),
        xacc=0,
        yacc=0,
        zacc=0,
        xgyro=0,
        ygyro=0,
        zgyro=0,
        xmag=0,
        ymag=0,
        zmag=0,
        temperature=0
    )


# xacc	int16_t	mG	X acceleration
# yacc	int16_t	mG	Y acceleration
# zacc	int16_t	mG	Z acceleration
# xgyro	int16_t	mrad/s	Angular speed around X axis
# ygyro	int16_t	mrad/s	Angular speed around Y axis
# zgyro	int16_t	mrad/s	Angular speed around Z axis
# xmag	int16_t	mgauss	X Magnetic field
# ymag	int16_t	mgauss	Y Magnetic field
# zmag	int16_t	mgauss	Z Magnetic field
# temperature ++	int16_t	cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).

async def send_scaled_imu3():
    await client.mav.scaled_imu3_send(
        time_boot_ms=client.boot_time_ms(),
        xacc=0,
        yacc=0,
        zacc=0,
        xgyro=0,
        ygyro=0,
        zgyro=0,
        xmag=0,
        ymag=0,
        zmag=0,
        temperature=0

    )


# xacc	int16_t	mG	X acceleration
# yacc	int16_t	mG	Y acceleration
# zacc	int16_t	mG	Z acceleration
# xgyro	int16_t	mrad/s	Angular speed around X axis
# ygyro	int16_t	mrad/s	Angular speed around Y axis
# zgyro	int16_t	mrad/s	Angular speed around Z axis
# xmag	int16_t	mgauss	X Magnetic field
# ymag	int16_t	mgauss	Y Magnetic field
# zmag	int16_t	mgauss	Z Magnetic field
# temperature ++	int16_t	cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).


# async async def send_system_time():
#     return {
#         "time_unix_usec":None,
#     }


async def send_sys_status():
    await client.mav.sys_status_send(
        onboard_control_sensors_present=0,
        onboard_control_sensors_enabled=0,
        onboard_control_sensors_health=0,
        load=0,
        voltage_battery=21000,
        current_battery=-1,
        battery_remaining=-1,
        drop_rate_comm=0,
        errors_comm=0,
        errors_count1=0,
        errors_count2=0,
        errors_count3=0,
        errors_count4=0,

    )


# onboard_control_sensors_present	uint32_t		MAV_SYS_STATUS_SENSOR	Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
# onboard_control_sensors_enabled	uint32_t		MAV_SYS_STATUS_SENSOR	Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of 1: enabled.
# onboard_control_sensors_health	uint32_t		MAV_SYS_STATUS_SENSOR	Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
# load	uint16_t	d%		Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
# voltage_battery	uint16_t	mV	invalid:UINT16_MAX	Battery voltage, UINT16_MAX: Voltage not sent by autopilot
# current_battery	int16_t	cA	invalid:-1	Battery current, -1: Current not sent by autopilot
# battery_remaining	int8_t	%	invalid:-1	Battery energy remaining, -1: Battery remaining energy not sent by autopilot
# drop_rate_comm	uint16_t	c%		Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
# errors_comm	uint16_t			Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
# errors_count1	uint16_t			Autopilot-specific errors
# errors_count2	uint16_t			Autopilot-specific errors
# errors_count3	uint16_t			Autopilot-specific errors
# errors_count4	uint16_t			Autopilot-specific errors
# onboard_control_sensors_present_extended ++	uint32_t		MAV_SYS_STATUS_SENSOR_EXTENDED	Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
# onboard_control_sensors_enabled_extended ++	uint32_t		MAV_SYS_STATUS_SENSOR_EXTENDED	Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of 1: enabled.
# onboard_control_sensors_health_extended ++	uint32_t		MAV_SYS_STATUS_SENSOR_EXTENDED	Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.


async def send_terrain_report():
    await client.mav.terrain_report_send(
        lat=0,
        lon=0,
        spacing=0,
        terrain_height=0,
        current_height=0,
        pending=0,
        loaded=0

    )


async def send_vfr_hud():
    await client.mav.vfr_hud_send(
        airspeed=0,
        groundspeed=0,
        heading=0,
        throttle=0,
        alt=0,
        climb=0
    )


# airspeed	float	m/s	Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall speed.
# groundspeed	float	m/s	Current ground speed.
# heading	int16_t	deg	Current heading in compass units (0-360, 0=north).
# throttle	uint16_t	%	Current throttle setting (0 to 100).
# alt	float	m	Current altitude (MSL).
# climb	float	m/s	Current climb rate.

async def send_vibration():
    await client.mav.vibration_send(
        time_usec=client.boot_time_usec(),
        vibration_x=0,
        vibration_y=0,
        vibration_z=0,
        clipping_0=0,
        clipping_1=0,
        clipping_2=0

    )


# vibration_x	float		Vibration levels on X-axis
# vibration_y	float		Vibration levels on Y-axis
# vibration_z	float		Vibration levels on Z-axis
# clipping_0	uint32_t		first accelerometer clipping count
# clipping_1	uint32_t		second accelerometer clipping count
# clipping_2	uint32_t		third accelerometer clipping count


async def send_fence_status():
    await client.mav.fence_status_send(
        breach_status=0,
        breach_count=0,
        breach_type=0,
        breach_time=0,
        breach_mitigation=0

    )


# breach_status	uint8_t			Breach status (0 if currently inside fence, 1 if outside).
# breach_count	uint16_t			Number of fence breaches.
# breach_type	uint8_t		FENCE_BREACH	Last breach type.
# ---------------------------------------------------------------
# 0	FENCE_BREACH_NONE	No last fence breach
# 1	FENCE_BREACH_MINALT	Breached minimum altitude
# 2	FENCE_BREACH_MAXALT	Breached maximum altitude
# 3	FENCE_BREACH_BOUNDARY	Breached fence boundary
# ----------------------------------------------------------------
# breach_time	uint32_t	ms		Time (since boot) of last breach.
# breach_mitigation ++	uint8_t		FENCE_MITIGATE	Active action to prevent fence breach
# -------------------------------------------------------------
# 0	FENCE_MITIGATE_UNKNOWN	Unknown
# 1	FENCE_MITIGATE_NONE	No actions being taken
# 2	FENCE_MITIGATE_VEL_LIMIT	Velocity limiting active to prevent breach
# -------------------------------------------------------------

async def send_servo_output_raw():
    await client.mav.servo_output_raw_send(
        time_usec=client.boot_time_usec(),
        port=0,
        servo1_raw=0,
        servo2_raw=0,
        servo3_raw=0,
        servo4_raw=0,
        servo5_raw=0,
        servo6_raw=0,
        servo7_raw=0,
        servo8_raw=0,
        servo9_raw=0,
        servo10_raw=0,
        servo11_raw=0,
        servo12_raw=0,
        servo13_raw=0,
        servo14_raw=0,
        servo15_raw=0,
        servo16_raw=0

    )


# time_usec	uint32_t	us	Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
# port	uint8_t		Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
# servo1_raw	uint16_t	us	Servo output 1 value
# servo2_raw	uint16_t	us	Servo output 2 value
# servo3_raw	uint16_t	us	Servo output 3 value
# servo4_raw	uint16_t	us	Servo output 4 value
# servo5_raw	uint16_t	us	Servo output 5 value
# servo6_raw	uint16_t	us	Servo output 6 value
# servo7_raw	uint16_t	us	Servo output 7 value
# servo8_raw	uint16_t	us	Servo output 8 value
# servo9_raw ++	uint16_t	us	Servo output 9 value
# servo10_raw ++	uint16_t	us	Servo output 10 value
# servo11_raw ++	uint16_t	us	Servo output 11 value
# servo12_raw ++	uint16_t	us	Servo output 12 value
# servo13_raw ++	uint16_t	us	Servo output 13 value
# servo14_raw ++	uint16_t	us	Servo output 14 value
# servo15_raw ++	uint16_t	us	Servo output 15 value
# servo16_raw ++	uint16_t	us	Servo output 16 value


async def send_rc_channels():
    await client.mav.rc_channels_send(
        time_boot_ms=client.boot_time_ms(),
        chancount=0,
        chan1_raw=0,
        chan2_raw=0,
        chan3_raw=0,
        chan4_raw=0,
        chan5_raw=0,
        chan6_raw=0,
        chan7_raw=0,
        chan8_raw=0,
        chan9_raw=0,
        chan10_raw=0,
        chan11_raw=0,
        chan12_raw=0,
        chan13_raw=0,
        chan14_raw=0,
        chan15_raw=0,
        chan16_raw=0,
        chan17_raw=0,
        chan18_raw=0,
        rssi=0,

    )


# time_boot_ms	uint32_t	ms	Timestamp (time since system boot).
# chancount	uint8_t		Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
# chan1_raw	uint16_t	us	RC channel 1 value.
# chan2_raw	uint16_t	us	RC channel 2 value.
# chan3_raw	uint16_t	us	RC channel 3 value.
# chan4_raw	uint16_t	us	RC channel 4 value.
# chan5_raw	uint16_t	us	RC channel 5 value.
# chan6_raw	uint16_t	us	RC channel 6 value.
# chan7_raw	uint16_t	us	RC channel 7 value.
# chan8_raw	uint16_t	us	RC channel 8 value.
# chan9_raw	uint16_t	us	RC channel 9 value.
# chan10_raw	uint16_t	us	RC channel 10 value.
# chan11_raw	uint16_t	us	RC channel 11 value.
# chan12_raw	uint16_t	us	RC channel 12 value.
# chan13_raw	uint16_t	us	RC channel 13 value.
# chan14_raw	uint16_t	us	RC channel 14 value.
# chan15_raw	uint16_t	us	RC channel 15 value.
# chan16_raw	uint16_t	us	RC channel 16 value.
# chan17_raw	uint16_t	us	RC channel 17 value.
# chan18_raw	uint16_t	us	RC channel 18 value.
# rssi	uint8_t		Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.


async def send_rc_channels_raw():
    await client.mav.rc_channels_raw_send(
        time_boot_ms=client.boot_time_ms(),
        port=0,
        chan1_raw=0,
        chan2_raw=0,
        chan3_raw=0,
        chan4_raw=0,
        chan5_raw=0,
        chan6_raw=0,
        chan7_raw=0,
        chan8_raw=0,
        rssi=0

    )


# time_boot_ms	uint32_t	ms	Timestamp (time since system boot).
# port	uint8_t		Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
# chan1_raw	uint16_t	us	RC channel 1 value.
# chan2_raw	uint16_t	us	RC channel 2 value.
# chan3_raw	uint16_t	us	RC channel 3 value.
# chan4_raw	uint16_t	us	RC channel 4 value.
# chan5_raw	uint16_t	us	RC channel 5 value.
# chan6_raw	uint16_t	us	RC channel 6 value.
# chan7_raw	uint16_t	us	RC channel 7 value.
# chan8_raw	uint16_t	us	RC channel 8 value.
# rssi	uint8_t		Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.


# axis	uint8_t	PID_TUNING_AXIS	Axis.
# Messages with same value are from the same source (instance).
# desired	float		Desired rate.
# achieved	float		Achieved rate.
# FF	float		FF component.
# P	float		P component.
# I	float		I component.
# D	float		D component.
# SRate ++	float		Slew rate.
# PDmod ++	float		P/D oscillation modifier.


async def send_ahrs():
    await client.mav.ahrs_send(
        omegaIx=0,
        omegaIy=0,
        omegaIz=0,
        accel_weight=0,
        renorm_val=0,
        error_rp=0,
        error_yaw=0

    )


# omegaIx	float	rad/s	X gyro drift estimate.
# omegaIy	float	rad/s	Y gyro drift estimate.
# omegaIz	float	rad/s	Z gyro drift estimate.
# accel_weight	float		Average accel_weight.
# renorm_val	float		Average renormalisation value.
# error_rp	float		Average error_roll_pitch value.
# error_yaw	float		Average error_yaw value.

async def send_system_time():
    await client.mav.system_time_send(
        time_boot_ms=client.boot_time_ms(),
        time_unix_usec=client.boot_time_usec()

    )


# time_unix_usec	uint64_t	us	Timestamp (UNIX epoch time).
# time_boot_ms	uint32_t	ms	Timestamp (time since system boot).


async def send_rangefinder():
    await client.mav.rangefinder_send(
        distance=0,
        voltage=0
    )


# distance	float	m	Distance.
# voltage	float	V	Raw voltage if available, zero otherwise.

async def send_distance_sensor():
    await client.mav.distance_sensor_send(
        time_boot_ms=client.boot_time_ms(),
        min_distance=0,
        max_distance=0,
        current_distance=0,
        type=0,
        id=0,
        orientation=0,
        covariance=0,
        horizontal_fov=0,
        vertical_fov=0,
        quaternion=[0] * 4,
        signal_quality=0,

    )


# time_boot_ms	uint32_t	ms		Timestamp (time since system boot).
# min_distance	uint16_t	cm		Minimum distance the sensor can measure
# max_distance	uint16_t	cm		Maximum distance the sensor can measure
# current_distance	uint16_t	cm		Current distance reading
# type	uint8_t		MAV_DISTANCE_SENSOR	Type of distance sensor.
# -----------------------------------------------------------------------------
# 0	MAV_DISTANCE_SENSOR_LASER	Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
# 1	MAV_DISTANCE_SENSOR_ULTRASOUND	Ultrasound rangefinder, e.g. MaxBotix units
# 2	MAV_DISTANCE_SENSOR_INFRARED	Infrared rangefinder, e.g. Sharp units
# 3	MAV_DISTANCE_SENSOR_RADAR	Radar type, e.g. uLanding units
# 4	MAV_DISTANCE_SENSOR_UNKNOWN	Broken or unknown type, e.g. analog units
# -----------------------------------------------------------------------------
# id	uint8_t			Onboard ID of the sensor Messages with same value are from the same source (instance).
# orientation	uint8_t		MAV_SENSOR_ORIENTATION	Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
# -------------------------------------------------------------------------------------------
# 0	MAV_SENSOR_ROTATION_NONE	Roll: 0, Pitch: 0, Yaw: 0
# 1	MAV_SENSOR_ROTATION_YAW_45	Roll: 0, Pitch: 0, Yaw: 45
# 2	MAV_SENSOR_ROTATION_YAW_90	Roll: 0, Pitch: 0, Yaw: 90
# 3	MAV_SENSOR_ROTATION_YAW_135	Roll: 0, Pitch: 0, Yaw: 135
# 4	MAV_SENSOR_ROTATION_YAW_180	Roll: 0, Pitch: 0, Yaw: 180
# 5	MAV_SENSOR_ROTATION_YAW_225	Roll: 0, Pitch: 0, Yaw: 225
# 6	MAV_SENSOR_ROTATION_YAW_270	Roll: 0, Pitch: 0, Yaw: 270
# 7	MAV_SENSOR_ROTATION_YAW_315	Roll: 0, Pitch: 0, Yaw: 315
# 8	MAV_SENSOR_ROTATION_ROLL_180	Roll: 180, Pitch: 0, Yaw: 0
# 9	MAV_SENSOR_ROTATION_ROLL_180_YAW_45	Roll: 180, Pitch: 0, Yaw: 45
# 10	MAV_SENSOR_ROTATION_ROLL_180_YAW_90	Roll: 180, Pitch: 0, Yaw: 90
# 11	MAV_SENSOR_ROTATION_ROLL_180_YAW_135	Roll: 180, Pitch: 0, Yaw: 135
# 12	MAV_SENSOR_ROTATION_PITCH_180	Roll: 0, Pitch: 180, Yaw: 0
# 13	MAV_SENSOR_ROTATION_ROLL_180_YAW_225	Roll: 180, Pitch: 0, Yaw: 225
# 14	MAV_SENSOR_ROTATION_ROLL_180_YAW_270	Roll: 180, Pitch: 0, Yaw: 270
# 15	MAV_SENSOR_ROTATION_ROLL_180_YAW_315	Roll: 180, Pitch: 0, Yaw: 315
# 16	MAV_SENSOR_ROTATION_ROLL_90	Roll: 90, Pitch: 0, Yaw: 0
# 17	MAV_SENSOR_ROTATION_ROLL_90_YAW_45	Roll: 90, Pitch: 0, Yaw: 45
# 18	MAV_SENSOR_ROTATION_ROLL_90_YAW_90	Roll: 90, Pitch: 0, Yaw: 90
# 19	MAV_SENSOR_ROTATION_ROLL_90_YAW_135	Roll: 90, Pitch: 0, Yaw: 135
# 20	MAV_SENSOR_ROTATION_ROLL_270	Roll: 270, Pitch: 0, Yaw: 0
# 21	MAV_SENSOR_ROTATION_ROLL_270_YAW_45	Roll: 270, Pitch: 0, Yaw: 45
# 22	MAV_SENSOR_ROTATION_ROLL_270_YAW_90	Roll: 270, Pitch: 0, Yaw: 90
# 23	MAV_SENSOR_ROTATION_ROLL_270_YAW_135	Roll: 270, Pitch: 0, Yaw: 135
# 24	MAV_SENSOR_ROTATION_PITCH_90	Roll: 0, Pitch: 90, Yaw: 0
# 25	MAV_SENSOR_ROTATION_PITCH_270	Roll: 0, Pitch: 270, Yaw: 0
# 26	MAV_SENSOR_ROTATION_PITCH_180_YAW_90	Roll: 0, Pitch: 180, Yaw: 90
# 27	MAV_SENSOR_ROTATION_PITCH_180_YAW_270	Roll: 0, Pitch: 180, Yaw: 270
# 28	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90	Roll: 90, Pitch: 90, Yaw: 0
# 29	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90	Roll: 180, Pitch: 90, Yaw: 0
# 30	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90	Roll: 270, Pitch: 90, Yaw: 0
# 31	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180	Roll: 90, Pitch: 180, Yaw: 0
# 32	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180	Roll: 270, Pitch: 180, Yaw: 0
# 33	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270	Roll: 90, Pitch: 270, Yaw: 0
# 34	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270	Roll: 180, Pitch: 270, Yaw: 0
# 35	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270	Roll: 270, Pitch: 270, Yaw: 0
# 36	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90	Roll: 90, Pitch: 180, Yaw: 90
# 37	MAV_SENSOR_ROTATION_ROLL_90_YAW_270	Roll: 90, Pitch: 0, Yaw: 270
# 38	MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293	Roll: 90, Pitch: 68, Yaw: 293
# 39	MAV_SENSOR_ROTATION_PITCH_315	Pitch: 315
# 40	MAV_SENSOR_ROTATION_ROLL_90_PITCH_315	Roll: 90, Pitch: 315
# 100	MAV_SENSOR_ROTATION_CUSTOM	Custom orientation
# -------------------------------------------------------------------------------------------
# covariance	uint8_t	cm^2	invalid:UINT8_MAX	Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.
# horizontal_fov ++	float	rad	invalid:0	Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.
# vertical_fov ++	float	rad	invalid:0	Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.
# quaternion ++	float[4]		invalid:[0]	Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."
# signal_quality ++	uint8_t	%	invalid:0	Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.

async def send_battery_status():
    await client.mav.battery_status_send(
        id=0,
        battery_function=0,
        type=0,
        temperature=32767,
        voltages=[0] * 16,
        current_battery=-1,
        current_consumed=-1,
        energy_consumed=-1,
        battery_remaining=-1,
        time_remaining=0,
        charge_state=0,
        voltages_ext=[0] * 4,
        mode=0,
        fault_bitmask=0,
    )

# id	uint8_t			Battery ID Messages with same value are from the same source (instance).
# battery_function	uint8_t		MAV_BATTERY_FUNCTION	Function of the battery
# ------------------------------------------------------------------------
# 0	MAV_BATTERY_FUNCTION_UNKNOWN	Battery function is unknown
# 1	MAV_BATTERY_FUNCTION_ALL	Battery supports all flight systems
# 2	MAV_BATTERY_FUNCTION_PROPULSION	Battery for the propulsion system
# 3	MAV_BATTERY_FUNCTION_AVIONICS	Avionics battery
# 4	MAV_BATTERY_FUNCTION_PAYLOAD	Payload battery
# ------------------------------------------------------------------------
# type	uint8_t		MAV_BATTERY_TYPE	Type (chemistry) of the battery
# ---------------------------------------------------------------------------
# 0	MAV_BATTERY_TYPE_UNKNOWN	Not specified.
# 1	MAV_BATTERY_TYPE_LIPO	Lithium polymer battery
# 2	MAV_BATTERY_TYPE_LIFE	Lithium-iron-phosphate battery
# 3	MAV_BATTERY_TYPE_LION	Lithium-ION battery
# 4	MAV_BATTERY_TYPE_NIMH	Nickel metal hydride battery
# ---------------------------------------------------------------------------
# temperature	int16_t	cdegC	invalid:INT16_MAX	Temperature of the battery. INT16_MAX for unknown temperature.
# voltages	uint16_t[10]	mV	invalid:[UINT16_MAX]	Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
# current_battery	int16_t	cA	invalid:-1	Battery current, -1: autopilot does not measure the current
# current_consumed	int32_t	mAh	invalid:-1	Consumed charge, -1: autopilot does not provide consumption estimate
# energy_consumed	int32_t	hJ	invalid:-1	Consumed energy, -1: autopilot does not provide energy consumption estimate
# battery_remaining	int8_t	%	invalid:-1	Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
# time_remaining ++	int32_t	s	invalid:0	Remaining battery time, 0: autopilot does not provide remaining battery time estimate
# charge_state ++	uint8_t		MAV_BATTERY_CHARGE_STATE	State for extent of discharge, provided by autopilot for warning or external reactions
# -----------------------------------------------------------------------
# 0	MAV_BATTERY_CHARGE_STATE_UNDEFINED	Low battery state is not provided
# 1	MAV_BATTERY_CHARGE_STATE_OK	Battery is not in low state. Normal operation.
# 2	MAV_BATTERY_CHARGE_STATE_LOW	Battery state is low, warn and monitor close.
# 3	MAV_BATTERY_CHARGE_STATE_CRITICAL	Battery state is critical, return or abort immediately.
# 4	MAV_BATTERY_CHARGE_STATE_EMERGENCY	Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage.
# 5	MAV_BATTERY_CHARGE_STATE_FAILED	Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT.
# 6	MAV_BATTERY_CHARGE_STATE_UNHEALTHY	Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT.
# 7	MAV_BATTERY_CHARGE_STATE_CHARGING	Battery is charging.
# -----------------------------------------------------------------------
# voltages_ext ++	uint16_t[4]	mV	invalid:[0]	Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a value of 0, where zero indicates not supported (note, this is different than for the voltages field and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.
# mode ++	uint8_t		MAV_BATTERY_MODE	Battery mode. Default (0) is that battery mode reporting is not supported or battery is in normal-use mode.
# ---------------------------------------------------------------------------
# 0	MAV_BATTERY_MODE_UNKNOWN	Battery mode not supported/unknown battery mode/normal operation.
# 1	MAV_BATTERY_MODE_AUTO_DISCHARGING	Battery is auto discharging (towards storage level).
# 2	MAV_BATTERY_MODE_HOT_SWAP	Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits).
# ---------------------------------------------------------------------------
# fault_bitmask ++	uint32_t		MAV_BATTERY_FAULT	Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).
# ------------------------------------------------------------------------------
# 1	MAV_BATTERY_FAULT_DEEP_DISCHARGE	Battery has deep discharged.
# 2	MAV_BATTERY_FAULT_SPIKES	Voltage spikes.
# 4	MAV_BATTERY_FAULT_CELL_FAIL	One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used).
# 8	MAV_BATTERY_FAULT_OVER_CURRENT	Over-current fault.
# 16	MAV_BATTERY_FAULT_OVER_TEMPERATURE	Over-temperature fault.
# 32	MAV_BATTERY_FAULT_UNDER_TEMPERATURE	Under-temperature fault.
# 64	MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE	Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage).
# 128	MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE	Battery firmware is not compatible with current autopilot firmware.
# 256	BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION	Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).
# ------------------------------------------------------------------------------


async def send_gimbal_device_attitude_status():
    await client.mav.gimbal_device_attitude_status_send(
        target_system=0,
        target_component=0,
        time_boot_ms=client.boot_time_ms(),
        flags=0,
        q=[0, 0, 0, 0],
        angular_velocity_x=0,
        angular_velocity_y=0,
        angular_velocity_z=0,
        failure_flags=0,
        delta_yaw=0,
        delta_yaw_velocity=0,
        gimbal_device_id=0
    )
# target_system	uint8_t			System ID
# target_component	uint8_t			Component ID
# time_boot_ms	uint32_t	ms		Timestamp (time since system boot).
# flags	uint16_t		GIMBAL_DEVICE_FLAGS	Current gimbal flags set.
# -----------------------------------------------------------------------
# GIMBAL_DEVICE_FLAGS
# (Bitmask) Flags for gimbal device (lower level) operation.
# Value	Name	Description
# 1	GIMBAL_DEVICE_FLAGS_RETRACT	Set to retracted safe position (no stabilization), takes precedence over all other flags.
# 2	GIMBAL_DEVICE_FLAGS_NEUTRAL	Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (roll=pitch=yaw=0) but may be any orientation.
# 4	GIMBAL_DEVICE_FLAGS_ROLL_LOCK	Lock roll angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal.
# 8	GIMBAL_DEVICE_FLAGS_PITCH_LOCK	Lock pitch angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal.
# 16	GIMBAL_DEVICE_FLAGS_YAW_LOCK	Lock yaw angle to absolute angle relative to North (not relative to vehicle). If this flag is set, the yaw angle and z component of angular velocity are relative to North (earth frame, x-axis pointing North), else they are relative to the vehicle heading (vehicle frame, earth frame rotated so that the x-axis is pointing forward).
# 32	GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME	Yaw angle and z component of angular velocity are relative to the vehicle heading (vehicle frame, earth frame rotated such that the x-axis is pointing forward).
# 64	GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME	Yaw angle and z component of angular velocity are relative to North (earth frame, x-axis is pointing North).
# 128	GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME	Gimbal device can accept yaw angle inputs relative to North (earth frame). This flag is only for reporting (attempts to set this flag are ignored).
# 256	GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE	The gimbal orientation is set exclusively by the RC signals feed to the gimbal's radio control inputs. MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE) are ignored.
# 512	GIMBAL_DEVICE_FLAGS_RC_MIXED	The gimbal orientation is determined by combining/mixing the RC signals feed to the gimbal's radio control inputs and the MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE). How these two controls are combined or mixed is not defined by the protocol but is up to the implementation.
# -----------------------------------------------------------------------
# q	float[4]			Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description.
# angular_velocity_x	float	rad/s	invalid:NaN	X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN if unknown.
# angular_velocity_y	float	rad/s	invalid:NaN	Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN if unknown.
# angular_velocity_z	float	rad/s	invalid:NaN	Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN if unknown.
# failure_flags	uint32_t		GIMBAL_DEVICE_ERROR_FLAGS	Failure flags (0 for no failure)
# --------------------------------------------------------------------------------------------------
# 1	GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT	Gimbal device is limited by hardware roll limit.
# 2	GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT	Gimbal device is limited by hardware pitch limit.
# 4	GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT	Gimbal device is limited by hardware yaw limit.
# 8	GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR	There is an error with the gimbal encoders.
# 16	GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR	There is an error with the gimbal power source.
# 32	GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR	There is an error with the gimbal motors.
# 64	GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR	There is an error with the gimbal's software.
# 128	GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR	There is an error with the gimbal's communication.
# 256	GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING	Gimbal device is currently calibrating.
# 512	GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER	Gimbal device is not assigned to a gimbal manager.
# ----------------------------------------------------------------------------------------------------
# delta_yaw ++	float	rad	invalid:NAN	Yaw angle relating the quaternions in earth and body frames (see message description). NaN if unknown.
# delta_yaw_velocity ++	float	rad/s	invalid:NAN	Yaw angular velocity relating the angular velocities in earth and body frames (see message description). NaN if unknown.
# gimbal_device_id ++	uint8_t		invalid:0	This field is to be used if the gimbal manager and the gimbal device are the same component and hence have the same component ID. This field is then set a number between 1-6. If the component ID is separate, this field is not required and must be set to 0.


async def send_mag_cal_report():
    await client.mav.mag_cal_report_send(
        compass_id=0,
        cal_mask=0,
        cal_status=0,
        autosaved=0,
        fitness=0,
        ofs_x=0,
        ofs_y=0,
        ofs_z=0,
        diag_x=0,
        diag_y=0,
        diag_z=0,
        offdiag_x=0,
        offdiag_y=0,
        offdiag_z=0,
        orientation_confidence=0,
        old_orientation=0,
        new_orientation=0,
        scale_factor=0,


    )
# compass_id	uint8_t			Compass being calibrated.
# Messages with same value are from the same source (instance).
# cal_mask	uint8_t			Bitmask of compasses being calibrated.
# cal_status	uint8_t		MAG_CAL_STATUS	Calibration Status.
# -----------------------------------------------------------------------------------
# 0	MAG_CAL_NOT_STARTED
# 1	MAG_CAL_WAITING_TO_START
# 2	MAG_CAL_RUNNING_STEP_ONE
# 3	MAG_CAL_RUNNING_STEP_TWO
# 4	MAG_CAL_SUCCESS
# 5	MAG_CAL_FAILED
# 6	MAG_CAL_BAD_ORIENTATION
# 7	MAG_CAL_BAD_RADIUS
# -----------------------------------------------------------------------------------
# autosaved	uint8_t			0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
# fitness	float	mgauss		RMS milligauss residuals.
# ofs_x	float			X offset.
# ofs_y	float			Y offset.
# ofs_z	float			Z offset.
# diag_x	float			X diagonal (matrix 11).
# diag_y	float			Y diagonal (matrix 22).
# diag_z	float			Z diagonal (matrix 33).
# offdiag_x	float			X off-diagonal (matrix 12 and 21).
# offdiag_y	float			Y off-diagonal (matrix 13 and 31).
# offdiag_z	float			Z off-diagonal (matrix 32 and 23).
# orientation_confidence ++	float			Confidence in orientation (higher is better).
# old_orientation ++	uint8_t		MAV_SENSOR_ORIENTATION	orientation before calibration.
# ---------------------------------------------------------------------------
# 0	MAV_SENSOR_ROTATION_NONE	Roll: 0, Pitch: 0, Yaw: 0
# 1	MAV_SENSOR_ROTATION_YAW_45	Roll: 0, Pitch: 0, Yaw: 45
# 2	MAV_SENSOR_ROTATION_YAW_90	Roll: 0, Pitch: 0, Yaw: 90
# 3	MAV_SENSOR_ROTATION_YAW_135	Roll: 0, Pitch: 0, Yaw: 135
# 4	MAV_SENSOR_ROTATION_YAW_180	Roll: 0, Pitch: 0, Yaw: 180
# 5	MAV_SENSOR_ROTATION_YAW_225	Roll: 0, Pitch: 0, Yaw: 225
# 6	MAV_SENSOR_ROTATION_YAW_270	Roll: 0, Pitch: 0, Yaw: 270
# 7	MAV_SENSOR_ROTATION_YAW_315	Roll: 0, Pitch: 0, Yaw: 315
# 8	MAV_SENSOR_ROTATION_ROLL_180	Roll: 180, Pitch: 0, Yaw: 0
# 9	MAV_SENSOR_ROTATION_ROLL_180_YAW_45	Roll: 180, Pitch: 0, Yaw: 45
# 10	MAV_SENSOR_ROTATION_ROLL_180_YAW_90	Roll: 180, Pitch: 0, Yaw: 90
# 11	MAV_SENSOR_ROTATION_ROLL_180_YAW_135	Roll: 180, Pitch: 0, Yaw: 135
# 12	MAV_SENSOR_ROTATION_PITCH_180	Roll: 0, Pitch: 180, Yaw: 0
# 13	MAV_SENSOR_ROTATION_ROLL_180_YAW_225	Roll: 180, Pitch: 0, Yaw: 225
# 14	MAV_SENSOR_ROTATION_ROLL_180_YAW_270	Roll: 180, Pitch: 0, Yaw: 270
# 15	MAV_SENSOR_ROTATION_ROLL_180_YAW_315	Roll: 180, Pitch: 0, Yaw: 315
# 16	MAV_SENSOR_ROTATION_ROLL_90	Roll: 90, Pitch: 0, Yaw: 0
# 17	MAV_SENSOR_ROTATION_ROLL_90_YAW_45	Roll: 90, Pitch: 0, Yaw: 45
# 18	MAV_SENSOR_ROTATION_ROLL_90_YAW_90	Roll: 90, Pitch: 0, Yaw: 90
# 19	MAV_SENSOR_ROTATION_ROLL_90_YAW_135	Roll: 90, Pitch: 0, Yaw: 135
# 20	MAV_SENSOR_ROTATION_ROLL_270	Roll: 270, Pitch: 0, Yaw: 0
# 21	MAV_SENSOR_ROTATION_ROLL_270_YAW_45	Roll: 270, Pitch: 0, Yaw: 45
# 22	MAV_SENSOR_ROTATION_ROLL_270_YAW_90	Roll: 270, Pitch: 0, Yaw: 90
# 23	MAV_SENSOR_ROTATION_ROLL_270_YAW_135	Roll: 270, Pitch: 0, Yaw: 135
# 24	MAV_SENSOR_ROTATION_PITCH_90	Roll: 0, Pitch: 90, Yaw: 0
# 25	MAV_SENSOR_ROTATION_PITCH_270	Roll: 0, Pitch: 270, Yaw: 0
# 26	MAV_SENSOR_ROTATION_PITCH_180_YAW_90	Roll: 0, Pitch: 180, Yaw: 90
# 27	MAV_SENSOR_ROTATION_PITCH_180_YAW_270	Roll: 0, Pitch: 180, Yaw: 270
# 28	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90	Roll: 90, Pitch: 90, Yaw: 0
# 29	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90	Roll: 180, Pitch: 90, Yaw: 0
# 30	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90	Roll: 270, Pitch: 90, Yaw: 0
# 31	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180	Roll: 90, Pitch: 180, Yaw: 0
# 32	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180	Roll: 270, Pitch: 180, Yaw: 0
# 33	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270	Roll: 90, Pitch: 270, Yaw: 0
# 34	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270	Roll: 180, Pitch: 270, Yaw: 0
# 35	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270	Roll: 270, Pitch: 270, Yaw: 0
# 36	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90	Roll: 90, Pitch: 180, Yaw: 90
# 37	MAV_SENSOR_ROTATION_ROLL_90_YAW_270	Roll: 90, Pitch: 0, Yaw: 270
# 38	MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293	Roll: 90, Pitch: 68, Yaw: 293
# 39	MAV_SENSOR_ROTATION_PITCH_315	Pitch: 315
# 40	MAV_SENSOR_ROTATION_ROLL_90_PITCH_315	Roll: 90, Pitch: 315
# 100	MAV_SENSOR_ROTATION_CUSTOM	Custom orientation
# ---------------------------------------------------------------------------
# new_orientation ++	uint8_t		MAV_SENSOR_ORIENTATION	orientation after calibration.
# scale_factor ++	float			field radius correction factor


async def send_mag_cal_progress():
    await client.mav.mag_cal_progress_send(
        compass_id=0,
        cal_mask=0,
        cal_status=0,
        attempt=0,
        completion_pct=0,
        completion_mask=[0] * 10,
        direction_x=0,
        direction_y=0,
        direction_z=0,

    )

# compass_id	uint8_t			Compass being calibrated.
# Messages with same value are from the same source (instance).
# cal_mask	uint8_t			Bitmask of compasses being calibrated.
# cal_status	uint8_t		MAG_CAL_STATUS	Calibration Status.
# attempt	uint8_t			Attempt number.
# completion_pct	uint8_t	%		Completion percentage.
# completion_mask	uint8_t[10]			Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
# direction_x	float			Body frame direction vector for display.
# direction_y	float			Body frame direction vector for display.
# direction_z	float			Body frame direction vector for display.


async def send_param_value():
    pass
    # await client.mav.param_value_send(
    #     param_id=b'',
    #     param_value=0,
    #     param_type=0,
    #     param_count=0,
    #     param_index=0,

    # )

# param_id	char[16]		Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
# param_value	float		Onboard parameter value
# param_type	uint8_t	MAV_PARAM_TYPE	Onboard parameter type.
# ------------------------------------------------------------
# 1	MAV_PARAM_TYPE_UINT8	8-bit unsigned integer
# 2	MAV_PARAM_TYPE_INT8	8-bit signed integer
# 3	MAV_PARAM_TYPE_UINT16	16-bit unsigned integer
# 4	MAV_PARAM_TYPE_INT16	16-bit signed integer
# 5	MAV_PARAM_TYPE_UINT32	32-bit unsigned integer
# 6	MAV_PARAM_TYPE_INT32	32-bit signed integer
# 7	MAV_PARAM_TYPE_UINT64	64-bit unsigned integer
# 8	MAV_PARAM_TYPE_INT64	64-bit signed integer
# 9	MAV_PARAM_TYPE_REAL32	32-bit floating-point
# 10	MAV_PARAM_TYPE_REAL64	64-bit floating-point
# -------------------------------------------------------------
# param_count	uint16_t		Total number of onboard parameters
# param_index	uint16_t		Index of this onboard parameter


events: Dict[int, tuple[float, Callable]] = {
    mavlink.MAVLINK_MSG_ID_AHRS: (1/16, send_ahrs),
    mavlink.MAVLINK_MSG_ID_AHRS2: (1/16, send_ahrs2),
    mavlink.MAVLINK_MSG_ID_ATTITUDE: (1/16, send_attitude),
    mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: (1/3, send_battery_status),
    mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: (1/3, send_distance_sensor),
    mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: (1/3, send_ekf_status_report),
    mavlink.MAVLINK_MSG_ID_FENCE_STATUS: (1/2, send_fence_status),
    mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: (1/16, send_gimbal_device_attitude_status),
    mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: (1/3, send_global_position_int),
    mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: (1/2, send_gps_raw_int),
    mavlink.MAVLINK_MSG_ID_HEARTBEAT: (1/1, send_heartbeat),
    mavlink.MAVLINK_MSG_ID_MAG_CAL_PROGRESS: (1/3, send_mag_cal_progress),
    mavlink.MAVLINK_MSG_ID_MAG_CAL_REPORT: (1/3, send_mag_cal_report),
    mavlink.MAVLINK_MSG_ID_MEMINFO: (1/2, send_meminfo),
    mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: (1/2, send_nav_controller_output),
    mavlink.MAVLINK_MSG_ID_PARAM_VALUE: (1/2, send_param_value),
    mavlink.MAVLINK_MSG_ID_POWER_STATUS: (1/2, send_power_status),
    mavlink.MAVLINK_MSG_ID_RANGEFINDER: (1/3, send_rangefinder),
    mavlink.MAVLINK_MSG_ID_RAW_IMU: (1/2, send_raw_imu),
    mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW: (1/2, send_rc_channels_raw),
    mavlink.MAVLINK_MSG_ID_RC_CHANNELS: (1/2, send_rc_channels),
    mavlink.MAVLINK_MSG_ID_SCALED_IMU2: (1/2, send_scaled_imu2),
    mavlink.MAVLINK_MSG_ID_SCALED_IMU3: (1/2, send_scaled_imu3),
    mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE: (1/2, send_scaled_pressure),
    mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2: (1/2, send_scaled_pressure2),
    mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3: (1/2, send_scaled_pressure3),
    mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: (1/2, send_servo_output_raw),
    mavlink.MAVLINK_MSG_ID_SYS_STATUS: (1/2, send_sys_status),
    mavlink.MAVLINK_MSG_ID_SYSTEM_TIME: (1/3, send_system_time),
    mavlink.MAVLINK_MSG_ID_VFR_HUD: (1/16, send_vfr_hud),
    mavlink.MAVLINK_MSG_ID_VIBRATION: (1/3, send_vibration),
}
