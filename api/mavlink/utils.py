from . import mavlink

from enum import Enum
import copy

__all__ = [
    'mode_string_v10',
    'add_message',
    'set_close_on_exec',
    'VehicleModes',
    'mavfile_state',
    'param_state'
]


def mode_string_v10(msg):
    '''mode string for 1.0 protocol, from heartbeat'''
    return f"Mode({VehicleModes(msg.custom_mode).name})"


def add_message(messages, mtype, msg):
    '''add a msg to array of messages, taking account of instance messages'''
    if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:
        # simple case, no instance field
        messages[mtype] = msg
        return
    instance_value = getattr(msg, msg._instance_field)
    if not mtype in messages:
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = {}
        messages[mtype]._instances[instance_value] = msg
        messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)
        return
    messages[mtype]._instances[instance_value] = msg
    prev_instances = messages[mtype]._instances
    messages[mtype] = copy.copy(msg)
    messages[mtype]._instances = prev_instances
    messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)


def set_close_on_exec(fd):
    '''set the close on exec flag on a file descriptor. Ignore exceptions'''
    try:
        import fcntl
        flags = fcntl.fcntl(fd, fcntl.F_GETFD)
        flags |= fcntl.FD_CLOEXEC
        fcntl.fcntl(fd, fcntl.F_SETFD, flags)
    except Exception:
        pass


class VehicleModes(Enum):
    # Mode not set by vehicle yet
    PRE_FLIGHT = -1
    # Manual angle with manual depth/throttle
    STABILIZE = 0
    # Manual body-frame angular rate with manual depth/throttle
    ACRO = 1
    # Manual angle with automatic depth/throttle
    ALT_HOLD = 2
    # Fully automatic waypoint control using mission commands
    AUTO = 3
    # Fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    GUIDED = 4
    # Automatic circular flight with automatic throttle
    CIRCLE = 7
    # Automatically return to surface, pilot maintains horizontal control
    SURFACE = 9
    # Automatic position hold with manual override, with automatic throttle
    POSHOLD = 16
    # Pass-through input with no stabilization
    MANUAL = 19
    # Automatically detect motors orientation
    MOTOR_DETECT = 20
    # Manual angle with automatic depth/throttle (from rangefinder altitude)
    SURFTRAK = 21


class mavfile_state(object):
    '''state for a particular system id'''

    def __init__(self):
        self.messages = {'MAV': self}
        self.flightmode = "UNKNOWN"
        self.vehicle_type = "UNKNOWN"
        self.mav_type = mavlink.MAV_TYPE_FIXED_WING
        self.mav_autopilot = mavlink.MAV_AUTOPILOT_GENERIC
        self.base_mode = 0
        self.armed = False  # canonical arm state for the vehicle as a whole

        if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
            try:
                self.messages['HOME'] = mavlink.MAVLink_gps_raw_int_message(
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            except AttributeError:
                # may be using a minimal dialect
                pass
            try:
                mavlink.MAVLink_waypoint_message = mavlink.MAVLink_mission_item_message
            except AttributeError:
                # may be using a minimal dialect
                pass
        else:
            try:
                self.messages['HOME'] = mavlink.MAVLink_gps_raw_message(
                    0, 0, 0, 0, 0, 0, 0, 0, 0)
            except AttributeError:
                # may be using a minimal dialect
                pass


class param_state(object):
    '''state for a particular system id/component id pair'''

    def __init__(self):
        self.params = {}
