from . import mavlink
from .utils import *
import asyncio
import socket
import errno
import time

__all__ = [
    'AsyncUdp',
    'UDP_MAX_PACKET_LEN',
    'MAX_UINT32',
]

# maximum packet length for a single receive call - use the UDP limit
UDP_MAX_PACKET_LEN = 65535
MAX_UINT32 = 0xFFFFFFFF


class AsyncUdp:
    '''a UDP mavlink socket'''

    def __init__(self, device, input=True, broadcast=False, source_system=255, source_component=0, notimestamps=False, use_native=False, timeout=5):
        a = device.split(':')
        if len(a) != 2:
            raise ValueError("UDP ports must be specified as host:port")
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server = input
        self.broadcast = False
        if input:
            self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.port.bind((a[0], int(a[1])))
        else:
            self.destination_addr = (a[0], int(a[1]))
            if broadcast:
                self.port.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.broadcast = True
        set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)
        self.last_address = None
        self.timeout = timeout
        self.clients = set()
        self.clients_last_alive = {}
        self.resolved_destination_addr = None

        self.fd = self.port.fileno()
        self.sysid = 0
        self.param_sysid = (0, 0)
        self.address = device
        self.timestamp = 0
        self.last_seq = {}
        self.mav_loss = 0
        self.mav_count = 0
        self.param_fetch_start = 0

        # state for each sysid
        self.sysid_state = {}
        self.sysid_state[self.sysid] = mavfile_state()

        # param state for each sysid/compid tuple
        self.param_state = {}
        self.param_state[self.param_sysid] = param_state()

        # status of param fetch, indexed by sysid,compid tuple
        self.source_system = source_system
        self.source_component = source_component
        self.first_byte = True
        self.robust_parsing = True
        self.mav = mavlink.MAVLink(self, srcSystem=self.source_system,
                                   srcComponent=self.source_component, use_native=use_native)
        self.mav.robust_parsing = self.robust_parsing
        self.logfile = None
        self.logfile_raw = None
        self.start_time = time.time()
        self.message_hooks = {}
        self.idle_hooks = []
        self.uptime = 0.0
        self.notimestamps = notimestamps
        self._timestamp = None
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        self.stop_on_EOF = False
        self.portdead = False

    @property
    def target_system(self):
        return self.sysid

    @target_system.setter
    def target_system(self, value):
        self.sysid = value
        if not self.sysid in self.sysid_state:
            self.sysid_state[self.sysid] = mavfile_state()
        if self.sysid != self.param_sysid[0]:
            self.param_sysid = (self.sysid, self.param_sysid[1])
            if not self.param_sysid in self.param_state:
                self.param_state[self.param_sysid] = param_state()

    @property
    def target_component(self):
        return self.param_sysid[1]

    @target_component.setter
    def target_component(self, value):
        if value != self.param_sysid[1]:
            self.param_sysid = (self.param_sysid[0], value)
            if not self.param_sysid in self.param_state:
                self.param_state[self.param_sysid] = param_state()

    @property
    def params(self):
        if self.param_sysid[1] == 0:
            eff_tuple = (self.sysid, 1)
            if eff_tuple in self.param_state:
                return getattr(self.param_state[eff_tuple], 'params')
        return getattr(self.param_state[self.param_sysid], 'params')

    @property
    def messages(self):
        return getattr(self.sysid_state[self.sysid], 'messages')

    def boot_time_usec(self):
        return int(self.start_time * 1e6) % (MAX_UINT32 + 1)

    def boot_time_ms(self):
        return int(self.start_time * 1e3) % (MAX_UINT32 + 1)

    def close(self):
        self.port.close()

    async def recv(self, n=None):
        try:
            loop = asyncio.get_running_loop()
            data, new_addr = await loop.sock_recvfrom(
                self.port, UDP_MAX_PACKET_LEN)
        except socket.error as e:
            if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                return ""
            raise
        if self.udp_server:
            self.clients.add(new_addr)
            self.clients_last_alive[new_addr] = time.time()
        elif self.broadcast:
            self.last_address = new_addr
        return data

    async def write(self, buf):
        try:
            loop = asyncio.get_running_loop()
            if self.udp_server:
                current_time = time.time()
                to_remove = set()
                for address in self.clients:
                    if self.timeout <= 0 or self.clients_last_alive[address] + self.timeout > current_time:
                        to_remove.add(address)
                        self.clients_last_alive.pop(address)
                    else:
                        await loop.sock_sendto(self.port, buf, address)
                self.clients -= to_remove
            else:
                if self.last_address and self.broadcast:
                    self.destination_addr = self.last_address
                    self.broadcast = False
                    await loop.sock_connect(self.port, self.destination_addr)
                # turn a (possible) hostname into an IP address to
                # avoid resolving the hostname for every packet sent:
                if self.destination_addr[0] != self.resolved_destination_addr:
                    self.resolved_destination_addr = self.destination_addr[0]
                    self.destination_addr = (socket.gethostbyname(
                        self.destination_addr[0]), self.destination_addr[1])
                await loop.sock_sendto(self.port, buf, self.destination_addr)
        except socket.error:
            pass

    def post_message(self, msg):
        '''default post message call'''
        if '_posted' in msg.__dict__:
            return
        msg._posted = True
        msg._timestamp = time.time()
        type = msg.get_type()

        if 'usec' in msg.__dict__:
            self.uptime = msg.usec * 1.0e-6
        if 'time_boot_ms' in msg.__dict__:
            self.uptime = msg.time_boot_ms * 1.0e-3

        if self._timestamp is not None:
            if self.notimestamps:
                msg._timestamp = self.uptime
            else:
                msg._timestamp = self._timestamp

        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()
        src_tuple = (src_system, src_component)

        if not src_system in self.sysid_state:
            # we've seen a new system
            self.sysid_state[src_system] = mavfile_state()

        add_message(self.sysid_state[src_system].messages, type, msg)

        if not msg.get_msgId() < 0:
            # Don't use unknown messages to calculate number of lost packets
            if not src_tuple in self.last_seq:
                last_seq = -1
            else:
                last_seq = self.last_seq[src_tuple]
            seq = (last_seq+1) % 256
            seq2 = msg.get_seq()
            if seq != seq2 and last_seq != -1:
                diff = (seq2 - seq) % 256
                self.mav_loss += diff
                # print("lost %u seq=%u seq2=%u last_seq=%u src_tupe=%s %s" % (diff, seq, seq2, last_seq, str(src_tuple), msg.get_type()))
            self.last_seq[src_tuple] = seq2
            self.mav_count += 1

        self.timestamp = msg._timestamp
        if type == 'HEARTBEAT':
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
                self.sysid_state[src_system].flightmode = mode_string_v10(
                    msg)
                self.sysid_state[src_system].armed = (
                    msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.sysid_state[src_system].mav_type = msg.type
                self.sysid_state[src_system].mav_autopilot = msg.autopilot
        elif type == 'HIGH_LATENCY2':
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            if msg.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.sysid_state[src_system].base_mode = msg.custom0
                self.sysid_state[src_system].armed = (
                    msg.custom0 & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.sysid_state[src_system].flightmode = mode_string_v10(msg)
            self.sysid_state[src_system].mav_type = msg.type
            self.sysid_state[src_system].mav_autopilot = msg.autopilot

        elif type == 'PARAM_VALUE':
            if not src_tuple in self.param_state:
                self.param_state[src_tuple] = param_state()
            self.param_state[src_tuple].params[msg.param_id] = msg.param_value

        if (msg.get_signed() and
            self.mav.signing.link_id == 0 and
            msg.get_link_id() != 0 and
            self.target_system == msg.get_srcSystem() and
                self.target_component == msg.get_srcComponent()):
            # change to link_id from incoming packet
            self.mav.signing.link_id = msg.get_link_id()

    async def recv_msg(self):
        '''message receive routine for UDP link'''
        s = await self.recv()

        m = self.mav.parse_char(s)
        if m is not None:
            self.post_message(m)

        return m
