from . import ardupilotmega as mavlink
from .connection import *
from .utils import *
from config import *


__all__ = [
    'VehicleModes',
    'mavlink',
    'client',
]


client = AsyncUdp(
    device=f'{IP_ADDR}',
    source_system=mavlink.MAV_TYPE_SUBMARINE,
    source_component=mavlink.MAV_COMP_ID_AUTOPILOT1,
    broadcast=True,
    input=False,
)
