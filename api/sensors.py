class SensorFormatter:
    """reformats the data that has been read from arduino and rp for each relative sensor"""

    def __init__(self, ardu_data):
        self.roll = ardu_data[0]
        self.pitch = ardu_data[1]
        self.yaw = ardu_data[2]

    def mpu_formatter(self):
        return {"roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw}

    def gps_formatter(self):
        pass

    def depth_formatter(self):
        pass

    def voltage_formatter(self):
        pass


class MPU:
    def __init__(self, mpu):

        self.roll = int(mpu["roll"])
        self.pitch = int(mpu["pitch"])
        self.yaw = int(mpu["yaw"])

    # (-------ROLl------)

    def roll_pos(self):
        pass

    def roll_neg(self):
        pass

    # (-------PITCH------)

    def get_pitch_power(self):
        pitch = abs(self.pitch)
        t_power = 40  # change this for further power management
        while pitch >= 10:
            pitch //= 10
        truster_power = pitch * t_power
        return truster_power

    # (-------YAW---------)
    def get_yaw_power(self , diff):
        diff = abs(diff)
        t_power = 20  # change this for further power management
        while diff >= 10:
            diff //= 10
        truster_power = diff * t_power
        return truster_power

