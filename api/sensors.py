class SensorFormatter:
    """reformats the data that has been read from arduino and rp for each relative sensor"""

    # TODO : give the ardu_data in dictionary format

    def __init__(self, ras_data, ardu_data):
        self.data = ardu_data  + ras_data

    def mpu_formatter(self):
        return {"roll": self.data[0],
                "pitch": self.data[1],
                "yaw": self.data[2]}

    def gps_formatter(self):
        pass

    def depth_formatter(self):
        pass

    def voltage_formatter(self):
        pass


class MPU:
    def __init__(self, mpu):

        self.roll = int(mpu.get("roll"))
        self.yaw = int(mpu.get("yaw"))
        self.pitch = int(mpu.get("pitch"))

    # (-------ROLl------)

    def roll_pos(self):
        pass

    def roll_neg(self):
        pass

    # (-------PITCH------)

    def pitch_pos(self):
        pitch = abs(self.pitch)
        while pitch >= 10:
            pitch //= 10
        truster_power = pitch * 40
        # m2 up / m5 down
        return f"m2={truster_power + 1500 } m5={(-1 * truster_power) + 1500}"

    def pitch_neg(self):
        pitch = abs(self.pitch)
        while pitch >= 10:
            pitch //= 10
        truster_power = pitch * 40
        # m2 down / m5 up
        return f"m2={truster_power + 1500} m5={(-1 * truster_power) + 1500 }"

    # (-------YAW---------)
