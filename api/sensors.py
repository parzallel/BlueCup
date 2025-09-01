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

    def pitch_pos(self):
        pitch = abs(self.pitch)
        while pitch >= 10:
            pitch //= 10
        truster_power = pitch * 40
        # m2 up / m5 down
        return f"m2={1500 - truster_power} m5={(-1 * truster_power) + 1500}"

    def pitch_neg(self):
        pitch = abs(self.pitch)
        while pitch >= 10:
            pitch //= 10
        truster_power = pitch * 40
        # m2 down / m5 up
        return f"m2={truster_power + 1500} m5={truster_power + 1500}"

    # (-------YAW---------)
