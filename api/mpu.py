class MPU:
    def __init__(self, mpu):
        self.roll = int(mpu.roll)
        self.yaw = int(mpu.yaw)
        self.pitch = int(mpu.pitch)

    # TODO : write function names to handle sudden robot movement

    # (-------ROLl------)

    def roll_pos(self):
        pass

    def roll_neg(self):
        pass

    # (-------PITCH------)

    def pitch_pos(self):
        pitch = abs(self.pitch)
        while  pitch >= 10 :
            pitch //= 10
        truster_power = pitch*40
        #m2 up / m5 down
        return f"m2={truster_power} m5={-1 * truster_power}"


    def pitch_neg(self):
        pitch = abs(self.pitch)
        while  pitch >= 10 :
            pitch //= 10
        truster_power = pitch*40
        #m2 down / m5 up
        return f"m2={truster_power} m5={-1 * truster_power}"

    # (-------YAW---------)


# TODO : write a main def and call the above functions

# TOdo  : change the attributes of the class to be variables after complete setup of sensors from sensor_handler
