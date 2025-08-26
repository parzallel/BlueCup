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

        while  self.pitch >= 10 :
            self.pitch //= 10
        truster_power = self.pitch*40

        return


    def pitch_neg(self):
        pass

    # (-------YAW---------)

# TODO : write a main def and call the above functions

# TOdo  : change the attributes of the class to be variables after complete setup of sensors from sensor_handler
