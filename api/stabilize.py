from . import sensors


# TODO : write an algorithm to stabilize the robot
# TODO : write the m2 and m5 handler

class Stabilize:

    def __init__(self, mpu):
        self.is_pitched = False
        self.is_yawed = False
        self.mpu = mpu

    def make_stable(self):
        return self.pitched()

    def pitched(self):

        if self.mpu.pitch > 10:
            self.is_pitched = True
            return self.mpu.pitch_pos()
        elif self.mpu.pitch < -10:
            self.is_pitched = True
            return self.mpu.pitch_neg()
        return None

    def warning(self ):
        if self.is_pitched :
            print("WARNING : Stabilizing the robot for --PITCH-- ")
