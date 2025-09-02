from . import sensors


# TODO : write an algorithm to stabilize the robot
# ToDo : write a code to stable the robot in saved yaw

class Stabilize:

    def __init__(self, mpu):

        self.mpu = mpu

    def make_stable(self):

        return self.pitched()

    def pitched(self):

        if self.mpu.pitch > 10:
            print("WARNING : Stabilizing the robot for --PITCH-- ")
            return self.mpu.pitch_pos()
        elif self.mpu.pitch < -10:
            print("WARNING : Stabilizing the robot for --PITCH-- ")
            return self.mpu.pitch_neg()

        return "m2=1500 m5=1500"
