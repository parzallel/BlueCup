# TODO : write an algorithm to stabilize the robot
# ToDo : write a code to stable the robot in saved yaw

class Stabilize:

    def __init__(self, mpu, saved_yaw):
        self.mpu = mpu
        self.saved_yaw = saved_yaw
        self.pitch_error = 6
        self.yaw_error = 8

    def make_stable(self):
        return self.get_tilt_direction() + self.get_turn_direction()

    def get_tilt_direction(self):

        if self.mpu.pitch > self.pitch_error:

            print("WARNING : Tilting the Penguin down  ")
            truster_power = self.mpu.get_pitch_power()
            return f"m2={1500 - truster_power} m5={(-1 * truster_power) + 1500}"

        elif self.mpu.pitch < -self.pitch_error:

            print("WARNING : Tilting the Penguin up ")
            truster_power = self.mpu.get_pitch_power()
            return f"m2={truster_power + 1500} m5={truster_power + 1500}"

        return ""

    def get_turn_direction(self):
        """
        Compare current yaw with saved yaw and return whether to turn left or right
        to realign with saved_yaw. Yaw values are circular in [0, 360].
        """
        if self.saved_yaw is None:
            return ""
        # Normalize values to [0, 360)
        current = self.mpu.yaw % 360
        target = int(self.saved_yaw) % 360

        # Calculate smallest angular difference (-180 to 180)
        diff = (target - current + 540) % 360 - 180
        # if robot turned wrongly should change the function calls

        if diff > self.yaw_error:
            truster_power = self.mpu.get_yaw_power(diff)
            print("WARNING : Turning the Penguin to left")
            return f"m1={(-1 * truster_power) + 1500}"  # Turning left reduces yaw difference
        elif diff < -self.yaw_error:
            truster_power = self.mpu.get_yaw_power(diff)
            print("WARNING : Turning the Penguin to right")
            return f"m1={(-1 * truster_power) + 1500}"
        return ""
