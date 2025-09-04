from main import BASE_GEAR

# Current gear of the vehicle
CURRENT_GEAR = BASE_GEAR

# Button codes
BUTTONS = {
    "R1": 2048,
    "L1": 4096,
    "triangle" : 10
}

def inc_gear(max_gear=3) -> None:
    """
    Increment the current gear by 1 if below max_gear.

    Args:
        max_gear (int): Maximum gear limit.
    """
    global CURRENT_GEAR
    if CURRENT_GEAR < max_gear:
        CURRENT_GEAR += 1
        print(f"Gear: [{CURRENT_GEAR}]")
    else:
        print("NO MORE GEAR")


def dec_gear(min_gear=0)-> None:
    """
    Decrement the current gear by 1 if above min_gear.

    Args:
        min_gear (int): Minimum gear limit.
    """
    global CURRENT_GEAR
    if CURRENT_GEAR > min_gear:
        CURRENT_GEAR -= 1
        print(f"Gear: [{CURRENT_GEAR}]")
    else:
        print("NO MORE GEAR")


def motor_speed_format(m1, m2, m3, m4, m5, m6, base_speed=1500):
    """
    Format motor speeds for Arduino input.

    Args:
        m1-m6 (int): Motor speed adjustments.
        base_speed (int): Base speed offset.

    Returns:
        str: Formatted motor speed string.
    """
    return (
        f"m1={-m1 + base_speed} m2={m2 + base_speed} "
        f"m3={-m3 + base_speed} m4={-m4 + base_speed} "
        f"m5={-m5 + base_speed} m6={-m6 + base_speed}"
    )


class Controller:
    """
    Controller class to handle vehicle movement based on joystick and button input.
    """

    GEARS = [0, 100, 200, 300]

    def __init__(self, msg):
        """
        Initialize controller with joystick and button data.

        Args:
            msg: Object with attributes x, y, z, r, t, s, buttons.
        """
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.r = msg.r
        self.t = msg.t
        self.s = msg.s
        self.buttons = msg.buttons

    def acc(self) -> int:
        """
        Calculate the speed adjustment based on current gear and x-axis input.

        Returns:
            int: Speed adjustment value.
        """
        try:
            engaged_gear = Controller.GEARS[CURRENT_GEAR]
        except IndexError:
            print("Error: invalid gear configuration")
            return 0
        x_power = self.x // 10 + engaged_gear
        if self.x > 0:
            return x_power
        elif self.x < 0:
            return -x_power
        else:
            return x_power

    def in_action(self):
        """
        Determine the vehicle action based on joystick and button inputs.

        Returns:
            str or None: Formatted motor speeds or None if no action.
        """
        if self.buttons == BUTTONS["R1"]:
            inc_gear()
        elif self.buttons == BUTTONS["L1"]:
            dec_gear()
        elif self.buttons == BUTTONS["triangle"]:
            return self.thrusters_off()
        elif self.y != 0:
            return self.pivot()
        else:
            return self.move(m2=0)

        return None

    def pivot(self):
        """
        Calculate motor speeds for pivoting based on y-axis input and current gear.

        Returns:
            str or None: Formatted motor speeds for pivot action.
        """
        y_power = self.y // 10
        acc = self.acc()
        z_adj = int(self.z // 2.5)
        m2 , m5 = z_adj + 100
        pivot_power = acc + 2*y_power
        if CURRENT_GEAR != 3:
            if y_power > 0:
                return motor_speed_format(m1=acc, m2=m2, m3= pivot_power,
                                          m4=acc, m5=m5, m6=acc)
            elif y_power < 0:
                return motor_speed_format(m1=pivot_power, m2=m2, m3=acc,
                                          m4=acc, m5=m5, m6=acc)
        else:  # CURRENT_GEAR == 3
            if y_power > 0:
                return motor_speed_format(m1=pivot_power, m2=m2, m3=acc,
                                          m4=acc, m5=m5, m6=acc)
            elif y_power < 0:
                return motor_speed_format(m1=acc, m2=m2, m3=pivot_power,
                                          m4=acc, m5=m5, m6=acc)

        return None

    def thrusters_off(self):
        """
        Turn off thrusters and reset gear to base.

        Returns:
            str: Formatted motor speeds with thrusters off.
        """
        global CURRENT_GEAR
        CURRENT_GEAR = BASE_GEAR
        print("WARNING: motors are off")
        x = self.acc()
        z_adj = int(self.z // 2.5)
        return motor_speed_format(m1=x, m2=z_adj, m3=x, m4=x, m5=z_adj, m6=x)

    def move(self, m2):
        """
        Calculate motor speeds for forward/backward movement.

        Args:
            m2 (int): Additional adjustment for the second motor.

        Returns:
            str: Formatted motor speeds for movement.
        """
        x = self.acc()
        z_adj = int(self.z // 2.5)
        return motor_speed_format(m1=x, m2=z_adj + m2, m3=x, m4=x, m5=z_adj, m6=x)
# from main import BASE_GEAR
#
# CURRENT_GEAR = BASE_GEAR
#
#
# def inc_gear():
#     global CURRENT_GEAR
#     if CURRENT_GEAR < 3:
#         CURRENT_GEAR += 1
#         print(f"Gear : [{CURRENT_GEAR}]")
#     else:
#         print("NO MORE GEAR")
#
# buttons ={
#     "R1" : 2048,
#     "L1" : 4096,
# }
# def dec_gear():
#     global CURRENT_GEAR
#     if CURRENT_GEAR > 0:
#         CURRENT_GEAR -= 1
#         print(f"Gear : [{CURRENT_GEAR}]")
#     else:
#         print("NO MORE GEAR")
#
#
# def motor_speed_format(m1, m2, m3, m4, m5, m6):  # reformats the motor speed in order to pass to arduino
#     return f"m1={(-1 * m1) + 1500} m2={m2 + 1500} m3={(-1 * m3) + 1500} m4={(-1 * m4) + 1500} m5={(-1 * m5) + 1500} m6={(-1 * m6) + 1500}"
#
#
# class Controller:
#     gears = [0, 100, 200, 300]
#
#     def __init__(self, msg):
#         self.x = msg.x
#         self.y = msg.y
#         self.z = msg.z
#         self.r = msg.r
#         self.t = msg.t
#         self.s = msg.s
#         self.buttons = msg.buttons
#
#     def acc(self):
#         try:
#             engaged_gear = Controller.gears[CURRENT_GEAR]
#         except IndexError:
#             print("there is sth wrong with the gears")
#         else:
#             if self.x > 0:
#                 return engaged_gear + self.x // 10
#             elif self.x < 0:
#                 return -1 * engaged_gear + self.x // 10
#             else:
#                 return self.x // 10
#
#     def in_action(self):
#         """automated method to returns the suitable motor speed for the vehicle to perform properly.
#         this is based on the data from the Cockpit adn the controller itself"""
#
#         if self.buttons == buttons.get("R1"):
#             inc_gear()
#         elif self.buttons == buttons.get("L1"):
#             dec_gear()
#         elif self.buttons == 10:
#             self.thrusters_off()
#         elif self.y != 0:
#             return self.pivot()
#
#         elif self.x != 0 or CURRENT_GEAR != 3:
#             return self.move(m2=0)
#         else:
#             return self.move(m2=0)
#
#         return None
#
#     def pivot(self):
#         y = self.y // 10
#         x = self.acc()
#         if CURRENT_GEAR != 3:
#             if y > 0:
#                 return motor_speed_format(m1=x, m2=int(self.z // 2.5 + 100),
#                                           m3=x + y * 2,
#                                           m4=x, m5=int(self.z // 2.5),
#                                           m6=x)
#             elif y < 0:
#                 return motor_speed_format(m1=x - y * 2, m2=int(self.z // 2.5 + 100),
#                                           m3=x,
#                                           m4=x, m5=int(self.z // 2.5),
#                                           m6=x)
#         elif CURRENT_GEAR == 3:
#             if y > 0:
#                 return motor_speed_format(m1=x + (y + 100), m2=int(self.z // 2.5 + 100),
#                                           m3=x,
#                                           m4=x, m5=int(self.z // 2.5),
#                                           m6=x)
#             elif y < 0:
#                 return motor_speed_format(m1=x, m2=int(self.z // 2.5 + 100),
#                                           m3=x + (y + 100),
#                                           m4=x, m5=int(self.z // 2.5),
#                                           m6=x)
#
#         return None
#
#     def thrusters_off(self):
#         x = self.acc()
#         global CURRENT_GEAR
#         CURRENT_GEAR = BASE_GEAR
#         print("WARNING : motors are off")
#         return motor_speed_format(m1=x, m2=int(self.z // 2.5), m3=x,
#                                   m4=x, m5=int(self.z // 2.5), m6=x)
#
#     def move(self, m2):
#         x = self.acc()
#         return motor_speed_format(m1=x, m2=int(self.z // 2.5 + m2), m3=x,
#                                   m4=x, m5=int(self.z // 2.5), m6=x)