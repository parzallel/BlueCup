from codecs import xmlcharrefreplace_errors

from main import BASE_GEAR

CURRENT_GEAR = BASE_GEAR


def inc_gear():
    global CURRENT_GEAR
    if CURRENT_GEAR < 3:
        CURRENT_GEAR += 1
        print(f"Gear : [{CURRENT_GEAR}]")
    else:
        print("NO MORE GEAR")


buttons = {
    "R1": 2048,
    "L1": 4096,
    "triangle": 10
}


def dec_gear():
    global CURRENT_GEAR
    if CURRENT_GEAR > 0:
        CURRENT_GEAR -= 1
        print(f"Gear : [{CURRENT_GEAR}]")
    else:
        print("NO MORE GEAR")


def thruster_speed_formatter(m1, m2, m3, m4, m5, m6):  # reformats the motor speed in order to pass to arduino
    return (f"m1={-m1 + 1500} "
            f"m2={m2 + 1500} "
            f"m3={-m3 + 1500} "
            f"m4={-m4 + 1500} "
            f"m5={-m5 + 1500} "
            f"m6={-m6 + 1500}"
            )


class Controller:
    """
    Controller class that processes cockpit and controller inputs
    to calculate appropriate motor speeds for the vehicle.
    """
    GEARS = [1, 2, 3, 4]  # available gears

    def __init__(self, msg, base_gear=0, depth_offset=0, horizontal_offset=100):

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.r = msg.r
        self.t = msg.t
        self.s = msg.s
        self.buttons = msg.buttons

        # configurable attributes
        self.BASE_GEAR = base_gear
        self.depth_offset = depth_offset
        self.horizontal_offset = horizontal_offset
        self.NASTY_OFFSET_FOR_M2 = 100

        # computed powers (will be updated when needed)
        self.thrust_power = 0
        self.vertical_thrust = self.z + self.depth_offset
        self.horizontal_thrust = self.r + self.horizontal_offset

    def acc(self):
        """Calculate thrust power based on current gear and x-axis input."""
        try:
            engaged_gear = Controller.GEARS[CURRENT_GEAR]
        except IndexError:
            print("Error: invalid gear index.")
            return 0
        else:
            if self.x > 0:
                self.thrust_power = engaged_gear *self.x
            elif self.x < 0:
                self.thrust_power = engaged_gear *self.x
            else:
                self.thrust_power = self.x
            return self.thrust_power

    def in_action(self):
        """
        Automated method that returns suitable motor speeds
        based on cockpit/controller state.
        """
        if self.buttons == buttons["R1"]:
            inc_gear()
        elif self.buttons == buttons["L1"]:
            dec_gear()
        elif self.buttons == buttons["triangle"]:
            self.thrusters_off()
        elif self.y != 0:
            return self.pivot()
        elif self.r != 0:
            return self.horizontal_move(peripheral=self.NASTY_OFFSET_FOR_M2)
        elif self.x != 0 :
            return self.move(peripheral=self.NASTY_OFFSET_FOR_M2)
        else:
            return self.move(peripheral=0)

    def horizontal_move(self, peripheral):
        x_power = self.acc()
        r_power = self.r
        z_power = self.vertical_thrust
        if -300 < x_power < 300 :
            if self.r > 0:
                return thruster_speed_formatter(m1=r_power, m2=z_power + peripheral,
                                                m3=x_power, m4=x_power,
                                                m5=z_power, m6=r_power)
            elif self.r < 0:
                return thruster_speed_formatter(m1=x_power, m2=z_power + peripheral,
                                                m3=-r_power, m4=-r_power,
                                                m5=z_power, m6=x_power)
        else:
            if self.r > 0:
                return thruster_speed_formatter(m1=x_power , m2=z_power + peripheral,
                                                m3=x_power - r_power, m4=x_power - r_power,
                                                m5=z_power, m6=x_power)
            elif self.r < 0:
                return thruster_speed_formatter(m1=x_power + r_power, m2=z_power + peripheral,
                                                m3=x_power, m4=x_power,
                                                m5=z_power, m6=x_power + r_power)

        return ""

    def pivot(self):
        """Handle pivoting movement based on y-axis input."""
        pivot_power = self.y * 2
        x_power = self.acc()
        z_power = self.vertical_thrust
        m4 = x_power
        m6 = x_power

        if -300 < x_power < 300:
            if self.y > 0:
                return thruster_speed_formatter(m1=x_power, m2=z_power,
                                                m3=x_power + pivot_power, m4=m4,
                                                m5=z_power, m6=m6)
            elif self.y < 0:
                return thruster_speed_formatter(m1=x_power - pivot_power, m2=z_power,
                                                m3=x_power, m4=m4,
                                                m5=z_power, m6=m6)
        else:
            full_speed_offset = 50
            if self.y > 0:
                return thruster_speed_formatter(m1=x_power - (pivot_power + full_speed_offset), m2=z_power,
                                                m3=x_power, m4=m4,
                                                m5=z_power, m6=m6)
            elif self.y < 0:
                return thruster_speed_formatter(m1=x_power, m2=z_power,
                                                          m3=x_power + (pivot_power - full_speed_offset), m4=m4,
                                                          m5=z_power, m6=m6)


        return ""

    def thrusters_off(self):
        """Turn off thrusters and reset gear to base gear."""
        x_power = self.acc()
        self.vertical_thrust = self.z
        z_power = self.vertical_thrust

        global CURRENT_GEAR
        CURRENT_GEAR = self.BASE_GEAR

        print("WARNING: motors are off")
        return thruster_speed_formatter(m1=x_power, m2=z_power, m3=x_power,
                                        m4=x_power, m5=z_power, m6=x_power)

    def move(self, peripheral):
        """Handle forward/backward movement."""
        x_power = self.acc()
        self.vertical_thrust = self.z + self.depth_offset
        z_power = self.vertical_thrust
        return thruster_speed_formatter(m1=x_power, m2=z_power + peripheral, m3=x_power,
                                        m4=x_power, m5=z_power, m6=x_power)
