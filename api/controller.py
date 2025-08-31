import datetime
import threading
import time
# TODO : change the gear base and complete the manual controller
BASE_GEAR = 3


def save_gear(current_gear):
    with open("api/gear.txt", "w") as file:
        file.write(str(current_gear) + "\n")


def load_gear():
    with open("api/gear.txt", "r") as file:
        gear = file.readline()
        return int(gear)


def inc_gear(current_gear):
    if current_gear != 6:
        current_gear += 1
        save_gear(current_gear)
        print(f"Gear : {current_gear - 3}")
    else:
        print("Warning : NO MORE GEAR")


def dec_gear(current_gear):
    if current_gear != 0:
        current_gear -= 1
        save_gear(current_gear)
        print(f"Gear : {current_gear - 3}")
    else:
        print("Warning : NO MORE GEAR")


def motor_speed_format(m1, m2, m3, m4, m5, m6):  # reformats the motor speed in order to pass to arduino
    return f"m1={(-1 * m1) + 1500} m2={m2 + 1500} m3={(-1 * m3) + 1500} m4={(-1 * m4) + 1500} m5={(-1 * m5) + 1500} m6={(-1 * m6) + 1500}"


class Controller:
    gears = [-300, -200, -100, 0, 100, 200, 300]

    def __init__(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.r = msg.r
        self.t = msg.t
        self.s = msg.s
        self.buttons = msg.buttons

    def acc(self, current_gear):
        try:
            engaged_gear = Controller.gears[current_gear]
        except IndexError:
            print("there is sth wrong with the gears")
        else:
            return engaged_gear + self.x // 10

    def in_action(self):
        """automated method to returns the suitable motor speed for the vehicle to perform properly.
        this is based on the data from the Cockpit adn the controller itself"""

        current_gear = load_gear()

        if self.buttons == 2048:
            inc_gear(current_gear)

        elif self.buttons == 4096:
            dec_gear(current_gear)
        elif self.buttons == 10:
            self.mode_stabilize(current_gear)
        elif self.y != 0:
            return self.pivot(current_gear)

        elif self.x != 0 or current_gear !=3:  # you have to change this code later . remove the m2 assignment for the function
            return self.move(current_gear, m2=100)
        else:
            return self.move(current_gear, m2=0)

        return None

    def pivot(self, current_gear):
        y = self.y // 10
        x = self.acc(current_gear)
        if current_gear != 6:
            if y > 0:
                return motor_speed_format(m1=x, m2=int(self.z // 2.5 + 100),
                                          m3=x + y * 2,
                                          m4=x, m5=int(self.z // 2.5),
                                          m6=x)
            elif y < 0:
                return motor_speed_format(m1=x - y * 2, m2=int(self.z // 2.5 + 100),
                                          m3=x,
                                          m4=x, m5=int(self.z // 2.5),
                                          m6=x)
        elif current_gear == 6:
            if y > 0:
                return motor_speed_format(m1=x + (y + 100), m2=int(self.z // 2.5 + 100),
                                          m3=x,
                                          m4=x, m5=int(self.z // 2.5),
                                          m6=x)
            elif y < 0:
                return motor_speed_format(m1=x, m2=int(self.z // 2.5 + 100),
                                          m3=x + (y + 100),
                                          m4=x, m5=int(self.z // 2.5),
                                          m6=x)

        return None

    def mode_stabilize(self, current_gear):
        x = self.acc(current_gear)
        save_gear(BASE_GEAR)
        print("WARNING : motors are off")
        return motor_speed_format(m1=x, m2=int(self.z // 2.5), m3=x,
                                  m4=x, m5=int(self.z // 2.5), m6=x)

    def move(self, current_gear, m2):
        # m2 = int(self.z // 2.5 + 100)
        x = self.acc(current_gear)
        return motor_speed_format(m1=x, m2=int(self.z // 2.5 + m2), m3=x,
                                  m4=x, m5=int(self.z // 2.5 ), m6=x)
