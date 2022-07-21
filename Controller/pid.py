#!/usr/bin/env python

"""

   PID-Controller

   author: Abdullah DANGAC (@abdullahdangac)

   Created 21.10.2021

"""


class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.error_prev = 0
        self.error_sum = 0


    def set_gains(self, Kp, Ki, Kd, dt=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        if dt is None:
            pass
        else:
            self.dt = dt


    def reset_pid(self):
        self.error_prev = 0
        self.error_sum = 0


    def p_control(self, error):
        u = self.Kp * error

        return u


    def pi_control(self, error):
        P = self.Kp * error
        I = self.Ki * self.error_sum * self.dt
        u = P + I

        self.error_sum += error

        return u


    def pd_control(self, error):
        P = self.Kp * error
        D = self.Kd * (error - self.error_prev) / self.dt
        u = P + D

        self.error_prev = error

        return u


    def pid_control(self, error):
        P = self.Kp * error
        I = self.Ki * self.error_sum * self.dt
        D = self.Kd * (error - self.error_prev) / self.dt
        u = P + I + D

        self.error_prev = error
        self.error_sum += error

        return u          


def main():
    pid = PID(Kp=1, Ki=1, Kd=1, dt=0.01)
    print("PID Controller")
    print("Kp: ", pid.Kp)
    print("Ki: ", pid.Ki)
    print("Kd: ", pid.Kd)
    print("dt: ", pid.dt)


if __name__=='__main__':
    try:
        main()
    except RuntimeError:
        pass
