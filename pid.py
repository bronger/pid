#!/usr/bin/env python

import time


class PIDController:

    def __init__(self, K_p_critical, T_critical, timer=time.time, mode="PID"):
        if mode == "PID":
            self.K_p = 0.6 * K_p_critical
            self.T_i = T_critical / 2
            self.T_d = T_critical / 8
            self.mode = "PID"
        elif mode == "PI":
            self.K_p = 0.45 * K_p_critical
            self.T_i = T_critical / 1.2
            self.mode = "PI"
        elif mode == "PD":
            self.K_p = 0.8 * K_p_critical
            self.T_d = T_critical / 8
            self.mode = "PD"
        elif mode == "P":
            self.K_p = 0.5 * K_p_critical
            self.mode = "P"
        elif mode == "P_max":
            self.K_p = K_p_critical
            self.mode = "P"
        else:
            raise TypeError
        self.mode = mode
        self.time = None
        self.timer = timer

    @property
    def desired_value(self):
        return self._desired_value

    @desired_value.setter
    def desired_value(self, value):
        self._desired_value = value
        self.time = None
        self.integral = None
        self.error = None

    def _make_step(self):
        error = self.desired_value - self._current_value
        time_ = self.timer()
        if self.time is None:
            self.integral = 0
            self.differential = 0
        else:
            if time_ <= self.time:
                return
            self.integral += (error + self.error) / 2 * (time_ - self.time)
            self.differential = (error - self.error) / (time_ - self.time)
        self.error = error
        self.time = time_

    @property
    def current_value(self):
        return self._current_value

    @current_value.setter
    def current_value(self, value):
        self._current_value = value
        self._make_step()

    @property
    def control_variable(self):
        self._make_step()
        terms = set()
        if "P" in self.mode:
            terms.add(self.error)
        if "I" in self.mode:
            terms.add(1 / self.T_i * self.integral)
        if "D" in self.mode:
            terms.add(self.T_d * self.differential)
        return self.K_p * sum(terms)
 

class AirBody:

    def __init__(self):
        self.layer_temperatures = 10 * [0]
        self.γ = 0.1

    def update(self, T_left):
        self.layer_temperatures[0] = T_left
        for i in range(len(self.layer_temperatures) - 1):
            E = self.γ * (self.layer_temperatures[i] - self.layer_temperatures[i + 1])
            self.layer_temperatures[i + 1] += E
            self.layer_temperatures[i] -= E

    @property
    def T_left(self):
        return self.layer_temperatures[0]

    @property
    def T_right(self):
        return self.layer_temperatures[-1]


air_body = AirBody()
pid = PIDController(14, 230, lambda: t)
pid.desired_value = 1

t = 0
while t < 1000:
    t += 1
    pid.current_value = air_body.T_right
    air_body.update(pid.control_variable)
    print(t, pid.current_value)
