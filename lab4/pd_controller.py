class PDController:
    def __init__(self, k_p: float = 0.0, k_d: float = 0.0,
                 range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max

        self.k_p = k_p
        self.k_d = k_d

        self.l_current_time = 0.0
        self.r_current_time = 0.0
        self.l_error = 0.0
        self.r_error = 0.0
        pass

    def update_left(self, goal: float, current_val: float, current_time: float) -> float:
        error = goal - current_val
        delta_time = current_time - self.l_current_time
        delta_error = error - self.l_error

        de_dt = 0.0 if delta_time == 0 else delta_error / delta_time

        # print("de/dt = %f" % de_dt)
        self.l_error = error
        self.l_current_time = current_time

        return clamp(current_val + error * self.k_p + self.k_d * de_dt, self.range_min, self.range_max)

    def update_right(self, goal: float, current_val: float, current_time: float) -> float:
        error = goal - current_val
        delta_time = current_time - self.r_current_time
        delta_error = error - self.r_error

        de_dt = 0.0 if delta_time == 0 else delta_error / delta_time

        # print("error = %f" % error)
        # print("de/dt = %f" % de_dt)
        # print("delta_error = %f" % delta_error)
        # print("delta_time = %f" % delta_time)
        # print("de/dt = %f" % de_dt)
        self.r_error = error
        self.r_current_time = current_time

        return clamp(current_val + error * self.k_p + self.k_d * de_dt, self.range_min, self.range_max)


def clamp(value, range_min, range_max):
    return max(min(value, range_max), range_min)
