from utils import clamp


class PDController:
    def __init__(self, k_p: float = 0.0, k_d: float = 0.0,
                 range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max

        self.k_p = k_p
        self.k_d = k_d

        self.prev_l_time = 0.0
        self.prev_r_time = 0.0
        self.prev_l_error = 0.0
        self.prev_r_error = 0.0
        pass

    def update_left(self, error: float, current_time: float) -> float:
        delta_time = current_time - self.l_current_time
        delta_error = error - self.l_error

        de_dt = 0.0 if delta_time == 0 else delta_error / delta_time

        # print("de/dt = %f" % de_dt)
        self.l_error = error
        self.l_current_time = current_time

        return clamp(current_val + error * self.k_p + self.k_d * de_dt, self.range_min, self.range_max)
