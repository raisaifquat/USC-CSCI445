class PDController:
    def __init__(self, k_p: float = 0.0, k_d: float = 0.0,
                 range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max

        self.k_p = k_p
        self.k_d = k_d

        self.current_time = 0.0
        self.error = 0.0
        pass

    def update(self, error: float, current_time: float = 0.0) -> float:
        if (current_time - self.current_time) == 0:
            de_dt = 0.0
        else:
            de_dt = (error - self.error)/(current_time - self.current_time)

        # print("de/dt = %f" % de_dt)

        self.error = error
        self.current_time = current_time

        return self.error * self.k_p + self.k_d * de_dt
