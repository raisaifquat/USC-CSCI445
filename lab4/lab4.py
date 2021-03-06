from pyCreate2 import create2
from odometry import Odometry
from p_controller import PController
from pd_controller import PDController
from utils import clamp

import numpy as np
import matplotlib.pyplot as plt

robotProperties = {
    "diameter_left": 72,
    "diameter_right": 72,
    "wheel_base": 235,
    "encoder_count": 508.8
}


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.p_controller = PController(k_p=500.0)
        self.pd_controller = PDController(k_p=500.0, k_d=50.0)

    def run(self):
        def f_left(error, speed, curr_time_) -> float:
            ctrl = self.p_controller.update(error)
            # ctrl = self.pd_controller.update(
            #     error,
            #     error - self.pd_controller.prev_l_error,
            #     curr_time_ - self.pd_controller.prev_l_time
            # )
            print("left ctrl: %f" % ctrl)
            self.pd_controller.prev_l_error = error
            self.pd_controller.prev_l_time = curr_time_

            return clamp(speed + ctrl, self.p_controller.range_min,
                         self.p_controller.range_max)

        def f_right(error, speed, curr_time_) -> float:
            ctrl = self.p_controller.update(error)
            # ctrl = self.pd_controller.update(
            #     error,
            #     error - self.pd_controller.prev_r_error,
            #     curr_time_ - self.pd_controller.prev_r_time
            # )
            # print("right ctrl: %f" % ctrl)
            self.pd_controller.prev_r_error = error
            self.pd_controller.prev_r_time = curr_time_

            return clamp(speed - ctrl, self.p_controller.range_min,
                         self.p_controller.range_max)

        self.create.start()
        self.create.safe()

        self.servo.go_to(50)
        self.time.sleep(1)

        start_time = self.time.time()
        curr_time = start_time
        goal = 0.5
        print(goal)

        vl = 100.0
        vr = 100.0

        plt_distance = ()

        start_time = self.time.time()
        curr_time = start_time
        # prev_time = curr_time
        while curr_time - start_time < 100:
            curr_state = self.sonar.get_distance()
            self.time.sleep(0.1)

            vleft = f_left(goal - curr_state, vl, curr_time)
            vright = f_right(goal - curr_state, vr, curr_time)
            print("[curr_state = %f, error = %f\nleft: %f, right: %f]\n" % (
                curr_state,
                goal - curr_state,
                vleft,
                vright
            ))
            # self.create.drive_direct(
            #     f_left(goal - curr_state, vl),
            #     f_right(goal - curr_state, vr)
            # )
            # self.create.drive_direct(vleft, vright)
            self.create.drive_direct(
                right_wheel_velocity_in_mm_per_sec=int(vright),
                left_wheel_velocity_in_mm_per_sec=int(vleft)
            )

            # prev_time = curr_time
            curr_time = self.time.time()
