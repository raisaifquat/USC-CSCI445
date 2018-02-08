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
        self.p_controller = PController(k_p=150.0)
        self.pd_controller = PDController(k_p=1.5, k_d=0.0025)

    def run(self):
        def f_left(error, speed) -> float:
            ctrl = self.p_controller.update(error)
            print("left ctrl: %f" % ctrl)
            if error > 0:
                # too close
                # increase
                res = speed + ctrl
            elif error < 0:
                # too far
                # increase
                res = speed - ctrl
            else:
                res = 100.0

            return clamp(res, self.p_controller.range_min,
                         self.p_controller.range_max)

        def f_right(error, speed) -> float:
            ctrl = self.p_controller.update(error)
            print("right ctrl: %f" % ctrl)
            if error > 0:
                # too close to wall
                # decrease right speed
                res = speed - ctrl
            elif error < 0:
                # too far to wall
                # decrease right speed
                res = speed + ctrl
            else:
                res = 100.0

            return clamp(res, self.p_controller.range_min,
                         self.p_controller.range_max)

        self.create.start()
        self.create.safe()

        self.servo.go_to(75)
        self.time.sleep(1)

        start_time = self.time.time()
        curr_time = start_time
        goal = 0.5
        print(goal)

        # l_goal = goal - robotProperties["wheel_base"]
        # r_goal = goal + robotProperties["wheel_base"]
        vl = 100.0
        vr = 100.0

        start_time = self.time.time()
        curr_time = start_time
        # prev_time = curr_time
        curr_state = np.array([])
        while curr_time - start_time < 100:
            # curr_state = np.min(curr_state)
            curr_state = self.sonar.get_distance()
            self.time.sleep(0.1)

            vleft = f_left(goal - curr_state, vl)
            vright = f_right(goal - curr_state, vr)
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
            self.create.drive_direct(vleft, vright)
            # self.create.drive_direct(vright, vleft)

            # prev_time = curr_time
            curr_time = self.time.time()
