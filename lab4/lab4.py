from pyCreate2 import create2
from odometry import Odometry
from p_controller import PController
from pd_controller import PDController

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
        self.odometry = Odometry(
            robotProperties["diameter_left"],
            robotProperties["diameter_right"],
            robotProperties["wheel_base"],
            robotProperties["encoder_count"]
        )
        self.p_controller = PController(k_p=1.0)
        self.pd_controller = PDController(k_p=1.5, k_d=0.0025)

    def run(self):
        def f_left(error, speed) -> float:
            return self.p_controller.update(error, speed)

        def f_right(error, speed) -> float:
            return self.p_controller.update(error, speed)

        self.create.start()
        self.create.safe()

        # request sensors
        # self.create.start_stream([
        #     create2.Sensor.LeftEncoderCounts,
        #     create2.Sensor.RightEncoderCounts,
        # ])

        self.servo.go_to(90)
        self.time.sleep(1)

        start_time = self.time.time()
        curr_time = start_time
        goal = np.array([])
        while curr_time - start_time < 2:
            goal = np.append(goal, self.sonar.get_distance())
            # print(self.sonar.get_distance())
            self.time.sleep(0.1)
            curr_time = self.time.time()

        print(goal)
        goal = np.average(goal)
        print(goal)

        l_goal = goal - robotProperties["wheel_base"]
        r_goal = goal + robotProperties["wheel_base"]
        vl = 100.0
        vr = 100.0

        start_time = self.time.time()
        curr_time = start_time
        # prev_time = curr_time
        while curr_time - start_time < 10:
            curr_state = self.sonar.get_distance()
            self.time.sleep(0.1)
            print("[left: %f, right: %f]\n" % (f_left(l_goal - curr_state, vl), f_right(r_goal - curr_state, vr)))
            self.create.drive_direct(f_left(goal - curr_state, vl), f_right(goal - curr_state, vr))

            # prev_time = curr_time
            curr_time = self.time.time()
