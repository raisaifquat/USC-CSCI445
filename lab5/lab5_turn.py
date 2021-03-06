from pyCreate2 import create2

import numpy as np

from odometry import Odometry
from pd_controller import PDController
from pid_controller import PIDController


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

        self.odometry = Odometry()
        self.pd_controller = PDController(500, 100, -75, 75)
        # self.pid_controller = PIDController(500, 100, 0, -75, 75, -50, 50)
        self.pid_controller = PIDController(250, 30, 1, -100, 100, -100, 100)

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, np.rad2deg(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        plt_time_arr = np.array([])
        plt_angle_arr = np.array([])

        goal_angle = np.pi / 2
        goal_angle %= 2 * np.pi
        base_speed = 0
        timeout = abs(17 * (goal_angle / np.pi)) + 1

        angle = self.odometry.theta
        plt_time_arr = np.append(plt_time_arr, self.time.time())
        plt_angle_arr = np.append(plt_angle_arr, angle)

        start_time = self.time.time()
        while self.time.time() - start_time < timeout:
            angle = self.odometry.theta
            plt_time_arr = np.append(plt_time_arr, self.time.time())
            plt_angle_arr = np.append(plt_angle_arr, angle)

            # output = self.pd_controller.update(angle, goal_angle, self.time.time())
            output = self.pid_controller.update(angle, goal_angle, self.time.time())
            # print("angle =%f, output = %f" % (np.rad2deg(angle), output))
            # print("[r = %f, l = %f]\n" % (int(base_speed + output), int(base_speed - output)))
            self.create.drive_direct(int(base_speed + output), int(base_speed - output))
            self.sleep(0.01)

        np.savetxt("timeOutput.csv", plt_time_arr, delimiter=",")
        np.savetxt("angleOutput.csv", plt_angle_arr, delimiter=",")

