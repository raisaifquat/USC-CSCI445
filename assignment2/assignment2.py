from pyCreate2 import create2

import numpy as np
import matplotlib.pyplot as plt

from odometry import Odometry


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.odometry = Odometry()

        self.x_arr = np.array([])
        self.y_arr = np.array([])
        self.x_arr_truth = np.array([])
        self.y_arr_truth = np.array([])
        self.time_arr = np.array([])

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        t = start
        while t - start < time_in_sec:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, np.rad2deg(self.odometry.theta)))
                self.time_arr = np.append(self.time_arr, t)
                self.x_arr = np.append(self.x_arr, self.odometry.x)
                self.y_arr = np.append(self.y_arr, self.odometry.y)
                self.x_arr_truth = np.append(self.x_arr_truth, self.create.sim_get_position()[0])
                self.y_arr_truth = np.append(self.y_arr_truth, self.create.sim_get_position()[1])
            t = self.time.time()

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        speed = 300  # mm/s
        time_90deg_turn = 1.9 / (speed / 100.0)  # seconds

        # move forward for 1m
        self.create.drive_direct(speed, speed)
        self.sleep(1000 / speed)

        # turn left 90 deg
        self.create.drive_direct(speed, -speed)
        self.sleep(time_90deg_turn)

        # move forward for 0.5m
        self.create.drive_direct(speed, speed)
        self.sleep(500 / speed)

        # turn left 90 deg
        self.create.drive_direct(speed, -speed)
        self.sleep(time_90deg_turn)

        # move forward for 1m
        self.create.drive_direct(speed, speed)
        self.sleep(1000 / speed)

        # turn left 90 deg
        self.create.drive_direct(speed, -speed)
        self.sleep(time_90deg_turn)

        # move forward for 0.5m
        self.create.drive_direct(speed, speed)
        self.sleep(500 / speed)

        # stop simulation
        self.create.stop()

        x_offset = self.x_arr_truth[np.argwhere(self.time_arr > 0.05)[0]]
        y_offset = self.y_arr_truth[np.argwhere(self.time_arr > 0.05)[0]]

        plt.plot(self.time_arr, self.x_arr + x_offset, label='x coor measured')
        plt.plot(self.time_arr, self.x_arr_truth, '--', label='x coor truth')
        plt.plot(self.time_arr, self.y_arr + y_offset, label='y coor measured')
        plt.plot(self.time_arr, self.y_arr_truth, '--', label='y coor truth')

        # plt.plot(self.x_arr + x_offset, self.y_arr + y_offset, label='coor measured')
        # plt.plot(self.x_arr_truth, self.y_arr_truth, '--', label='coor truth')
        plt.legend()
        plt.grid()
        plt.show()
