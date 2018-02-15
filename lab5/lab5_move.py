from pyCreate2 import create2

import numpy as np
import matplotlib.pyplot as plt

from odometry import Odometry
from pd_controller import PDController
from pid_controller import PIDController
from utils import dist


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
        self.pid_controller = PIDController(500, 100, 10, -100, 100, -100, 100)
        self.pid_controller_dist = PIDController(500, 100, 10, -100, 100, -100, 100)

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
        plt_goal_angle_arr = np.array([])

        goal_x = -0.5
        goal_y = -0.5
        goal_coor = np.array([goal_x, goal_y])
        goal_angle = np.arctan2(goal_y, goal_x)
        goal_angle %= 2 * np.pi
        print("goal angle = %f" % np.rad2deg(goal_angle))
        base_speed = 100
        # timeout = abs(17 * (goal_angle / np.pi)) + 1
        goal_r = 0.05

        angle = self.odometry.theta
        dist_to_goal = dist(goal_coor, np.array([self.odometry.x, self.odometry.y]))

        plt_time_arr = np.append(plt_time_arr, self.time.time())
        plt_angle_arr = np.append(plt_angle_arr, angle)
        plt_goal_angle_arr = np.append(plt_goal_angle_arr, goal_angle)

        while abs(dist_to_goal) > goal_r:
            goal_angle = np.arctan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            goal_angle %= 2 * np.pi
            print("(%f, %f), dist to goal = %f\n" % (self.odometry.x, self.odometry.y, dist_to_goal))
            dist_to_goal = dist(goal_coor, np.array([self.odometry.x, self.odometry.y]))
            angle = self.odometry.theta

            plt_angle_arr = np.append(plt_angle_arr, angle)
            plt_goal_angle_arr = np.append(plt_goal_angle_arr, goal_angle)

            # output = self.pd_controller.update(angle, goal_angle, self.time.time())
            output = self.pid_controller.update(angle, goal_angle, self.time.time())
            output_dist = self.pid_controller_dist.update(0, dist_to_goal, self.time.time())
            # print("angle =%f, output = %f" % (np.rad2deg(angle), output))
            # print("[r = %f, l = %f]\n" % (int(base_speed + output), int(base_speed - output)))
            self.create.drive_direct(
                int(output_dist + output),
                int(output_dist - output)
            )
            self.sleep(0.01)

        np.savetxt("timeOutput.csv", plt_time_arr, delimiter=",")
        np.savetxt("angleOutput.csv", plt_angle_arr, delimiter=",")
        np.savetxt("goalAngleOutput.csv", plt_goal_angle_arr, delimiter=",")
