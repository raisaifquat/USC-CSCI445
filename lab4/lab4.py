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
        self.odometry = Odometry(
            robotProperties["diameter_left"],
            robotProperties["diameter_right"],
            robotProperties["wheel_base"],
            robotProperties["encoder_count"]
        )
        self.p_controller = PController(k_p=1.0)
        self.pd_controller = PDController(k_p=1.0, k_d=1.0)

    def run(self):

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        r_speed_arr = np.array([])
        time_arr = np.array([])

        l_goal_speed = 100
        r_goal_speed = 100
        l_update = 0
        r_update = 0

        start_time = self.time.time()
        curr_time = start_time
        prev_time = curr_time
        while curr_time - start_time < 5:
            self.create.drive_direct(l_update, r_update)
            state = self.create.update()
            if state is not None:
                print(state.__dict__)
                time_elapsed = curr_time - prev_time
                self.odometry.update(state.rightEncoderCounts, state.leftEncoderCounts, time_elapsed)
                # print("[x = %f, y = %f, angle = %f]" % (self.odometry.x,
                #                                         self.odometry.y,
                #                                         np.rad2deg(self.odometry.angle)))
                print("time_elapsed: %f\nr_speed: %f, l_speed: %f" % (
                    time_elapsed, self.odometry.r_speed, self.odometry.l_speed
                ))

                # r_update = self.p_controller.update(r_goal_speed, self.odometry.r_speed)
                # l_update = self.p_controller.update(l_goal_speed, self.odometry.l_speed)
                r_update = self.pd_controller.update(r_goal_speed, self.odometry.r_speed, curr_time)
                l_update = self.pd_controller.update(l_goal_speed, self.odometry.l_speed, curr_time)
                print("r_update_value: %f, l_update_value: %f\n" % (r_update, l_update))

                time_arr = np.append(time_arr, curr_time)
                r_speed_arr = np.append(r_speed_arr, self.odometry.r_speed)

            prev_time = curr_time
            curr_time = self.time.time()

        plt.plot(time_arr, r_speed_arr)
        plt.xlabel('time')
        plt.ylabel('right wheel speed')
        plt.grid()
        plt.show()
