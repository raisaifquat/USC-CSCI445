"""
Sample Code for Lab3
Use "run.py [--sim] lab3" to execute
"""
from pyCreate2 import create2
# from my_robot import MyRobot
from odometry import Odometry
import numpy as np

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
        # self.my_robot = MyRobot(None, self.create, self.time, self.odometry)
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.prev_r_count = 0
        self.prev_l_count = 0

    def print_odometry(self, state_):
        delta_r = self.odometry.get_delta_r(state_.rightEncoderCounts - self.prev_r_count)
        delta_l = self.odometry.get_delta_l(state_.leftEncoderCounts - self.prev_l_count)
        self.prev_r_count = state_.rightEncoderCounts
        self.prev_l_count = state_.leftEncoderCounts

        delta_theta = self.odometry.get_delta_theta(delta_r=delta_r, delta_l=delta_l)
        delta_d = self.odometry.get_delta_d(delta_r=delta_r, delta_l=delta_l)

        print("[delta_r = %f, delta_l = %f, delta_d = %f, delta_theta = %f]" % (delta_r, delta_l, delta_d, delta_theta))
        self.x += delta_d * np.cos(self.angle)
        self.y += delta_d * np.sin(self.angle)
        self.angle += delta_theta
        print("[x = %f, y = %f, angle = %f]\n" % (self.x,
                                                  self.y,
                                                  np.rad2deg(self.angle)))

    def run(self):
        def move_robot(is_print: bool = False):
            # move forward
            self.my_robot.forward(5 * 0.1, is_print=is_print)

            # left turn (in place)
            self.my_robot.turn_left(2, is_print=is_print)

            # wait
            self.my_robot.wait(2, is_print=is_print)

            # right turn (in place)
            self.my_robot.turn_right(2, is_print=is_print)

            # wait
            self.my_robot.wait(2, is_print=is_print)

            # move forward
            self.my_robot.forward(2 * 0.1, is_print=is_print)

            # move forward while turning
            self.my_robot.move(0.2, 0.1, 7.5, is_print=is_print)

            # move forward
            self.my_robot.forward(5 * 0.1, is_print=is_print)

            # move backwards
            self.my_robot.backward(5 * 0.1, is_print=is_print)

            # stop
            self.my_robot.wait(3, is_print=is_print)

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # move forward
        while self.time.time() < 10:
            self.create.drive_direct(50, 50)
            state = self.create.update()
            if state is not None:
                print(state.__dict__)
                self.print_odometry(state)

        print("\n--------------reverse----------------")
        start_time = self.time.time()
        while self.time.time() - start_time < 10:
            self.create.drive_direct(-50, -50)
            state = self.create.update()
            if state is not None:
                print(state.__dict__)
                self.print_odometry(state)

        print("\n--------------turning----------------")
        start_time = self.time.time()
        while self.time.time() - start_time < 10:
            self.create.drive_direct(50, -50)
            state = self.create.update()
            if state is not None:
                print(state.__dict__)
                self.print_odometry(state)

