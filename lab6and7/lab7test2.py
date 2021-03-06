from pyCreate2 import create2
import math
import numpy as np

import odometry
import pd_controller2
import pid_controller


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pd_controller = pid_controller.PIDController(1000, 0, 100, [-75, 75], [-200, 200], is_angle=False)

    def sleep(self, time_in_sec, is_get_dist: bool = False, interrupt=None):
        result = math.inf
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()

            if is_get_dist:
                result = min(self.sonar.get_distance(), result)

                if interrupt is not None and interrupt(result):
                    break

            if start + time_in_sec <= t:
                break

        return None if not is_get_dist else result

    def sweep_sonar(self, degree, sleep_time: float = 1.0, interrupt=None) -> float:
        curr_angle = math.degrees(self.odometry.theta)

        min_dist_to_wall = math.inf

        self.servo.go_to(curr_angle - degree)
        dist = self.sleep(sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        self.servo.go_to(curr_angle + degree)
        dist = self.sleep(sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        self.servo.go_to(curr_angle)
        dist = self.sleep(sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        return min_dist_to_wall

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        self.create.drive_direct(-100, 100)
        self.sleep(10.0)
