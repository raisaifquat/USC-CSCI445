from pyCreate2 import create2
import math
import numpy as np
from enum import Enum

import odometry
import pid_controller


class State(Enum):
    init_go_to_goal = 0,
    go_to_goal = 1,
    init_wall_following = 2,
    wall_following = 3,
    update = 4


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
        self.pidWallFollow = pid_controller.PIDController(1000, 0, 100, [-75, 75], [-200, 200], is_angle=False)

    def sleep(self, time_in_sec, is_get_dist: bool = False, interrupt=lambda x: False):
        result = math.inf
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            t = self.time.time()

            if is_get_dist:
                result = min(self.sonar.get_distance(), result)

            if start + time_in_sec <= t or interrupt(result):
                break

        return None if not is_get_dist else result

    def get_dist_to_goal(self, goal_x: float, goal_y: float) -> float:
        return math.sqrt((goal_x - self.odometry.x) ** 2 + (goal_y - self.odometry.y) ** 2)

    def go_to_angle(self, angle: float = 0, sleep_time: float = 0.5, is_get_dist: bool = False,
                    interrupt=lambda x: False):
        self.servo.go_to(angle)
        return self.sleep(sleep_time, is_get_dist=is_get_dist, interrupt=interrupt)

    def sweep_sonar(self, turn_angle, curr_angle: float = None, sleep_time: float = 1.0,
                    interrupt=lambda x: False) -> float:
        if curr_angle is None:
            curr_angle = math.degrees(self.odometry.theta)

        min_dist_to_wall = math.inf

        dist = self.go_to_angle(curr_angle - turn_angle, sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        dist = self.go_to_angle(curr_angle - turn_angle, sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        dist = self.go_to_angle(curr_angle - turn_angle, sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        self.servo.go_to(curr_angle)
        self.sleep(sleep_time)

        return min_dist_to_wall

    def go_to_goal(self, goal_x: float, goal_y: float) -> None:
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

            dist_to_goal = self.get_dist_to_goal(goal_x, goal_y)
            output_distance = self.pidDistance.update(0, dist_to_goal, self.time.time())

            v_right = int(output_theta + output_distance)
            v_left = int(-output_theta + output_distance)
            print("gtg [v_right: %.2f, v_left: %.2f]" % (v_right, v_left))
            self.create.drive_direct(v_right, v_left)

    def follow_wall(self, distance: float, goal_dist: float, base_speed: float = 100.0) -> None:
        if distance is None:
            return

        output = self.pidWallFollow.update(distance, goal_dist, self.time.time())

        v_right = int(base_speed - output)
        v_left = int(base_speed + output)
        print("fw [v_right: %.2f, v_left: %.2f]" % (v_right, v_left))
        self.create.drive_direct(v_right, v_left)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        way_points = np.array([
            [2.0, 0.0],
            [2.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.0]
        ])

        dist_threshold = 0.05
        wall_threshold = 0.3
        dist_offset_threshold = 0.05
        goal_dist_to_wall = wall_threshold
        sonar_sweep_angle = 30
        sonar_sweep_sleep_time = 0.5

        def go_to_goal_interrupt(dist_):
            return dist_ <= wall_threshold
            # return False

        def wall_follow_interrupt(dist_):
            return dist_ >= (goal_dist_to_wall * 1.1)

        curr_sonar_angle = 0
        for point in way_points:
            goal_x = point[0]
            goal_y = point[1]
            base_speed = 100

            print("Going to @{%.4f, %.4f}" % (goal_x, goal_y))

            while self.get_dist_to_goal(goal_x, goal_y) > dist_threshold:
                dist_to_wall = self.sonar.get_distance()
                # dist_to_wall = self.sweep_sonar(sonar_sweep_angle, sonar_sweep_sleep_time, curr_sonar_angle,
                #                                 interrupt=go_to_goal_interrupt)
                print("dist_to_wall: %.4f" % dist_to_wall)
                # self.go_to_angle(curr_sonar_angle, sonar_sweep_sleep_time)

                while dist_to_wall is not None and dist_to_wall > wall_threshold:
                    self.go_to_goal(goal_x, goal_y)
                    dist_to_wall = self.sonar.get_distance()
                    # dist_to_wall = self.sweep_sonar(sonar_sweep_angle, sonar_sweep_sleep_time,
                    #                                 interrupt=go_to_goal_interrupt)

                    # turn_angles = np.array(
                    #     [curr_sonar_angle - sonar_sweep_angle, curr_sonar_angle, curr_sonar_angle + sonar_sweep_angle])
                    # for turn_angle in turn_angles:
                    #     dist_to_wall = self.go_to_angle(turn_angle, sonar_sweep_sleep_time, is_get_dist=True,
                    #                                     interrupt=go_to_goal_interrupt)
                    #     print("gtg [dist_to_wall: %.4f]\n" % dist_to_wall)
                    #
                    #     if go_to_goal_interrupt(dist_to_wall):
                    #         break
                    #
                    #     self.go_to_goal(goal_x, goal_y)

                prev_dist_to_goal = self.get_dist_to_goal(goal_x, goal_y)
                dist_to_wall = self.sonar.get_distance()
                dist_offset = self.get_dist_to_goal(goal_x, goal_y) - prev_dist_to_goal

                while (dist_offset < dist_offset_threshold
                       and dist_to_wall is not None
                       and dist_to_wall < (goal_dist_to_wall * 1.1)):

                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        self.follow_wall(dist_to_wall, goal_dist_to_wall, base_speed=base_speed)

                    curr_angle = math.degrees(self.odometry.theta)
                    print("fw [dist_to_wall: %.4f]\nfw [curr_angle: %.4f]\n" % (dist_to_wall, curr_angle))

                    dist_to_wall = self.sonar.get_distance()
                    # dist_to_wall = self.go_to_angle(-(curr_angle * 5 / 6), 0.1, is_get_dist=True)
                    # dist_to_wall = self.go_to_angle(-curr_angle, 0.1, is_get_dist=True)

            # self.servo.go_to(0)
            # self.sleep(1.0)

        print("Arrived @[{%.4f},{%.4f},{%.4f}]\n" % (
            self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
