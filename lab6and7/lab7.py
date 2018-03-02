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

    def sleep(self, time_in_sec, action=None, action_args=None):
        start = self.time.time()
        while self.time.time() - start < time_in_sec:
            state = self.create.update()
            if action is not None:
                if action_args is None:
                    action()
                else:
                    action(action_args)

            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

    def go_to_goal(self, goal_x: float, goal_y: float) -> None:
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

            distance = self.dist_to_goal(goal_x, goal_y)
            output_distance = self.pidDistance.update(0, distance, self.time.time())

            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

    def dist_to_goal(self, goal_x: float, goal_y: float) -> float:
        return math.sqrt((goal_x - self.odometry.x) ** 2 + (goal_y - self.odometry.y) ** 2)

    def dist_to_wall(self) -> float:
        return self.sonar.get_distance()

    def follow_wall(self, distance: float, goal_dist: float, base_speed: float = 100.0) -> None:
        if distance is None:
            return
            
        output = self.pidWallFollow.update(distance, goal_dist, self.time.time())
        self.create.drive_direct(int(base_speed - output), int(base_speed + output))

    def sweep_sonar(self, degree, sleep_time: float = 0.01) -> float:
        curr_angle = math.degrees(self.odometry.theta)

        min_dist_to_wall = np.array([self.sonar.get_distance()])

        self.servo.go_to(curr_angle - degree)
        self.sleep(sleep_time, self.servo.go_to, curr_angle - degree)
        min_dist_to_wall = np.append(min_dist_to_wall, self.sonar.get_distance())

        self.servo.go_to(curr_angle + degree)
        self.sleep(sleep_time, self.servo.go_to, curr_angle + degree)
        min_dist_to_wall = np.append(min_dist_to_wall, self.sonar.get_distance())

        self.servo.go_to(curr_angle)
        self.sleep(sleep_time, self.servo.go_to, curr_angle)

        return min_dist_to_wall.min()

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
        wall_threshold = 0.4
        dist_offset_threshold = 0.05
        # wheel speed
        v_left = 0.0
        v_right = 0.0

        for point in way_points:
            goal_x = point[0]
            goal_y = point[1]
            base_speed = 100

            print("Going to @{%.4f, %.4f}" % (goal_x, goal_y))

            while self.dist_to_goal(goal_x, goal_y) > dist_threshold:
                dist_to_wall = self.dist_to_wall()
                print("distance to wall %.4f" % dist_to_wall)

                if dist_to_wall is not None and dist_to_wall > wall_threshold:
                    print("distance to goal %.4f\n" % self.dist_to_goal(goal_x, goal_y))
                    self.go_to_goal(goal_x, goal_y)
                    continue
                    # state = self.create.update()
                    # if state is not None:
                    #     self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

                    #     goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    #     output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    #     distance = self.dist_to_goal(goal_x, goal_y)
                    #     output_distance = self.pidDistance.update(0, distance, self.time.time())

                    #     v_right = int(output_theta + output_distance)
                    #     v_left = int(-output_theta + output_distance)

                prev_dist_to_goal = self.dist_to_goal(goal_x, goal_y)
                dist_to_wall = self.dist_to_wall()
                dist_offset = self.dist_to_goal(goal_x, goal_y) - prev_dist_to_goal
                goal_dist_to_wall = wall_threshold + 0.1

                if (dist_offset < dist_offset_threshold
                        and dist_to_wall is not None
                        and dist_to_wall < (goal_dist_to_wall * 1.1)):
                    
                    self.follow_wall(dist_to_wall, goal_dist_to_wall, base_speed=base_speed)
                    # output = self.pidWallFollow.update(dist_to_wall, goal_dist_to_wall, self.time.time())

                    # v_right = int(base_speed - output)
                    # v_left = int(base_speed + output)

                    curr_angle = math.degrees(self.odometry.theta)
                    print("current angle: %.4f\n" % curr_angle)

                    self.servo.go_to(-(curr_angle * 5 / 6))
                    self.sleep(0.1)

                self.create.drive_direct(v_right, v_left)

            # self.servo.go_to(0)
            # self.sleep(0.01)

        print("Arrived @[{%.4f},{%.4f},{%.4f}]\n" % (
            self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
