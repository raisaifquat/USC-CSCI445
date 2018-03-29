from pyCreate2 import create2
import lab8_map
import math
import random
from enum import Enum

from particle_filter import ParticleFilter
from odometry import Odometry
from pid_controller import PIDController


class Command(Enum):
    FORWARD = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2


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
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.map = lab8_map.Map("lab8_map.json")

        self.odometry = Odometry()
        self.odometry.x = 1
        self.odometry.y = 0.5
        self.pidTheta = PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)

        # constants
        self.base_speed = 100
        self.variance_sensor = 0.1
        self.variance_distance = 0.01
        self.variance_direction = 0.05
        self.world_width = 3.0  # the x
        self.world_height = 3.0  # the y
        self.travel_dist = 0.25
        self.min_dist_to_wall = self.travel_dist + 0.2
        self.min_dist_to_localize = 0.2
        self.min_theta_to_localize = math.pi / 4
        self.n_threshold = math.log(0.09)

        self.filter = ParticleFilter(self.virtual_create, self.variance_sensor, self.variance_distance,
                                     self.variance_direction, num_particles=100, world_width=self.world_width,
                                     world_height=self.world_height, input_map=self.map, n_threshold=self.n_threshold)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        isLocalized = False
        angle_counter = 0

        # This is an example on how to detect that a button was pressed in V-REP
        while not isLocalized:
            self.draw_particles()

            # generate random num
            command = random.choice([c for c in Command])

            if command is Command.FORWARD:
                if self.sonar.get_distance() < self.min_dist_to_wall:
                    continue

                self.filter.move(0, self.travel_dist)

                # 100 mm/s = 0.1 m/s
                self.create.drive_direct(self.base_speed, self.base_speed)
                self.sleep(abs(self.travel_dist / 0.1))

                # stop
                self.create.drive_direct(0, 0)
            elif command is Command.TURN_LEFT:
                # turn_angle = math.pi / 2 + self.odometry.theta
                turn_angle = math.pi / 2 + angle_counter
                turn_angle %= 2 * math.pi
                angle_counter += math.pi / 2
                angle_counter %= 2 * math.pi

                self.filter.move(turn_angle, 0)

                # turn left by 90 degree
                self.go_to_angle(turn_angle)
            elif command is Command.TURN_RIGHT:
                # turn_angle = -math.pi / 2 + self.odometry.theta
                turn_angle = -math.pi / 2 + angle_counter
                turn_angle %= 2 * math.pi
                angle_counter -= math.pi / 2
                angle_counter %= 2 * math.pi

                self.filter.move(turn_angle, 0)

                # turn right by 90 degree
                self.go_to_angle(turn_angle)

            self.filter.sense(self.sonar.get_distance())

            # check if localized

            # distance between odometry and estimated positions
            dist_position_to_goal = math.sqrt(
                (self.odometry.x - self.filter.x) ** 2 + (self.odometry.y - self.filter.y) ** 2)
            diff_theta_to_goal = abs(self.odometry.theta - self.filter.theta)

            isLocalized = dist_position_to_goal < self.min_dist_to_localize and diff_theta_to_goal < self.min_theta_to_localize

    def go_to_angle(self, goal_theta):
        while abs(math.atan2(
                math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))
        ) > 0.1:
            # print("Go TO: " + str(math.degrees(goal_theta)) + " " + str(math.degrees(self.odometry.theta)))
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)

        self.create.drive_direct(0, 0)

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
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def draw_particles(self):
        data = []

        # get position data from all particles
        for particle in self.filter.particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])

            # paint all particles in simulation
            self.virtual_create.set_point_cloud(data)
