from pyCreate2 import create2
import lab8_map
import math
from particle_filter import ParticleFilter
from odometry import Odometry
from pid_controller import PIDController


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
        self.pidTheta = PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)

        # constants
        self.base_speed = 100
        self.variance_sensor = 0.1
        self.variance_distance = 0.01
        self.variance_direction = 0.05
        self.world_width = 3.0  # the x
        self.world_height = 3.0  # the y

        self.filter = ParticleFilter(self.virtual_create, self.variance_sensor, self.variance_distance,
                                     self.variance_direction, num_particles=100, world_width=self.world_width,
                                     world_height=self.world_height)

    def run(self):
        # # This is an example on how to visualize the pose of our estimated position
        # # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        # self.virtual_create.set_pose((0.5, 0.5, 0.1), 0)
        #
        # # This is an example on how to show particles
        # # the format is x,y,z,theta,x,y,z,theta,...
        # data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        # self.virtual_create.set_point_cloud(data)
        #
        # # This is an example on how to estimate the distance to a wall for the given
        # # map, assuming the robot is at (0, 0) and has heading math.pi
        # print(self.map.closest_distance((0.5,0.5), 0))
        #
        # # This is an example on how to detect that a button was pressed in V-REP
        # while True:
        #     b = self.virtual_create.get_last_button()
        #     if b == self.virtual_create.Button.MoveForward:
        #         print("Forward pressed!")
        #     elif b == self.virtual_create.Button.TurnLeft:
        #         print("Turn Left pressed!")
        #     elif b == self.virtual_create.Button.TurnRight:
        #         print("Turn Right pressed!")
        #     elif b == self.virtual_create.Button.Sense:
        #         print("Sense pressed!")
        #
        #     self.time.sleep(0.01)
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # This is an example on how to detect that a button was pressed in V-REP
        while True:
            self.draw_particles()

            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.filter.move(0, 0.5)

                # 100 mm/s = 0.1 m/s
                self.create.drive_direct(self.base_speed, self.base_speed)
                self.sleep(abs(0.5 / 0.1))

                # stop
                self.create.drive_direct(0, 0)
            elif b == self.virtual_create.Button.TurnLeft:
                turn_angle = math.pi / 2 + self.odometry.theta
                turn_angle %= 2 * math.pi

                self.filter.move(turn_angle, 0)

                # turn left by 90 degree
                self.go_to_angle(turn_angle)
            elif b == self.virtual_create.Button.TurnRight:
                turn_angle = -math.pi / 2 + self.odometry.theta
                turn_angle %= 2 * math.pi

                self.filter.move(turn_angle, 0)

                # turn right by 90 degree
                self.go_to_angle(turn_angle)
            elif b == self.virtual_create.Button.Sense:
                self.filter.sense(self.sonar.get_distance())

            self.sleep(0.01)

    def go_to_angle(self, goal_theta):
        while abs(math.atan2(
                math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))
        ) > 0.01:
            print("Go TO: " + str(math.degrees(goal_theta)) + " " + str(math.degrees(self.odometry.theta)))
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
