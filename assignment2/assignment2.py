from pyCreate2 import create2

import math

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
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
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

        speed = 100  # mm/s
        time_90deg_turn = 1.9  # seconds

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
