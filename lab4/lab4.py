from pyCreate2 import create2
from p_controller import PController
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
        self.p_controller = PController()

    def run(self):

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        start_time = self.time.time()
        curr_time = start_time
        prev_time = curr_time
        while curr_time - start_time < 1:
            self.create.drive_direct(100, 100)
            state = self.create.update()
            if state is not None:
                print(state.__dict__)
                time_elapsed = curr_time - prev_time
                self.odometry.update(state.rightEncoderCounts, state.leftEncoderCounts, time_elapsed)
                print("[x = %f, y = %f, angle = %f]" % (self.odometry.x,
                                                        self.odometry.y,
                                                        np.rad2deg(self.odometry.angle)))
                print("time_elapsed: %f\nr_speed: %f, l_speed: %f\n" % (
                    time_elapsed, self.odometry.r_speed, self.odometry.l_speed
                ))

            prev_time = curr_time
            curr_time = self.time.time()
