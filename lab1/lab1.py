"""
Example to move robot forward for 10 seconds
Use "python3 run.py [--sim] example1" to execute
"""


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

    def run(self):
        def move(
                right_wheel_velocity_in_mm_per_sec: float = 100.0,
                left_wheel_velocity_in_mm_per_sec: float = 100.0,
                num_seconds: float = 10.0
        ) -> None:
            self.create.drive_direct(right_wheel_velocity_in_mm_per_sec,
                                     left_wheel_velocity_in_mm_per_sec)
            self.time.sleep(num_seconds)

        def wait(num_seconds: float = 10.0) -> None:
            move(0, 0, num_seconds)

        def move_forward(velocity_in_mm_per_sec: float = 100.0, num_seconds: float = 10.0) -> None:
            move(velocity_in_mm_per_sec, velocity_in_mm_per_sec, num_seconds)

        def move_backward(velocity_in_mm_per_sec: float = 100.0, num_seconds: float = 10.0) -> None:
            move_forward(-velocity_in_mm_per_sec, num_seconds)

        self.create.start()
        self.create.safe()

        # move forward
        move_forward(100, 5)

        # left turn (in place)
        move(100, -100, 2)

        # wait
        wait(2)

        # right turn (in place)
        move(-100, 100, 2)

        # wait
        wait(2)

        # move forward
        move_forward(100, 2)

        # move forward while turning
        move(200, 100, 7.5)

        # move forward
        move_forward(100, 5)

        # move backwards
        move_backward(100, 5)

        # stop
        wait(3)

        self.create.stop()
