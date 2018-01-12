"""
Example to move robot forward for 10 seconds
Use "python3 run.py [--sim] example1" to execute
"""
from typing import List


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

    def run(self):
        def average(nums: List[float]) -> float:
            return sum(nums) / len(nums)

        def move(
                right_wheel_velocity_in_m_per_sec: float = 1.0,
                left_wheel_velocity_in_m_per_sec: float = 1.0,
                duration: float = None,  # in seconds
                distance: float = 1.0  # in meters
        ) -> None:
            def get_time(distance_: float, speed_: float) -> float:
                return abs(distance_ / speed_)

            if duration is None:
                duration = get_time(
                    distance,
                    average([right_wheel_velocity_in_m_per_sec, left_wheel_velocity_in_m_per_sec])
                )

            self.create.drive_direct(right_wheel_velocity_in_m_per_sec * 1000,
                                     left_wheel_velocity_in_m_per_sec * 1000)
            self.time.sleep(duration)

        def stop() -> None:
            self.create.drive_direct(0, 0)

        def wait(duration: float = 1.0) -> None:
            stop()
            self.time.sleep(duration)

        def forward(distance: float = 1.0, speed: float = 1.0) -> None:  # speed in m/s
            move(speed, speed, None, distance)

        def backward(distance: float = 1.0, speed: float = 1.0) -> None:  # speed in m/s
            forward(distance, -speed)

        def turn_left(duration: float = 1.0, speed: float = 1.0) -> None:  # speed in m/s
            move(speed, -speed, duration, None)

        def turn_right(duration: float = 1.0, speed: float = 1.0) -> None:  # speed in m/s
            move(-speed, speed, duration, None)

        self.create.start()
        self.create.safe()

        # move forward
        forward(5 * 0.1, 0.1)

        # left turn (in place)
        turn_left(2, 0.1)

        # wait
        wait(2)

        # right turn (in place)
        turn_right(2, 0.1)

        # wait
        wait(2)

        # move forward
        forward(2 * 0.1, 0.1)

        # move forward while turning
        move(0.2, 0.1, 7.5)

        # move forward
        forward(5 * 0.1, 0.1)

        # move backwards
        backward(5 * 0.1, 0.1)

        # stop
        wait(3)

        self.create.stop()
