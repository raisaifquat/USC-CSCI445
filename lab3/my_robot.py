from typing import List, Callable
import sys


class MyRobot:
    def __init__(
            self,
            base_speed: float = None,  # base_speed in m/s
            create=None,
            time=None
    ):
        self.base_speed = 0.1 if base_speed is None else base_speed
        # self.create = create
        # self.time = time
        self.drive_direct = create.drive_direct
        self.update = create.update
        self.sleep = time.sleep

    def move(
            self,
            right_wheel_velocity_in_m_per_sec: float = None,
            left_wheel_velocity_in_m_per_sec: float = None,
            duration: float = None,  # in seconds
            distance: float = 1.0,  # in meters,
            is_print: bool = False
    ) -> None:
        def get_time(distance_: float, speed_: float) -> float:
            return abs(distance_ / speed_)

        if right_wheel_velocity_in_m_per_sec is None:
            right_wheel_velocity_in_m_per_sec = self.base_speed

        if left_wheel_velocity_in_m_per_sec is None:
            left_wheel_velocity_in_m_per_sec = self.base_speed

        if duration is None:
            duration = get_time(
                distance,
                average([right_wheel_velocity_in_m_per_sec, left_wheel_velocity_in_m_per_sec])
            )

        self.drive_direct(int(right_wheel_velocity_in_m_per_sec * 1000),
                          int(left_wheel_velocity_in_m_per_sec * 1000))
        # self.print_state()
        self.sleep(duration)
        self.print_state(is_print)

    def stop(self) -> None:
        self.drive_direct(0, 0)

    def wait(self, duration: float = 1.0, is_print: bool = False) -> None:
        self.stop()
        self.sleep(duration)
        self.print_state(is_print)

    def forward(self, distance: float = 1.0, speed: float = None, is_print: bool = False) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.move(speed, speed, distance=distance, is_print=is_print)

    def backward(self, distance: float = 1.0, speed: float = None, is_print: bool = False) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.forward(distance, -speed, is_print=is_print)

    def turn_left(self, duration: float = 1.0, speed: float = None, is_print: bool = False) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.move(speed, -speed, duration, is_print=is_print)

    def turn_right(self, duration: float = 1.0, speed: float = None, is_print: bool = False) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.move(-speed, speed, duration, is_print=is_print)

    def print_state(self, is_print: bool = False):
        state = self.update()
        if state is None and not is_print:
            return
        print(state.__dict__)


def average(nums: List[float]) -> float:
    return sum(nums) / len(nums)
