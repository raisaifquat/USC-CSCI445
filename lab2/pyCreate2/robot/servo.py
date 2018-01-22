"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""
from .pwm import Pwm


class Servo:
    def __init__(self, number: int) -> None:
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self.pwm = Pwm(number)
        self.pwm.enable()

    def __del__(self) -> None:
        self.pwm.disable()

    def go_to(self, angle: float) -> None:
        # 0.75–2.25 ms high pulse, 20 ms intervals
        # 1 / (20 ms) = 1 / (0.02 s) = 50 Hz
        self.pwm.set_frequency(50)

        # clamp angle to [-90.0, 90.0]
        angle = -90.0 if angle < -90.0 else angle
        angle = 90.0 if angle > 90.0 else angle

        self.pwm.set_duty_cycle((angle + 90) / 180)
