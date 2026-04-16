import math

# Tuning constants
RAMP_STEP       = 50
MIN_MOTOR_SPEED = 350
MAX_SPEED       = 4000
LEFT_SCALE      = 1.2
RIGHT_SCALE     = 1.0


class MotionController:
    def __init__(self, motor):
        self.motor = motor

        self.smooth_FL = 0
        self.smooth_BL = 0
        self.smooth_FR = 0
        self.smooth_BR = 0

        self.current_FL = 0
        self.current_BL = 0
        self.current_FR = 0
        self.current_BR = 0

    def set_motors(self, FL: int, BL: int, FR: int, BR: int):
        clamp = lambda val: max(-MAX_SPEED, min(MAX_SPEED, val))

        self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR

        self.smooth_FL = self._ramp(self.smooth_FL, FL)
        self.smooth_BL = self._ramp(self.smooth_BL, BL)
        self.smooth_FR = self._ramp(self.smooth_FR, FR)
        self.smooth_BR = self._ramp(self.smooth_BR, BR)

        self.motor.set_motor_model(
            clamp(int(self.smooth_FL * LEFT_SCALE)),
            clamp(int(self.smooth_BL * LEFT_SCALE)),
            clamp(int(self.smooth_FR * RIGHT_SCALE)),
            clamp(int(self.smooth_BR * RIGHT_SCALE))
        )

    def hard_stop(self):
        """Instant stop — bypasses ramp."""
        self.smooth_FL = self.smooth_BL = self.smooth_FR = self.smooth_BR = 0
        self.current_FL = self.current_BL = self.current_FR = self.current_BR = 0
        self.motor.set_motor_model(0, 0, 0, 0)

    def stop_motors(self):
        """Gradual stop via ramp."""
        self.set_motors(0, 0, 0, 0)

    def _ramp(self, current: int, target: int) -> int:
        if current == target:
            return target

        new = min(current + RAMP_STEP, target) if target > current else max(current - RAMP_STEP, target)

        if target == 0:
            return new
        elif target > 0 and 0 < new < MIN_MOTOR_SPEED:
            return MIN_MOTOR_SPEED
        elif target < 0 and -MIN_MOTOR_SPEED < new < 0:
            return -MIN_MOTOR_SPEED

        return new