import time
import math
import random
from motor import Ordinary_Car
from ultrasonic import Ultrasonic
from odometry import Odometry

RETURN_SPEED       = 1000
TURN_SPEED         = 600
HOME_THRESHOLD     = 0.15
ANGLE_THRESHOLD    = 0.3
SAFE_DISTANCE      = 30.0

MAX_ATTEMPTS           = 3
BRAKE_DURATION         = 0.15
TURN_DURATION          = 0.7
CLEAR_DURATION         = 0.6
STABILIZE_DURATION     = 0.2
ESCAPE_TURN_DURATION   = 1.2
ESCAPE_DRIVE_DURATION  = 1.5

ROTATE_TO_HOME      = "rotate_to_home"
DRIVE_TO_HOME       = "drive_to_home"
AVOIDING_BRAKE      = "avoiding_brake"
AVOIDING_TURN       = "avoiding_turn"
AVOIDING_STABILIZE  = "avoiding_stabilize"
AVOIDING_CLEAR      = "avoiding_clear"
RANDOM_ESCAPE_TURN  = "random_escape_turn"
RANDOM_ESCAPE_DRIVE = "random_escape_drive"
ARRIVED             = "arrived"

class ReturnHome:
    def __init__(self, motor: Ordinary_Car, sonic: Ultrasonic, odometry: Odometry,
                 set_motors_fn=None, boundary_fn=None):
        self.motor          = motor
        self.sonic          = sonic
        self.odometry       = odometry
        self._set_motors_fn = set_motors_fn
        self._boundary_fn   = boundary_fn

        self.state       = ROTATE_TO_HOME
        self.state_start = time.time()
        self.attempts    = 0
        self.escape_dir  = 1

    def _time_in_state(self) -> float:
        return time.time() - self.state_start

    def _set_state(self, state: str):
        self.state = state
        self.state_start = time.time()
        print(f"[return_home] → {state}")

    def _get_distance(self) -> float:
        if self._boundary_fn and self._boundary_fn():
            return 10.0  # fake obstacle at boundary
        readings = []
        for _ in range(3):
            d = self.sonic.get_distance()
            if d is not None and d > 0:
                readings.append(d)
        return sum(readings) / len(readings) if readings else None

    def _distance_to_home(self) -> float:
        pos = self.odometry.get_position()
        return math.sqrt(pos["x"] ** 2 + pos["y"] ** 2)

    def _angle_error_to_home(self) -> float:
        pos = self.odometry.get_position()
        target_angle = math.atan2(-pos["y"], -pos["x"])
        current_heading = math.radians(pos["heading"])
        error = target_angle - current_heading
        return (error + math.pi) % (2 * math.pi) - math.pi

    def _set_motors(self, FL, BL, FR, BR):
        if self._set_motors_fn:
            self._set_motors_fn(FL, BL, FR, BR)
        else:
            self.motor.set_motor_model(FL, BL, FR, BR)
        self.odometry.update(FL, BL, FR, BR)

    def is_done(self) -> bool:
        return self.state == ARRIVED

    def update(self) -> bool:
        if self.state == ARRIVED:
            return True

        if self._distance_to_home() < HOME_THRESHOLD and self.state in (ROTATE_TO_HOME, DRIVE_TO_HOME):
            print("[return_home] Home reached!")
            self._set_motors(0, 0, 0, 0)
            self._set_state(ARRIVED)
            return True

        dist = self._get_distance()

        if self.state == ROTATE_TO_HOME:
            angle_error = self._angle_error_to_home()
            if abs(angle_error) < ANGLE_THRESHOLD:
                self._set_motors(0, 0, 0, 0)
                self._set_state(DRIVE_TO_HOME)
            elif angle_error > 0:
                self._set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
            else:
                self._set_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)

        elif self.state == DRIVE_TO_HOME:
            if dist is not None and dist < SAFE_DISTANCE:
                self._set_motors(-900, -900, -900, -900)
                self._set_state(AVOIDING_BRAKE)
            else:
                angle_error = self._angle_error_to_home()
                if abs(angle_error) > ANGLE_THRESHOLD:
                    self._set_motors(0, 0, 0, 0)
                    self._set_state(ROTATE_TO_HOME)
                else:
                    self._set_motors(RETURN_SPEED, RETURN_SPEED, RETURN_SPEED, RETURN_SPEED)

        elif self.state == AVOIDING_BRAKE:
            if self._time_in_state() >= BRAKE_DURATION:
                self._set_motors(0, 0, 0, 0)
                self._set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                self._set_state(AVOIDING_TURN)

        elif self.state == AVOIDING_TURN:
            if self._time_in_state() >= TURN_DURATION:
                self._set_motors(0, 0, 0, 0)
                self._set_state(AVOIDING_STABILIZE)

        elif self.state == AVOIDING_STABILIZE:
            if self._time_in_state() >= STABILIZE_DURATION:
                self._set_motors(RETURN_SPEED, RETURN_SPEED, RETURN_SPEED, RETURN_SPEED)
                self._set_state(AVOIDING_CLEAR)

        elif self.state == AVOIDING_CLEAR:
            if dist is not None and dist < SAFE_DISTANCE:
                self._set_motors(-900, -900, -900, -900)
                self._set_state(AVOIDING_BRAKE)
            elif self._time_in_state() >= CLEAR_DURATION:
                self.attempts += 1
                self._set_motors(0, 0, 0, 0)
                if self.attempts >= MAX_ATTEMPTS:
                    self.escape_dir = random.choice([1, -1])
                    if self.escape_dir == 1:
                        self._set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                    else:
                        self._set_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
                    self._set_state(RANDOM_ESCAPE_TURN)
                else:
                    self._set_state(ROTATE_TO_HOME)

        elif self.state == RANDOM_ESCAPE_TURN:
            if self._time_in_state() >= ESCAPE_TURN_DURATION:
                self._set_motors(RETURN_SPEED, RETURN_SPEED, RETURN_SPEED, RETURN_SPEED)
                self._set_state(RANDOM_ESCAPE_DRIVE)

        elif self.state == RANDOM_ESCAPE_DRIVE:
            if dist is not None and dist < SAFE_DISTANCE:
                self._set_motors(-900, -900, -900, -900)
                self._set_state(AVOIDING_BRAKE)
            elif self._time_in_state() >= ESCAPE_DRIVE_DURATION:
                self.attempts = 0
                self._set_motors(0, 0, 0, 0)
                self._set_state(ROTATE_TO_HOME)

        return False