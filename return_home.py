import time
import math
import random
from motor import Ordinary_Car
from ultrasonic import Ultrasonic
from odometry import Odometry

# Tuning parameters
RETURN_SPEED    = 1000   # slower than normal for precision
TURN_SPEED      = 1400
HOME_THRESHOLD  = 0.15   # meters — stop when this close to home
ANGLE_THRESHOLD = 0.15   # radians — close enough to facing home
SAFE_DISTANCE   = 30.0   # cm

MAX_ATTEMPTS    = 3      # failed obstacle avoidances before random escape
ESCAPE_DURATION = 1.5    # seconds to drive blindly during random escape

BRAKE_DURATION    = 0.15
TURN_DURATION     = 0.7
CLEAR_DURATION    = 0.6  # how long to drive forward after turning to clear obstacle
STABILIZE_DURATION = 0.2

# States
ROTATE_TO_HOME  = "rotate_to_home"
DRIVE_TO_HOME   = "drive_to_home"
AVOIDING_BRAKE  = "avoiding_brake"
AVOIDING_TURN   = "avoiding_turn"
AVOIDING_CLEAR  = "avoiding_clear"
AVOIDING_STABILIZE = "avoiding_stabilize"
RANDOM_ESCAPE   = "random_escape"
ARRIVED         = "arrived"

class ReturnHome:
    def __init__(self, motor: Ordinary_Car, sonic: Ultrasonic, odometry: Odometry):
        self.motor    = motor
        self.sonic    = sonic
        self.odometry = odometry

        self.state      = ROTATE_TO_HOME
        self.state_start = time.time()
        self.attempts   = 0
        self.escape_angle = 0.0

    def _time_in_state(self) -> float:
        return time.time() - self.state_start

    def _set_state(self, state: str):
        self.state = state
        self.state_start = time.time()
        print(f"[return_home] → {state}")

    def _get_distance(self) -> float:
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
        # Normalize to [-π, π]
        return (error + math.pi) % (2 * math.pi) - math.pi

    def _set_motors(self, FL, BL, FR, BR):
        self.motor.set_motor_model(FL, BL, FR, BR)
        # Update odometry
        self.odometry.update(FL, BL, FR, BR)

    def is_done(self) -> bool:
        return self.state == ARRIVED

    def update(self):
        """Call this every loop iteration. Returns True when home is reached."""

        if self.state == ARRIVED:
            return True

        # Check distance to home first
        if self._distance_to_home() < HOME_THRESHOLD and self.state in (ROTATE_TO_HOME, DRIVE_TO_HOME):
            print(f"[return_home] Home reached!")
            self._set_motors(0, 0, 0, 0)
            self._set_state(ARRIVED)
            return True

        dist = self._get_distance()

        if self.state == ROTATE_TO_HOME:
            angle_error = self._angle_error_to_home()
            if abs(angle_error) < ANGLE_THRESHOLD:
                # Facing home — start driving
                self._set_state(DRIVE_TO_HOME)
            else:
                # Rotate toward home
                if angle_error > 0:
                    self._set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                else:
                    self._set_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)

        elif self.state == DRIVE_TO_HOME:
            if dist is not None and dist < SAFE_DISTANCE:
                # Obstacle detected
                self._set_motors(-900, -900, -900, -900)
                self._set_state(AVOIDING_BRAKE)
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
            # Drive forward to clear obstacle, still checking for new obstacles
            if dist is not None and dist < SAFE_DISTANCE:
                # Hit another obstacle while clearing — treat as new attempt
                self._set_motors(-900, -900, -900, -900)
                self._set_state(AVOIDING_BRAKE)
            elif self._time_in_state() >= CLEAR_DURATION:
                self.attempts += 1
                print(f"[return_home] Obstacle cleared (attempt {self.attempts}/{MAX_ATTEMPTS})")
                if self.attempts >= MAX_ATTEMPTS:
                    # Too many attempts — random escape
                    self.escape_angle = random.uniform(math.pi / 2, math.pi)  # 90–180 degrees
                    if random.random() > 0.5:
                        self.escape_angle *= -1  # randomize direction too
                    print(f"[return_home] Max attempts reached, random escape: {math.degrees(self.escape_angle):.1f}°")
                    self._set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                    self._set_state(RANDOM_ESCAPE)
                else:
                    self._set_state(ROTATE_TO_HOME)

        elif self.state == RANDOM_ESCAPE:
            # Rotate for a duration proportional to escape angle then drive forward
            turn_time = abs(self.escape_angle) / (2 * math.pi) * (TURN_DURATION * 4)
            if self._time_in_state() >= turn_time:
                self.attempts = 0
                self._set_motors(RETURN_SPEED, RETURN_SPEED, RETURN_SPEED, RETURN_SPEED)
                time.sleep(ESCAPE_DURATION)
                self._set_motors(0, 0, 0, 0)
                self._set_state(ROTATE_TO_HOME)

        return False