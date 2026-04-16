import time
import math
import random
from motor import Ordinary_Car
from ultrasonic import Ultrasonic
from odometry import Odometry

# Tuning
MAX_SPEED          = 4000
RETURN_SPEED       = 1000
TURN_SPEED         = 900
ARRIVAL_THRESHOLD  = 0.15
ANGLE_THRESHOLD    = 0.3
SAFE_DISTANCE      = 30.0
WAYPOINT_PAUSE     = 1.0

BRAKE_DURATION        = 0.15
TURN_DURATION         = 0.7
CLEAR_DURATION        = 0.6
STABILIZE_DURATION    = 0.2
ESCAPE_TURN_DURATION  = 1.2
ESCAPE_DRIVE_DURATION = 1.5
MAX_ATTEMPTS          = 3

SLOWDOWN_DISTANCE = 0.5
MIN_DRIVE_SPEED   = 400

# PID gains — tune these
PID_KP = 800.0   # proportional — main turning force
PID_KD = 200.0   # derivative — dampens overshoot
PID_KI = 0.0     # integral — leave at 0 until P and D are tuned

# States
ROTATE_TO_TARGET    = "rotate_to_target"
DRIVE_TO_TARGET     = "drive_to_target"
PAUSING             = "pausing"
AVOIDING_BRAKE      = "avoiding_brake"
AVOIDING_TURN       = "avoiding_turn"
AVOIDING_STABILIZE  = "avoiding_stabilize"
AVOIDING_CLEAR      = "avoiding_clear"
RANDOM_ESCAPE_TURN  = "random_escape_turn"
RANDOM_ESCAPE_DRIVE = "random_escape_drive"
DONE                = "done"


class PID:
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral    = 0.0
        self._last_error  = 0.0
        self._last_time   = time.time()

    def reset(self):
        self._integral   = 0.0
        self._last_error = 0.0
        self._last_time  = time.time()

    def compute(self, error: float) -> float:
        now = time.time()
        dt  = now - self._last_time
        if dt <= 0:
            dt = 0.01

        self._integral += error * dt
        derivative = (error - self._last_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        self._last_error = error
        self._last_time  = now
        return output


class GoToPosition:
    def __init__(self, motor: Ordinary_Car, sonic: Ultrasonic, odometry: Odometry,
                 set_motors_fn=None, boundary_fn=None):
        self.motor          = motor
        self.sonic          = sonic
        self.odometry       = odometry
        self._set_motors_fn = set_motors_fn
        self._boundary_fn   = boundary_fn

        self.waypoints      = []
        self.current_target = None
        self.state          = DONE
        self.state_start    = time.time()
        self.attempts       = 0
        self.escape_dir     = 1

        self.heading_pid = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD, output_limit=TURN_SPEED)

    def add_waypoint(self, x: float, y: float, label: str = ""):
        self.waypoints.append((x, y, label))
        print(f"[goto] Waypoint added: ({x:.3f}, {y:.3f}) '{label}'")
        if self.state == DONE:
            self._next_waypoint()

    def update_current_target(self, x: float, y: float):
        """Update current target in place for live tracking."""
        if self.current_target:
            label = self.current_target[2]
            self.current_target = (x, y, label)

    def clear_waypoints(self):
        self.waypoints = []
        self.current_target = None
        self._set_motors(0, 0, 0, 0)
        self.state = DONE
        self.heading_pid.reset()
        print("[goto] Waypoints cleared")

    def _next_waypoint(self):
        if self.waypoints:
            self.current_target = self.waypoints.pop(0)
            x, y, label = self.current_target
            print(f"[goto] Moving to ({x:.3f}, {y:.3f}) '{label}'")
            self.attempts = 0
            self._set_state(ROTATE_TO_TARGET)
        else:
            print("[goto] All waypoints reached")
            self._set_motors(0, 0, 0, 0)
            self.state = DONE

    def _set_state(self, state: str):
        self.state       = state
        self.state_start = time.time()
        self.heading_pid.reset()  # reset PID on every state transition
        print(f"[goto] → {state}")

    def _time_in_state(self) -> float:
        return time.time() - self.state_start

    def _get_distance(self) -> float:
        if self._boundary_fn and self._boundary_fn():
            return 10.0
        readings = []
        for _ in range(3):
            d = self.sonic.get_distance()
            if d is not None and d > 0:
                readings.append(d)
        return sum(readings) / len(readings) if readings else None

    def _distance_to_target(self) -> float:
        if not self.current_target:
            return 0
        tx, ty, _ = self.current_target
        pos = self.odometry.get_position()
        return math.sqrt((pos["x"] - tx) ** 2 + (pos["y"] - ty) ** 2)

    def _angle_error_to_target(self) -> float:
        if not self.current_target:
            return 0
        tx, ty, _ = self.current_target
        pos = self.odometry.get_position()
        dx = tx - pos["x"]
        dy = ty - pos["y"]
        target_angle = math.atan2(-dy, dx)
        current_heading = math.radians(pos["heading"])
        error = target_angle - current_heading
        return (error + math.pi) % (2 * math.pi) - math.pi

    def _set_motors(self, FL, BL, FR, BR):
        if self._set_motors_fn:
            self._set_motors_fn(FL, BL, FR, BR)
        else:
            self.motor.set_motor_model(FL, BL, FR, BR)

    def is_done(self) -> bool:
        return self.state == DONE and not self.waypoints

    def update(self) -> bool:
        if self.state == DONE:
            return True

        if not self.current_target:
            return True

        if self._distance_to_target() < ARRIVAL_THRESHOLD and self.state in (ROTATE_TO_TARGET, DRIVE_TO_TARGET):
            x, y, label = self.current_target
            print(f"[goto] Arrived at ({x:.3f}, {y:.3f}) '{label}' — pausing {WAYPOINT_PAUSE}s")
            self._set_motors(0, 0, 0, 0)
            self._set_state(PAUSING)
            return False

        dist = self._get_distance()

        if self.state == ROTATE_TO_TARGET:
            angle_error = self._angle_error_to_target()
            if abs(angle_error) < ANGLE_THRESHOLD:
                self._set_motors(0, 0, 0, 0)
                self._set_state(DRIVE_TO_TARGET)
            else:
                turn = int(self.heading_pid.compute(angle_error))
                self._set_motors(-turn, -turn, turn, turn)

        elif self.state == DRIVE_TO_TARGET:
            if dist is not None and dist < SAFE_DISTANCE:
                self._set_motors(-900, -900, -900, -900)
                self._set_state(AVOIDING_BRAKE)
            else:
                angle_error = self._angle_error_to_target()
                dist_to_target = self._distance_to_target()
                scale = min(1.0, dist_to_target / SLOWDOWN_DISTANCE)
                drive_speed = max(MIN_DRIVE_SPEED, int(RETURN_SPEED * scale))

                # PID heading correction — steer while driving
                correction = int(self.heading_pid.compute(angle_error))

                # Clamp individual wheel speeds to valid range
                clamp = lambda val: max(-MAX_SPEED, min(MAX_SPEED, val))
                self._set_motors(
                    clamp(drive_speed - correction),
                    clamp(drive_speed - correction),
                    clamp(drive_speed + correction),
                    clamp(drive_speed + correction)
                )

        elif self.state == PAUSING:
            if self._time_in_state() >= WAYPOINT_PAUSE:
                self._next_waypoint()

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
                    self._set_state(ROTATE_TO_TARGET)

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
                self._set_state(ROTATE_TO_TARGET)

        return False