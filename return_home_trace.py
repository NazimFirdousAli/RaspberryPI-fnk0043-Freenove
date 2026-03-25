import time
from motor import Ordinary_Car
from ultrasonic import Ultrasonic

SAFE_DISTANCE         = 30.0
TURN_SPEED            = 1400

# How long a 180° rotation takes at TURN_SPEED
# Replace this after running calibrate.py rotation test
ROTATION_180_DURATION = 1.73 / 2  # = 0.865

# States
ROTATING_180 = "rotating_180"
REPLAYING    = "replaying"
WAITING      = "waiting"
ARRIVED      = "arrived"

class PathLogger:
    def __init__(self):
        self.log = []
        self._last_time = time.time()

    def record(self, FL: int, BL: int, FR: int, BR: int):
        now = time.time()
        dt  = now - self._last_time
        self._last_time = now

        if FL == 0 and BL == 0 and FR == 0 and BR == 0:
            return

        if (self.log and
            self.log[-1]["FL"] == FL and
            self.log[-1]["BL"] == BL and
            self.log[-1]["FR"] == FR and
            self.log[-1]["BR"] == BR):
            self.log[-1]["dt"] += dt
        else:
            self.log.append({
                "FL": FL, "BL": BL,
                "FR": FR, "BR": BR,
                "dt": dt
            })

    def reset(self):
        self.log = []
        self._last_time = time.time()

    def _is_rotation(self, entry: dict) -> bool:
        """Returns True if this entry is primarily a rotation command."""
        FL, BL, FR, BR = entry["FL"], entry["BL"], entry["FR"], entry["BR"]
        # In a rotation, left wheels and right wheels spin in opposite directions
        left_avg  = (FL + BL) / 2
        right_avg = (FR + BR) / 2
        # Check if left and right sides have opposite signs and are significant
        if abs(left_avg) < 100 or abs(right_avg) < 100:
            return False
        return (left_avg > 0) != (right_avg > 0)

    def get_log(self) -> list:
        """
        Returns log in reverse order.
        Rotation commands are negated, movement commands are kept as-is.
        After the 180° rotation, this correctly retraces the original path.
        """
        result = []
        for entry in reversed(self.log):
            if self._is_rotation(entry):
                result.append({
                    "FL": -entry["FL"],
                    "BL": -entry["BL"],
                    "FR": -entry["FR"],
                    "BR": -entry["BR"],
                    "dt": entry["dt"]
                })
            else:
                result.append(dict(entry))
        return result


class ReturnHomeTrace:
    def __init__(self, motor: Ordinary_Car, sonic: Ultrasonic, path_logger: PathLogger):
        self.motor      = motor
        self.sonic      = sonic
        self.replay_log = path_logger.get_log()
        self.index      = 0
        self.state      = ROTATING_180
        self.step_start = time.time()

        if not self.replay_log:
            print("[trace] No path to replay.")
            self.state = ARRIVED
        else:
            print(f"[trace] Rotating 180° before replay of {len(self.replay_log)} steps.")
            self.motor.set_motor_model(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
            self.step_start = time.time()

    def _start_step(self):
        if self.index >= len(self.replay_log):
            print("[trace] Home reached!")
            self.motor.set_motor_model(0, 0, 0, 0)
            self.state = ARRIVED
            return

        step = self.replay_log[self.index]
        self.motor.set_motor_model(step["FL"], step["BL"], step["FR"], step["BR"])
        self.step_start = time.time()
        self.state = REPLAYING
        print(f"[trace] Step {self.index + 1}/{len(self.replay_log)}")

    def _get_distance(self) -> float:
        readings = []
        for _ in range(3):
            d = self.sonic.get_distance()
            if d is not None and d > 0:
                readings.append(d)
        return sum(readings) / len(readings) if readings else None

    def is_done(self) -> bool:
        return self.state == ARRIVED

    def update(self) -> bool:
        """Call every loop iteration. Returns True when home is reached."""
        if self.state == ARRIVED:
            return True

        dist = self._get_distance()

        if self.state == ROTATING_180:
            if time.time() - self.step_start >= ROTATION_180_DURATION:
                self.motor.set_motor_model(0, 0, 0, 0)
                print("[trace] Rotation complete, starting replay.")
                self._start_step()

        elif self.state == REPLAYING:
            if dist is not None and dist < SAFE_DISTANCE:
                print(f"[trace] Obstacle at {dist:.1f}cm — waiting...")
                self.motor.set_motor_model(0, 0, 0, 0)
                self.state = WAITING
                return False

            elapsed = time.time() - self.step_start
            if elapsed >= self.replay_log[self.index]["dt"]:
                self.index += 1
                self._start_step()

        elif self.state == WAITING:
            if dist is None or dist >= SAFE_DISTANCE:
                print("[trace] Obstacle cleared — resuming.")
                step = self.replay_log[self.index]
                self.motor.set_motor_model(step["FL"], step["BL"], step["FR"], step["BR"])
                self.step_start = time.time()
                self.state = REPLAYING

        return False