import time
import sys
import math
from car import Car
from car_client import CarClient
from shared.topics import SYSTEM_MODE
from buzzer import Buzzer
from odometry import Odometry
from return_home import ReturnHome
from return_home_trace import ReturnHomeTrace, PathLogger
from go_to_position import GoToPosition

MANUAL = "manual"
AUTO = "auto"
RETURN_HOME = "return_home"
RETURN_HOME_TRACE = "return_home_trace"
GO_TO_POSITION = "go_to_position"

SPEED = 1000
MAX_SPEED = 4000

BOUNDARY_MARGIN = 0.2
SHEET_WIDTH_M   = 3.0
SHEET_HEIGHT_M  = 1.5
STUCK_TIME      = 2.0   # seconds before declaring stuck
STUCK_THRESHOLD = 0.05  # meters — less than this means stuck
STUCK_TURN_DURATION = 0.8  # seconds to turn when stuck

LEFT_SCALE  = 1.2
RIGHT_SCALE = 1.0

SAFE_DISTANCE      = 40.0
SLOW_SPEED         = 700
TURN_SPEED         = 1400
BRAKE_DURATION     = 0.15
TURN_DURATION      = 0.7
STABILIZE_DURATION = 0.2

AUTO_FORWARD   = "forward"
AUTO_BRAKING   = "braking"
AUTO_TURNING   = "turning"
AUTO_STABILIZE = "stabilize"

PAN_CENTER  = 60
TILT_CENTER = 0
PAN_MIN     = 0
PAN_MAX     = 120
TILT_MIN    = 0
TILT_MAX    = 180
SERVO_STEP  = 5

KEY_VECTORS = {
    "w": ( SPEED,  SPEED,  SPEED,  SPEED),
    "s": (-SPEED, -SPEED, -SPEED, -SPEED),
    "a": (-SPEED,  SPEED,  SPEED, -SPEED),
    "d": ( SPEED, -SPEED, -SPEED,  SPEED),
    "q": (-SPEED, -SPEED,  SPEED,  SPEED),
    "e": ( SPEED,  SPEED, -SPEED, -SPEED),
}

def compute_motor_vector(keys: list) -> tuple:
    FL, BL, FR, BR = 0, 0, 0, 0
    for key in keys:
        if key in KEY_VECTORS:
            v = KEY_VECTORS[key]
            FL += v[0]
            BL += v[1]
            FR += v[2]
            BR += v[3]
    clamp = lambda val: max(-MAX_SPEED, min(MAX_SPEED, val))
    return clamp(FL), clamp(BL), clamp(FR), clamp(BR)

class CarLoop:
    def __init__(self, car_id: str, broker_host: str):
        self.car = Car()
        self.buzzer = Buzzer()
        self.running = True
        self.current_mode = MANUAL
        self.previous_mode = None
        self.return_home = None
        self.current_keys = []

        self.path_logger = PathLogger()
        self.return_home_trace = None

        # Stuck detection state
        self.stuck_check_start  = time.time()
        self.stuck_check_pos    = (0.0, 0.0)
        self.stuck_recovering   = False
        self.stuck_recover_start = time.time()

        self.odometry = Odometry()
        self.current_FL = 0
        self.current_BL = 0
        self.current_FR = 0
        self.current_BR = 0

        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER
        self.buzzer_state = False

        self.car.servo.set_servo_pwm('0', PAN_CENTER)
        self.car.servo.set_servo_pwm('1', TILT_CENTER)

        self.go_to_position = GoToPosition(
            motor=self.car.motor,
            sonic=self.car.sonic,
            odometry=self.odometry,
            set_motors_fn=self.set_motors,
            boundary_fn=self._near_boundary_ahead
        )

        self.auto_state = AUTO_FORWARD
        self.auto_state_start = time.time()

        self.client = CarClient(
            car_id=car_id,
            broker_host=broker_host,
            on_message=self.handle_message
        )
        print(f"[{car_id}] Car loop started in {self.current_mode} mode...")

    def handle_message(self, topic: str, payload: dict):
        if topic == self.client.cmd_topic:
            keys = payload.get("keys", [])
            if keys and self.current_mode != MANUAL:
                print("[car_loop] Manual input — interrupting auto mode")
                self.current_mode = MANUAL
                self.go_to_position.clear_waypoints()
            self.current_keys = keys
        elif topic == self.client.servo_topic:
            self.pan  = payload.get("pan",  self.pan)
            self.tilt = payload.get("tilt", self.tilt)
        elif topic == self.client.buzzer_topic:
            self.buzzer_state = payload.get("state", False)
        elif topic == self.client.position_topic:
            x       = payload.get("x", self.odometry.x)
            y       = payload.get("y", self.odometry.y)
            heading = payload.get("heading", math.degrees(self.odometry.heading))
            self.odometry.x       = x
            self.odometry.y       = y
            self.odometry.heading = math.radians(heading)
        elif topic == self.client.waypoint_topic:
            x     = payload.get("x", 0)
            y     = payload.get("y", 0)
            label = payload.get("label", "")
            self.go_to_position.add_waypoint(x, y, label)
            self.current_mode = GO_TO_POSITION
        elif topic == SYSTEM_MODE:
            self.current_mode = payload.get("mode", MANUAL)

    def set_motors(self, FL: int, BL: int, FR: int, BR: int):
        clamp = lambda val: max(-MAX_SPEED, min(MAX_SPEED, val))
        self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR
        FL_corrected = clamp(int(FL * LEFT_SCALE))
        BL_corrected = clamp(int(BL * LEFT_SCALE))
        FR_corrected = clamp(int(FR * RIGHT_SCALE))
        BR_corrected = clamp(int(BR * RIGHT_SCALE))
        self.car.motor.set_motor_model(FL_corrected, BL_corrected, FR_corrected, BR_corrected)

    def stop_motors(self):
        self.set_motors(0, 0, 0, 0)

    def center_servos(self):
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER
        self.car.servo.set_servo_pwm('0', self.pan)
        self.car.servo.set_servo_pwm('1', self.tilt)

    def _time_in_state(self) -> float:
        return time.time() - self.auto_state_start
    
    def _check_stuck(self) -> bool:
        """Returns True if car is stuck and triggers recovery."""
        # Only check if motors are running
        if self.current_FL == 0 and self.current_BL == 0 and \
        self.current_FR == 0 and self.current_BR == 0:
            # Reset timer when stopped
            self.stuck_check_start = time.time()
            pos = self.odometry.get_position()
            self.stuck_check_pos = (pos["x"], pos["y"])
            return False

        # Check if enough time has passed
        if time.time() - self.stuck_check_start < STUCK_TIME:
            return False

        # Check if position has changed
        pos = self.odometry.get_position()
        dx = pos["x"] - self.stuck_check_pos[0]
        dy = pos["y"] - self.stuck_check_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        if dist < STUCK_THRESHOLD:
            print(f"[stuck] Car appears stuck at ({pos['x']:.3f}, {pos['y']:.3f}) — recovering")
            # Reset timer and position for next check
            self.stuck_check_start = time.time()
            self.stuck_check_pos = (pos["x"], pos["y"])
            return True

        # Made progress — reset
        self.stuck_check_start = time.time()
        self.stuck_check_pos = (pos["x"], pos["y"])
        return False

    def _set_auto_state(self, state: str):
        self.auto_state = state
        self.auto_state_start = time.time()

    def _near_boundary_ahead(self) -> bool:
        pos = self.odometry.get_position()
        x, y = pos["x"], pos["y"]
        heading = math.radians(pos["heading"])
        look_x = x + math.cos(heading) * BOUNDARY_MARGIN
        look_y = y + math.sin(heading) * BOUNDARY_MARGIN
        return (
            look_x < 0 or
            look_x > SHEET_WIDTH_M or
            look_y < 0 or
            look_y > SHEET_HEIGHT_M
        )

    def _near_boundary(self) -> bool:
        pos = self.odometry.get_position()
        x, y = pos["x"], pos["y"]
        return (
            x < -BOUNDARY_MARGIN or
            x > SHEET_WIDTH_M + BOUNDARY_MARGIN or
            y < -BOUNDARY_MARGIN or
            y > SHEET_HEIGHT_M + BOUNDARY_MARGIN
        )

    def get_accurate_distance(self) -> float:
        readings = []
        for _ in range(3):
            d = self.car.sonic.get_distance()
            if d is not None and d > 0:
                readings.append(d)
        return sum(readings) / len(readings) if readings else None

    def _effective_distance(self) -> float:
        dist = self.get_accurate_distance()
        if self._near_boundary_ahead():
            return 10.0
        return dist

    def run(self):
        try:
            while self.running:

                # Mode change detection
                if self.current_mode != self.previous_mode:
                    old_mode = self.previous_mode
                    print(f"[mode] {old_mode} → {self.current_mode}")
                    self.stop_motors()
                    self.center_servos()
                    self.auto_state = AUTO_FORWARD
                    self.previous_mode = self.current_mode
                    self.return_home = None
                    self.return_home_trace = None
                    self.stuck_recovering = False
                    if old_mode == GO_TO_POSITION and self.current_mode != GO_TO_POSITION:
                        self.go_to_position.clear_waypoints()

                # Stuck detection — only in autonomous modes
                if self.current_mode != MANUAL and not self.stuck_recovering:
                    if self._check_stuck():
                        self.stuck_recovering = True
                        self.stuck_recover_start = time.time()
                        self.set_motors(0, 0, 0, 0)
                        time.sleep(0.1)
                        self.set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)

                # Handle stuck recovery
                if self.stuck_recovering:
                    if time.time() - self.stuck_recover_start >= STUCK_TURN_DURATION:
                        self.set_motors(0, 0, 0, 0)
                        self.stuck_recovering = False
                        self.stuck_check_start = time.time()
                        pos = self.odometry.get_position()
                        self.stuck_check_pos = (pos["x"], pos["y"])
                        print("[stuck] Recovery complete, resuming")

                # Mode logic — skip if recovering
                elif self.current_mode == MANUAL:
                    FL, BL, FR, BR = compute_motor_vector(self.current_keys)
                    self.set_motors(FL, BL, FR, BR)
                    self.path_logger.record(FL, BL, FR, BR)
                    self.car.servo.set_servo_pwm('0', self.pan)
                    self.car.servo.set_servo_pwm('1', self.tilt)
                    self.buzzer.set_state(self.buzzer_state)

                elif self.current_mode == AUTO:
                    dist = self._effective_distance()

                    if self.auto_state == AUTO_FORWARD:
                        self.set_motors(SLOW_SPEED, SLOW_SPEED, SLOW_SPEED, SLOW_SPEED)
                        self.path_logger.record(SLOW_SPEED, SLOW_SPEED, SLOW_SPEED, SLOW_SPEED)
                        if dist is not None and dist < SAFE_DISTANCE:
                            print(f"Obstacle at {dist:.1f}cm!")
                            self.set_motors(-900, -900, -900, -900)
                            self.path_logger.record(-900, -900, -900, -900)
                            self._set_auto_state(AUTO_BRAKING)

                    elif self.auto_state == AUTO_BRAKING:
                        if self._time_in_state() >= BRAKE_DURATION:
                            self.set_motors(0, 0, 0, 0)
                            self.set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                            self.path_logger.record(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                            self._set_auto_state(AUTO_TURNING)

                    elif self.auto_state == AUTO_TURNING:
                        if self._time_in_state() >= TURN_DURATION:
                            self.set_motors(0, 0, 0, 0)
                            self._set_auto_state(AUTO_STABILIZE)

                    elif self.auto_state == AUTO_STABILIZE:
                        if self._time_in_state() >= STABILIZE_DURATION:
                            self._set_auto_state(AUTO_FORWARD)

                elif self.current_mode == GO_TO_POSITION:
                    done = self.go_to_position.update()
                    if done:
                        self.current_mode = MANUAL
                        print("[car_loop] All waypoints reached, switching to manual")

                elif self.current_mode == RETURN_HOME:
                    if self.return_home is None:
                        self.return_home = ReturnHome(
                            motor=self.car.motor,
                            sonic=self.car.sonic,
                            odometry=self.odometry,
                            set_motors_fn=self.set_motors,
                            boundary_fn=self._near_boundary_ahead
                        )
                    done = self.return_home.update()
                    if done:
                        self.current_mode = MANUAL
                        self.return_home = None
                        print("[car_loop] Arrived home, switching to manual")

                elif self.current_mode == RETURN_HOME_TRACE:
                    if self.return_home_trace is None:
                        self.return_home_trace = ReturnHomeTrace(
                            motor=self.car.motor,
                            sonic=self.car.sonic,
                            path_logger=self.path_logger
                        )
                    done = self.return_home_trace.update()
                    if done:
                        self.current_mode = MANUAL
                        self.return_home_trace = None
                        self.path_logger.reset()
                        print("[car_loop] Trace complete, switching to manual")

                # Update odometry every iteration
                self.odometry.update(
                    self.current_FL,
                    self.current_BL,
                    self.current_FR,
                    self.current_BR
                )

                # Publish state every iteration
                pos = self.odometry.get_position()
                self.client.publish_state(
                    speed=self.current_FL,
                    heading=pos["heading"],
                    mode=self.current_mode,
                    x=pos["x"],
                    y=pos["y"]
                )

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop_motors()
            self.center_servos()
            self.buzzer.set_state(False)
            self.buzzer.close()
            self.car.close()

if __name__ == "__main__":
    car_id = sys.argv[1] if len(sys.argv) > 1 else "leader"
    broker_host = sys.argv[2] if len(sys.argv) > 2 else "localhost"
    loop = CarLoop(car_id=car_id, broker_host=broker_host)
    loop.run()