import time
import sys
import math
from car import Car
from car_client import CarClient
from shared.topics import SYSTEM_MODE
from buzzer import Buzzer
from odometry import Odometry
from go_to_position import GoToPosition
from motion import MotionController, RAMP_STEP

MANUAL         = "manual"
AUTO           = "auto"
RETURN_HOME    = "return_home"
GO_TO_POSITION = "go_to_position"

SPEED     = 1000
MAX_SPEED = 4000

BOUNDARY_MARGIN = 0.2
SHEET_WIDTH_M   = 3.0
SHEET_HEIGHT_M  = 1.5

HOME_X = 0.2
HOME_Y = SHEET_HEIGHT_M / 2

STUCK_TIME          = 2.0
STUCK_THRESHOLD     = 0.05
STUCK_TURN_DURATION = 0.8

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
        self.car    = Car()
        self.buzzer = Buzzer()
        self.running       = True
        self.current_mode  = MANUAL
        self.previous_mode = None
        self.current_keys  = []
        self.pending_clear = False

        self.odometry = Odometry()

        self.motion = MotionController(motor=self.car.motor)

        self.go_to_position = GoToPosition(
            motor=self.car.motor,
            sonic=self.car.sonic,
            odometry=self.odometry,
            set_motors_fn=self.motion.set_motors,
            boundary_fn=self._near_boundary_ahead
        )

        # Stuck detection state
        self.stuck_check_start   = time.time()
        self.stuck_check_pos     = (0.0, 0.0)
        self.stuck_recovering    = False
        self.stuck_recover_start = time.time()

        self.pan          = PAN_CENTER
        self.tilt         = TILT_CENTER
        self.buzzer_state = False

        self.car.servo.set_servo_pwm('0', PAN_CENTER)
        self.car.servo.set_servo_pwm('1', TILT_CENTER)

        self.auto_state       = AUTO_FORWARD
        self.auto_state_start = time.time()

        self.client = CarClient(
            car_id=car_id,
            broker_host=broker_host,
            on_message=self.handle_message
        )
        print(f"[{car_id}] Car loop started in {self.current_mode} mode...")

    # ------------------------------------------------------------------ #
    #  Message handling                                                    #
    # ------------------------------------------------------------------ #

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
            x           = payload.get("x", 0)
            y           = payload.get("y", 0)
            label       = payload.get("label", "")
            update_only = payload.get("update_only", False)

            if update_only:
                self.go_to_position.update_current_target(x, y)
            else:
                if label == "click":
                    self.pending_clear = True  # signal main loop to clear
                self.go_to_position.add_waypoint(x, y, label)
                self.current_mode = GO_TO_POSITION

        elif topic == SYSTEM_MODE:
            self.current_mode = payload.get("mode", MANUAL)

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def center_servos(self):
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER
        self.car.servo.set_servo_pwm('0', self.pan)
        self.car.servo.set_servo_pwm('1', self.tilt)

    def _time_in_state(self) -> float:
        return time.time() - self.auto_state_start

    def _set_auto_state(self, state: str):
        self.auto_state       = state
        self.auto_state_start = time.time()

    def _check_stuck(self) -> bool:
        if self.motion.current_FL == 0 and self.motion.current_BL == 0 and \
           self.motion.current_FR == 0 and self.motion.current_BR == 0:
            self.stuck_check_start = time.time()
            pos = self.odometry.get_position()
            self.stuck_check_pos = (pos["x"], pos["y"])
            return False

        if time.time() - self.stuck_check_start < STUCK_TIME:
            return False

        pos = self.odometry.get_position()
        dx = pos["x"] - self.stuck_check_pos[0]
        dy = pos["y"] - self.stuck_check_pos[1]

        if math.sqrt(dx**2 + dy**2) < STUCK_THRESHOLD:
            print(f"[stuck] Stuck at ({pos['x']:.3f}, {pos['y']:.3f}) — recovering")
            self.stuck_check_start = time.time()
            self.stuck_check_pos = (pos["x"], pos["y"])
            return True

        self.stuck_check_start = time.time()
        self.stuck_check_pos = (pos["x"], pos["y"])
        return False

    def _near_boundary_ahead(self) -> bool:
        pos = self.odometry.get_position()
        x, y = pos["x"], pos["y"]
        heading = math.radians(pos["heading"])
        look_x = x + math.cos(heading) * BOUNDARY_MARGIN
        look_y = y + math.sin(heading) * BOUNDARY_MARGIN
        return (
            look_x < 0 or look_x > SHEET_WIDTH_M or
            look_y < 0 or look_y > SHEET_HEIGHT_M
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

    # ------------------------------------------------------------------ #
    #  Main loop                                                           #
    # ------------------------------------------------------------------ #

    def run(self):
        try:
            while self.running:

                # Mode change detection
                if self.current_mode != self.previous_mode:
                    old_mode = self.previous_mode
                    print(f"[mode] {old_mode} → {self.current_mode}")
                    self.motion.hard_stop()
                    self.center_servos()
                    self.auto_state    = AUTO_FORWARD
                    self.previous_mode = self.current_mode
                    self.stuck_recovering = False
                    if self.current_mode == RETURN_HOME:
                        self.go_to_position.clear_waypoints()
                        self.go_to_position.add_waypoint(HOME_X, HOME_Y, "home")
                        self.current_mode = GO_TO_POSITION
                    elif old_mode == GO_TO_POSITION and self.current_mode != GO_TO_POSITION:
                        self.go_to_position.clear_waypoints()

                # Stuck detection
                if self.current_mode != MANUAL and not self.stuck_recovering:
                    if self._check_stuck():
                        self.stuck_recovering    = True
                        self.stuck_recover_start = time.time()
                        self.motion.hard_stop()
                        time.sleep(0.1)
                        self.motion.set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                
                if self.pending_clear:
                    self.go_to_position.clear_waypoints()
                    self.pending_clear = False

                # Stuck recovery
                if self.stuck_recovering:
                    if time.time() - self.stuck_recover_start >= STUCK_TURN_DURATION:
                        self.motion.hard_stop()
                        self.stuck_recovering = False
                        self.stuck_check_start = time.time()
                        pos = self.odometry.get_position()
                        self.stuck_check_pos = (pos["x"], pos["y"])
                        print("[stuck] Recovery complete, resuming")

                # Mode logic — skip if recovering
                elif self.current_mode == MANUAL:
                    FL, BL, FR, BR = compute_motor_vector(self.current_keys)
                    self.motion.set_motors(FL, BL, FR, BR)
                    self.car.servo.set_servo_pwm('0', self.pan)
                    self.car.servo.set_servo_pwm('1', self.tilt)
                    self.buzzer.set_state(self.buzzer_state)

                elif self.current_mode == AUTO:
                    dist = self._effective_distance()

                    if self.auto_state == AUTO_FORWARD:
                        self.motion.set_motors(SLOW_SPEED, SLOW_SPEED, SLOW_SPEED, SLOW_SPEED)
                        if dist is not None and dist < SAFE_DISTANCE:
                            print(f"Obstacle at {dist:.1f}cm!")
                            self.motion.set_motors(-900, -900, -900, -900)
                            self._set_auto_state(AUTO_BRAKING)

                    elif self.auto_state == AUTO_BRAKING:
                        if self._time_in_state() >= BRAKE_DURATION:
                            self.motion.hard_stop()
                            self.motion.set_motors(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                            self._set_auto_state(AUTO_TURNING)

                    elif self.auto_state == AUTO_TURNING:
                        if self._time_in_state() >= TURN_DURATION:
                            self.motion.hard_stop()
                            self._set_auto_state(AUTO_STABILIZE)

                    elif self.auto_state == AUTO_STABILIZE:
                        if self._time_in_state() >= STABILIZE_DURATION:
                            self._set_auto_state(AUTO_FORWARD)

                elif self.current_mode == GO_TO_POSITION:
                    done = self.go_to_position.update()
                    if done:
                        self.current_mode = MANUAL
                        print("[car_loop] All waypoints reached, switching to manual")

                # Update odometry
                self.odometry.update(
                    self.motion.current_FL,
                    self.motion.current_BL,
                    self.motion.current_FR,
                    self.motion.current_BR
                )

                # Publish state
                pos = self.odometry.get_position()
                self.client.publish_state(
                    speed=self.motion.current_FL,
                    heading=pos["heading"],
                    mode=self.current_mode,
                    x=pos["x"],
                    y=pos["y"]
                )

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.motion.hard_stop()
            self.center_servos()
            self.buzzer.set_state(False)
            self.buzzer.close()
            self.car.close()


if __name__ == "__main__":
    car_id      = sys.argv[1] if len(sys.argv) > 1 else "leader"
    broker_host = sys.argv[2] if len(sys.argv) > 2 else "localhost"
    loop = CarLoop(car_id=car_id, broker_host=broker_host)
    loop.run()