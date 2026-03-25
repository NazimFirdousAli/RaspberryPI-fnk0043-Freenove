import time
import sys
from car import Car
from car_client import CarClient
from shared.topics import SYSTEM_MODE
from buzzer import Buzzer
from odometry import Odometry
from return_home import ReturnHome
from return_home_trace import ReturnHomeTrace, PathLogger

MANUAL = "manual"
AUTO = "auto"
RETURN_HOME = "return_home"
RETURN_HOME_TRACE = "return_home_trace"

SPEED = 1500
MAX_SPEED = 4000

# Auto mode settings
SAFE_DISTANCE      = 40.0
SLOW_SPEED         = 700
TURN_SPEED         = 1400
BRAKE_DURATION     = 0.15
TURN_DURATION      = 0.7
STABILIZE_DURATION = 0.2

# Auto mode states
AUTO_FORWARD   = "forward"
AUTO_BRAKING   = "braking"
AUTO_TURNING   = "turning"
AUTO_STABILIZE = "stabilize"

# Servo settings
PAN_CENTER  = 60
TILT_CENTER = 0
PAN_MIN     = 0
PAN_MAX     = 120
TILT_MIN    = 0
TILT_MAX    = 180
SERVO_STEP  = 5

# Each key maps to (FL, BL, FR, BR)
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

        # Odometry state
        self.odometry = Odometry()
        self.current_FL = 0
        self.current_BL = 0
        self.current_FR = 0
        self.current_BR = 0

        # Servo state
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER

        # Buzzer state
        self.buzzer_state = False

        # Initialize servos to center
        self.car.servo.set_servo_pwm('0', PAN_CENTER)
        self.car.servo.set_servo_pwm('1', TILT_CENTER)

        # Auto mode state
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
            self.current_keys = payload.get("keys", [])
        elif topic == self.client.servo_topic:
            self.pan  = payload.get("pan",  self.pan)
            self.tilt = payload.get("tilt", self.tilt)
        elif topic == self.client.buzzer_topic:
            self.buzzer_state = payload.get("state", False)
        elif topic == SYSTEM_MODE:
            self.current_mode = payload.get("mode", MANUAL)

    def stop_motors(self):
        self.car.motor.set_motor_model(0, 0, 0, 0)
        self.current_FL = self.current_BL = self.current_FR = self.current_BR = 0

    def center_servos(self):
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER
        self.car.servo.set_servo_pwm('0', self.pan)
        self.car.servo.set_servo_pwm('1', self.tilt)

    def _time_in_state(self) -> float:
        return time.time() - self.auto_state_start

    def _set_auto_state(self, state: str):
        self.auto_state = state
        self.auto_state_start = time.time()

    def get_accurate_distance(self) -> float:
        readings = []
        for _ in range(3):
            d = self.car.sonic.get_distance()
            if d is not None and d > 0:
                readings.append(d)
        return sum(readings) / len(readings) if readings else None

    def run(self):
        try:
            while self.running:

                # Detect mode change and stop motors for clean transition
                if self.current_mode != self.previous_mode:
                    print(f"[mode] {self.previous_mode} → {self.current_mode}")
                    self.stop_motors()
                    self.center_servos()
                    self.auto_state = AUTO_FORWARD
                    self.previous_mode = self.current_mode
                    self.return_home = None
                    self.return_home_trace = None

                if self.current_mode == MANUAL:

                    # Movement
                    FL, BL, FR, BR = compute_motor_vector(self.current_keys)
                    self.car.motor.set_motor_model(FL, BL, FR, BR)
                    self.path_logger.record(FL, BL, FR, BR)
                    self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR

                    # Servos
                    self.car.servo.set_servo_pwm('0', self.pan)
                    self.car.servo.set_servo_pwm('1', self.tilt)

                    # Buzzer
                    self.buzzer.set_state(self.buzzer_state)

                elif self.current_mode == AUTO:
                    dist = self.get_accurate_distance()

                    if self.auto_state == AUTO_FORWARD:
                        FL, BL, FR, BR = SLOW_SPEED, SLOW_SPEED, SLOW_SPEED, SLOW_SPEED
                        self.car.motor.set_motor_model(FL, BL, FR, BR)
                        self.path_logger.record(FL, BL, FR, BR)
                        self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR
                        if dist is not None and dist < SAFE_DISTANCE:
                            print(f"Obstacle at {dist:.1f}cm!")
                            FL, BL, FR, BR = -900, -900, -900, -900
                            self.car.motor.set_motor_model(FL, BL, FR, BR)
                            self.path_logger.record(FL, BL, FR, BR)
                            self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR
                            self._set_auto_state(AUTO_BRAKING)

                    elif self.auto_state == AUTO_BRAKING:
                        if self._time_in_state() >= BRAKE_DURATION:
                            FL, BL, FR, BR = 0, 0, 0, 0
                            self.car.motor.set_motor_model(FL, BL, FR, BR)
                            self.path_logger.record(FL, BL, FR, BR)
                            self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR
                            FL, BL, FR, BR = -TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED
                            self.car.motor.set_motor_model(FL, BL, FR, BR)
                            self.path_logger.record(FL, BL, FR, BR)
                            self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR
                            self._set_auto_state(AUTO_TURNING)

                    elif self.auto_state == AUTO_TURNING:
                        if self._time_in_state() >= TURN_DURATION:
                            FL, BL, FR, BR = 0, 0, 0, 0
                            self.car.motor.set_motor_model(FL, BL, FR, BR)
                            self.path_logger.record(FL, BL, FR, BR)
                            self.current_FL, self.current_BL, self.current_FR, self.current_BR = FL, BL, FR, BR
                            self._set_auto_state(AUTO_STABILIZE)

                    elif self.auto_state == AUTO_STABILIZE:
                        if self._time_in_state() >= STABILIZE_DURATION:
                            self._set_auto_state(AUTO_FORWARD)

                elif self.current_mode == RETURN_HOME:
                    if self.return_home is None:
                        self.return_home = ReturnHome(
                            motor=self.car.motor,
                            sonic=self.car.sonic,
                            odometry=self.odometry
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