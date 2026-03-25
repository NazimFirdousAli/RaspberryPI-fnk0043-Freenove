import time
import sys
from car import Car
from car_client import CarClient
from shared.topics import SYSTEM_MODE
from buzzer import Buzzer

MANUAL = "manual"
AUTO = "auto"

SPEED = 1500
MAX_SPEED = 4000

# Auto mode settings
SAFE_DISTANCE     = 40.0
SLOW_SPEED        = 700
TURN_SPEED        = 1400
BRAKE_DURATION    = 0.15
TURN_DURATION     = 0.7
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
        self.current_keys = []

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
                    self.auto_state = AUTO_FORWARD  # reset auto state on mode switch
                    self.previous_mode = self.current_mode

                if self.current_mode == MANUAL:

                    # Movement
                    FL, BL, FR, BR = compute_motor_vector(self.current_keys)
                    self.car.motor.set_motor_model(FL, BL, FR, BR)

                    # Servos
                    self.car.servo.set_servo_pwm('0', self.pan)
                    self.car.servo.set_servo_pwm('1', self.tilt)

                    # Buzzer
                    self.buzzer.set_state(self.buzzer_state)

                elif self.current_mode == AUTO:
                    dist = self.get_accurate_distance()

                    if self.auto_state == AUTO_FORWARD:
                        self.car.motor.set_motor_model(SLOW_SPEED, SLOW_SPEED, SLOW_SPEED, SLOW_SPEED)
                        if dist is not None and dist < SAFE_DISTANCE:
                            print(f"Obstacle at {dist:.1f}cm!")
                            self.car.motor.set_motor_model(-900, -900, -900, -900)
                            self._set_auto_state(AUTO_BRAKING)

                    elif self.auto_state == AUTO_BRAKING:
                        if self._time_in_state() >= BRAKE_DURATION:
                            self.car.motor.set_motor_model(0, 0, 0, 0)
                            self.car.motor.set_motor_model(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED)
                            self._set_auto_state(AUTO_TURNING)

                    elif self.auto_state == AUTO_TURNING:
                        if self._time_in_state() >= TURN_DURATION:
                            self.car.motor.set_motor_model(0, 0, 0, 0)
                            self._set_auto_state(AUTO_STABILIZE)

                    elif self.auto_state == AUTO_STABILIZE:
                        if self._time_in_state() >= STABILIZE_DURATION:
                            self._set_auto_state(AUTO_FORWARD)

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