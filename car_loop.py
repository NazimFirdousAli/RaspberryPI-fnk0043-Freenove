import time
import sys
from car import Car
from car_client import CarClient
from shared.topics import SYSTEM_MODE

MANUAL = "manual"
AUTO = "auto"

SPEED = 1000
MAX_SPEED = 4000

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

def center_servos(self):
    self.pan  = 60
    self.tilt = 0
    self.car.servo.set_servo_pwm('0', self.pan)
    self.car.servo.set_servo_pwm('1', self.tilt)

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
        self.running = True
        self.current_mode = MANUAL
        self.previous_mode = None
        self.current_keys = []

        # Servo state
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER

        self.client = CarClient(
            car_id=car_id,
            broker_host=broker_host,
            on_message=self.handle_message
        )
        print(f"[{car_id}] Car loop started in {self.current_mode} mode...")

    def handle_message(self, topic: str, payload: dict):
        print(f"[message] {topic}: {payload}")  # add this line
        if topic == self.client.cmd_topic:
            self.current_keys = payload.get("keys", [])
        elif topic == self.client.servo_topic:
            self.pan  = payload.get("pan",  self.pan)
            self.tilt = payload.get("tilt", self.tilt)
        elif topic == SYSTEM_MODE:
            self.current_mode = payload.get("mode", MANUAL)

    def stop_motors(self):
        self.car.motor.set_motor_model(0, 0, 0, 0)

    def center_servos(self):
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER
        self.car.servo.set_servo_pwm('0', self.pan)
        self.car.servo.set_servo_pwm('1', self.tilt)

    def run(self):
        try:
            while self.running:

                if self.current_mode != self.previous_mode:
                    print(f"[mode] {self.previous_mode} → {self.current_mode}")
                    self.stop_motors()
                    self.previous_mode = self.current_mode

                if self.current_mode == MANUAL:

                    # Movement
                    FL, BL, FR, BR = compute_motor_vector(self.current_keys)
                    self.car.motor.set_motor_model(FL, BL, FR, BR)

                    # Servos
                    self.car.servo.set_servo_pwm('0', self.pan)
                    self.car.servo.set_servo_pwm('1', self.tilt)


                elif self.current_mode == AUTO:
                    pass  # later

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop_motors()
            self.center_servos()
            self.car.close()

if __name__ == "__main__":
    car_id = sys.argv[1] if len(sys.argv) > 1 else "leader"
    broker_host = sys.argv[2] if len(sys.argv) > 2 else "localhost"
    loop = CarLoop(car_id=car_id, broker_host=broker_host)
    loop.run()