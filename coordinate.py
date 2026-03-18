import time
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from motor import Ordinary_Car
from ultrasonic import Ultrasonic

class CalibratedNavigator:
    def __init__(self):
        self.car = Ordinary_Car()
        self.sensor = Ultrasonic()
        
        # --- CALIBRATION AREA ---
        # If the car travels TOO FAR: INCREASE this number.
        # If the car travels TOO SHORT: DECREASE this number.
        self.cm_per_second = 18.5  
        
        self.turn_90_time = 0.65   
        self.speed = 1000 # Keep speed constant for accuracy
        # ------------------------

        self.x, self.y = 0.0, 0.0
        self.angle = 90.0  # 90 = North/Forward
        self.points = ["A", "B", "C", "D", "E"]
        self.idx = 0

    def speak(self, text):
        print(f"[CAR] {text}")
        os.system(f'espeak "{text}" 2>/dev/null &')

    def move_distance(self, target_cm):
        # Calculate exactly how many seconds to run
        # Time = Distance / Speed
        duration = target_cm / self.cm_per_second
        
        self.speak(f"Target: {target_cm} cm. Driving for {duration:.2f} seconds.")
        
        start_time = time.time()
        self.car.set_motor_model(self.speed, self.speed, self.speed, self.speed)
        
        interrupted = False
        while (time.time() - start_time) < duration:
            # Safety check
            dist = self.sensor.get_distance()
            if dist is not None and 0 < dist < 20:
                self.car.set_motor_model(0,0,0,0)
                self.speak("Safety Stop!")
                interrupted = True
                break
            time.sleep(0.05)
            
        self.car.set_motor_model(0,0,0,0)
        
        # Update coordinates based on ACTUAL time spent moving
        actual_time = time.time() - start_time
        actual_dist = actual_time * self.cm_per_second
        
        rad = math.radians(self.angle)
        self.x += actual_dist * math.cos(rad)
        self.y += actual_dist * math.sin(rad)
        
        self.idx += 1
        to_pt = self.points[self.idx] if self.idx < len(self.points) else "?"
        self.speak(f"At Point {to_pt}. Pos: {int(self.x)}, {int(self.y)}")

    def run_mission(self):
        try:
            # Try a 50cm move now with the new constant
            self.move_distance(50)
        except KeyboardInterrupt:
            self.car.set_motor_model(0,0,0,0)

if __name__ == '__main__':
    nav = CalibratedNavigator()
    nav.run_mission()