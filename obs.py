import time
import sys
import os

# Ensure local imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from motor import Ordinary_Car
from ultrasonic import Ultrasonic

class SmartCar:
    def __init__(self):
        self.car = Ordinary_Car()
        self.sensor = Ultrasonic()
        
        # --- TUNING PARAMETERS ---
        self.safe_distance = 40.0   # Detection range in cm
        self.slow_speed = 700       # Very slow for precision (Range 0-4095)
        self.turn_speed = 1400      # Enough power to pivot on carpet
        # -------------------------

    def get_accurate_dist(self):
        """Takes 3 readings and returns the median to ignore 'ghost' errors"""
        readings = []
        for _ in range(3):
            d = self.sensor.get_distance()
            if d is not None and d > 0:
                readings.append(d)
            time.sleep(0.005) # Tiny gap between pings
        
        if len(readings) > 0:
            return sum(readings) / len(readings)
        return None

    def brake(self):
        """Briefly reverses motors to cancel forward momentum"""
        self.car.set_motor_model(-900, -900, -900, -900)
        time.sleep(0.15)
        self.car.set_motor_model(0, 0, 0, 0)

    def move_forward(self):
        self.car.set_motor_model(self.slow_speed, self.slow_speed, self.slow_speed, self.slow_speed)

    def turn_left(self, detected_dist):
        print(f"Obstacle at {detected_dist:.1f}cm! Braking...")
        self.brake()
        time.sleep(0.3)
        
        # Spin Left: Left side -, Right side +
        # Mecanum wheels spin in place better with a bit more power
        self.car.set_motor_model(-self.turn_speed, -self.turn_speed, self.turn_speed, self.turn_speed)
        time.sleep(0.7) 
        self.car.set_motor_model(0, 0, 0, 0)
        time.sleep(0.2) # Stabilize after turn

    def run(self):
        print("Running in ULTRA-SLOW accurate mode...")
        try:
            while True:
                # Use the new averaging function
                dist = self.get_accurate_dist()
                
                if dist is not None:
                    # Print distance to console to monitor accuracy
                    print(f"Current Distance: {dist:.1f} cm", end='\r')
                    
                    if dist < self.safe_distance:
                        self.turn_left(dist)
                    else:
                        self.move_forward()
                
                time.sleep(0.01) # High frequency check

        except KeyboardInterrupt:
            print("\nShutting down safely...")
            self.car.set_motor_model(0, 0, 0, 0)
            self.sensor.close()

if __name__ == '__main__':
    logic = SmartCar()
    logic.run()