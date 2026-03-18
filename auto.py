import cv2
import numpy as np
import time
import sys
import os

# 1. Setup paths for Freenove libraries
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from motor import Ordinary_Car
from ultrasonic import Ultrasonic

class SmartCar:
    def __init__(self):
        print("Initializing Hardware...")
        self.car = Ordinary_Car()
        self.sensor = Ultrasonic()
        
        # 2. START CAMERA AUTOMATICALLY
        print("Starting Camera Input...")
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        # Optimize settings so the car doesn't lag
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        if not self.cap.isOpened():
            print("ERROR: Camera failed to start. Check ribbon cable!")
        else:
            print("Camera is Online and Streaming.")

    def run(self):
        try:
            while True:
                # Grab a frame from the camera input
                ret, frame = self.cap.read()
                
                if ret:
                    # Logic: If we see the camera feed, we move!
                    # You can add your color detection here later
                    self.car.set_motor_model(800, 800, 800, 800)
                    
                    # Show the feed in VNC so you know it started
                    cv2.imshow('Car Live Feed', frame)
                
                # Safety check: Stop if something is too close
                dist = self.sensor.get_distance()
                if dist is not None and dist < 20:
                    print("Obstacle! Stopping.")
                    self.car.set_motor_model(0,0,0,0)
                    break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("Stopping Car...")
        finally:
            # CLEANUP: Always release the camera and stop motors
            self.car.set_motor_model(0,0,0,0)
            self.cap.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    # This starts the car AND the camera immediately
    my_car = SmartCar()
    my_car.run()