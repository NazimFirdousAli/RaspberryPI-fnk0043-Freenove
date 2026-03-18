import cv2
import numpy as np
import time
import os
import sys

# Link to Freenove files
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from motor import Ordinary_Car

class VoiceCar:
    def __init__(self):
        self.car = Ordinary_Car()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

        # Sensitive Blue Range [Hue, Saturation, Value]
        self.blue_low = np.array([75, 40, 40]) 
        self.blue_high = np.array([145, 255, 255])

    def speak(self, text):
        """This sends the text to the Raspberry Pi speakers"""
        print(f"CAR SAYS: {text}")
        # The '&' at the end lets the code keep running while it talks
        os.system(f'espeak "{text}" &')

    def run(self):
        print("Voice System Active. Searching...")
        last_speech_time = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret: continue

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.blue_low, self.blue_high)
                blue_points = cv2.countNonZero(mask)

                # If we see the cup (adjust 1000 if it's too sensitive)
                if blue_points > 1000:
                    self.car.set_motor_model(0, 0, 0, 0) # Stop
                    
                    # Only speak once every 3 seconds so it doesn't stutter
                    if time.time() - last_speech_time > 3:
                        self.speak("I see a blue cup in front of me")
                        last_speech_time = time.time()
                else:
                    # Slow cruise
                    self.car.set_motor_model(800, 800, 800, 800)

                if cv2.waitKey(1) & 0xFF == ord('q'): break
        finally:
            self.car.set_motor_model(0,0,0,0)
            self.cap.release()

if __name__ == '__main__':
    VoiceCar().run()