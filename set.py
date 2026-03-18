import time
# Assuming Freenove's Servo.py is in your folder
from Servo import servo 

def center_sensor():
    pwm = Servo()
    print("Setting Servo to 90 degrees...")
    # Standard Freenove kits use Channel 0 for the Pan (horizontal) servo
    pwm.setServoPwm(0, 90) 
    time.sleep(2)
    print("Servo is now at center. If your sensor isn't pointing straight,")
    print("unscrew the plastic arm, straighten it, and put it back on.")

if __name__ == '__main__':
    center_sensor()