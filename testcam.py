import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# GPIO setup
buzzer = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer, GPIO.OUT)

cap = cv2.VideoCapture(0)

def beep():
    GPIO.output(buzzer, True)
    time.sleep(0.5)
    GPIO.output(buzzer, False)

def move_forward():
    print("Car Moving Forward")

def stop_car():
    print("Car Stopped")

def avoid_obstacle():
    print("Obstacle Detected - Turning")

def check_blue(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100,150,0])
    upper_blue = np.array([140,255,255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > 500:
            return True

    return False


while True:

    ret, frame = cap.read()

    # --- Simulated obstacle detection ---
    obstacle = False   # Replace with ultrasonic sensor reading

    if obstacle:
        avoid_obstacle()

    else:
        move_forward()

    if check_blue(frame):
        stop_car()
        beep()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()