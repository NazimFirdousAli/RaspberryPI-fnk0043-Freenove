import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# ---------------- MOTOR SETUP ----------------
IN1, IN2, IN3, IN4 = 17, 18, 22, 23
ENA, ENB = 27, 24

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)

pwmA.start(60)
pwmB.start(60)

# ---------------- MOVEMENTS ----------------
def forward():
    GPIO.output(IN1, True); GPIO.output(IN2, False)
    GPIO.output(IN3, True); GPIO.output(IN4, False)

def left():
    GPIO.output(IN1, False); GPIO.output(IN2, True)
    GPIO.output(IN3, True); GPIO.output(IN4, False)

def right():
    GPIO.output(IN1, True); GPIO.output(IN2, False)
    GPIO.output(IN3, False); GPIO.output(IN4, True)

def stop():
    GPIO.output(IN1, False); GPIO.output(IN2, False)
    GPIO.output(IN3, False); GPIO.output(IN4, False)

# ---------------- DISTANCE FUNCTION ----------------
# Approximation using object width
KNOWN_WIDTH = 7.0   # cm (cup approx width)
FOCAL_LENGTH = 500  # adjust after calibration

def calculate_distance(perceived_width):
    if perceived_width == 0:
        return 999
    return (KNOWN_WIDTH * FOCAL_LENGTH) / perceived_width

# ---------------- CAMERA ----------------
cap = cv2.VideoCapture(0)

# Color ranges (HSV)
colors = {
    "Red": [
        (np.array([0,120,70]), np.array([10,255,255])),
        (np.array([170,120,70]), np.array([180,255,255]))
    ],
    "Blue": [
        (np.array([100,150,0]), np.array([140,255,255]))
    ],
    "Green": [
        (np.array([40,70,70]), np.array([80,255,255]))
    ]
}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640,480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    best_object = None
    max_area = 0

    # -------- DETECT MULTIPLE COLORS --------
    for color_name, ranges in colors.items():
        mask = None

        for lower, upper in ranges:
            temp_mask = cv2.inRange(hsv, lower, upper)
            mask = temp_mask if mask is None else mask + temp_mask

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 1200 and area > max_area:
                x, y, w, h = cv2.boundingRect(cnt)
                best_object = (x, y, w, h, color_name)
                max_area = area

    # -------- FOLLOW BEST OBJECT --------
    if best_object:
        x, y, w, h, color_name = best_object
        cx = x + w // 2

        distance = calculate_distance(w)

        # Draw box
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(frame, f"{color_name} {int(distance)}cm",
                    (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        print(color_name, "Distance:", int(distance), "cm")

        # -------- SMART MOVEMENT --------
        if distance < 15:
            print("STOP (Reached)")
            stop()

        else:
            if cx < 200:
                print("LEFT")
                left()
            elif cx > 440:
                print("RIGHT")
                right()
            else:
                print("FORWARD")
                forward()
    else:
        print("NO OBJECT")
        stop()

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) == 27:
        break

# Cleanup
cap.release()
GPIO.cleanup()
cv2.destroyAllWindows()