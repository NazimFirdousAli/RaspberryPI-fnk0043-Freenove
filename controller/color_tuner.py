import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow("Tuner")
cv2.createTrackbar("H Low",  "Tuner", 0,   180, nothing)
cv2.createTrackbar("H High", "Tuner", 180, 180, nothing)
cv2.createTrackbar("S Low",  "Tuner", 0,   255, nothing)
cv2.createTrackbar("S High", "Tuner", 255, 255, nothing)
cv2.createTrackbar("V Low",  "Tuner", 0,   255, nothing)
cv2.createTrackbar("V High", "Tuner", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hl = cv2.getTrackbarPos("H Low",  "Tuner")
    hh = cv2.getTrackbarPos("H High", "Tuner")
    sl = cv2.getTrackbarPos("S Low",  "Tuner")
    sh = cv2.getTrackbarPos("S High", "Tuner")
    vl = cv2.getTrackbarPos("V Low",  "Tuner")
    vh = cv2.getTrackbarPos("V High", "Tuner")

    lower = np.array([hl, sl, vl])
    upper = np.array([hh, sh, vh])
    mask  = cv2.inRange(hsv, lower, upper)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    print(f"HSV Lower: [{hl}, {sl}, {vl}]  Upper: [{hh}, {sh}, {vh}]", end='\r')

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()