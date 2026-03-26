import sys
import os
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import json
import time
import paho.mqtt.client as mqtt
from shared.topics import LEADER_STATE, FOLLOWER_STATE, LEADER_POSITION, FOLLOWER_POSITION
from shared.payloads import make_state

# --- CONFIGURE THESE ---
SHEET_WIDTH_M  = 3
SHEET_HEIGHT_M = 1.5
CAMERA_INDEX   = 0     # webcam index
PUBLISH_RATE   = 0.05  # publish every 50ms (20hz)

# HSV color ranges for arrow detection
# Tune these using color_tuner.py
LEADER_COLOR_HSV = {
    "lower": np.array([0, 150, 0]),
    "upper": np.array([15, 255, 255])
}
LEADER_COLOR_HSV2 = {
    "lower": np.array([170, 150, 0]),
    "upper": np.array([180, 255, 255])
}
FOLLOWER_COLOR_HSV = {
    "lower": np.array([40, 70, 70]),    # green arrow
    "upper": np.array([80, 255, 255])
}

# Sheet corner points in pixel space — set during calibration
# Order: top-left, top-right, bottom-right, bottom-left
SHEET_CORNERS_PX = None  # set by calibrate_sheet()

def calibrate_sheet(frame):
    """
    Let user click the 4 corners of the sheet to set up
    the pixel-to-meter transform.
    """
    corners = []
    clone = frame.copy()

    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
            corners.append((x, y))
            cv2.circle(clone, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Calibrate — click 4 corners (TL, TR, BR, BL)", clone)

    cv2.imshow("Calibrate — click 4 corners (TL, TR, BR, BL)", clone)
    cv2.setMouseCallback("Calibrate — click 4 corners (TL, TR, BR, BL)", click)

    print("Click the 4 corners of the sheet: Top-Left, Top-Right, Bottom-Right, Bottom-Left")
    while len(corners) < 4:
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    return np.array(corners, dtype=np.float32)

def get_transform(corners_px):
    """Get perspective transform from pixel space to sheet space."""
    dst = np.array([
        [0, 0],
        [SHEET_WIDTH_M, 0],
        [SHEET_WIDTH_M, SHEET_HEIGHT_M],
        [0, SHEET_HEIGHT_M]
    ], dtype=np.float32)
    M = cv2.getPerspectiveTransform(corners_px, dst)
    return M

def pixel_to_meters(px, py, M):
    """Convert pixel coordinates to meters using perspective transform."""
    pt = np.array([[[px, py]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, M)
    return float(result[0][0][0]), float(result[0][0][1])

def detect_arrow(frame, hsv, color_range, color_range2=None):
    mask = cv2.inRange(hsv, color_range["lower"], color_range["upper"])
    if color_range2:
        mask2 = cv2.inRange(hsv, color_range2["lower"], color_range2["upper"])
        mask = cv2.bitwise_or(mask, mask2)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 500:
        return None

    # Centroid
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    # Find the tip — the point farthest from the centroid
    centroid = np.array([cx, cy])
    points = c.reshape(-1, 2)
    distances = np.linalg.norm(points - centroid, axis=1)
    tip_idx = np.argmax(distances)
    tip = points[tip_idx]

    # Heading = angle from centroid to tip
    dx = tip[0] - cx
    dy = tip[1] - cy
    angle = math.degrees(math.atan2(-dy, dx))  # negative dy because y is flipped in image
    if angle < 0:
        angle += 360
    angle = (angle + 180) % 360  # flip direction

    return cx, cy, angle

def draw_debug(frame, cx, cy, angle, label, color, text_color=(255, 255, 255)):
    """Draw detection on frame for debugging."""
    cv2.circle(frame, (cx, cy), 8, color, -1)
    
    # Draw heading arrow in bright cyan
    length = 60
    rad = np.radians(angle)
    ex = int(cx + length * np.cos(rad))
    ey = int(cy - length * np.sin(rad))
    cv2.arrowedLine(frame, (cx, cy), (ex, ey), (255, 255, 0), 3)  # cyan arrow

    # Draw text with black background
    text = f"{label} {angle:.0f}°"
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
    cv2.rectangle(frame, (cx + 10, cy - th - 4), (cx + 10 + tw, cy + 4), (0, 0, 0), -1)
    cv2.putText(frame, text, (cx + 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)

class PositionTracker:
    def __init__(self, broker_host: str):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.connect(broker_host, 1883)
        self.client.loop_start()
        self.transform = None
        self.last_publish = time.time()

    def publish_position(self, car_id: str, x: float, y: float, heading: float):
        topic = LEADER_POSITION if car_id == "leader" else FOLLOWER_POSITION
        payload = {"x": x, "y": y, "heading": heading, "timestamp": time.time()}
        self.client.publish(topic, json.dumps(payload), qos=0)

    def run(self, camera_index: int = CAMERA_INDEX):
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        print("Camera opened. Calibrating sheet corners...")
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return

        corners = calibrate_sheet(frame)
        self.transform = get_transform(corners)
        print("Calibration complete! Starting tracking...")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            debug = frame.copy()

            # Detect leader
            leader = detect_arrow(frame, hsv, LEADER_COLOR_HSV, LEADER_COLOR_HSV2)
            if leader:
                cx, cy, angle = leader
                x, y = pixel_to_meters(cx, cy, self.transform)
                draw_debug(debug, cx, cy, angle, "Leader", (0, 0, 255))
                if time.time() - self.last_publish > PUBLISH_RATE:
                    self.publish_position("leader", x, y, angle)

            # Detect follower
            follower = detect_arrow(frame, hsv, FOLLOWER_COLOR_HSV)
            if follower:
                cx, cy, angle = follower
                x, y = pixel_to_meters(cx, cy, self.transform)
                draw_debug(debug, cx, cy, angle, "Follower", (0, 255, 0))
                if time.time() - self.last_publish > PUBLISH_RATE:
                    self.publish_position("follower", x, y, angle)
                    self.last_publish = time.time()

            cv2.imshow("Position Tracker", debug)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        self.client.loop_stop()

if __name__ == "__main__":
    broker_host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    tracker = PositionTracker(broker_host=broker_host)
    tracker.run()