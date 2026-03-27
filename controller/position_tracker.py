import sys
import os
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import json
import time
import paho.mqtt.client as mqtt
from shared.topics import LEADER_POSITION, LEADER_WAYPOINT, CUP_POSITIONS
from shared.payloads import make_waypoint

# --- CONFIGURE THESE ---
SHEET_WIDTH_M  = 3
SHEET_HEIGHT_M = 1.5
CAMERA_INDEX   = 0
PUBLISH_RATE   = 0.033  # 30hz

# Arrow HSV
LEADER_COLOR_HSV = {
    "lower": np.array([0, 150, 0]),
    "upper": np.array([15, 255, 255])
}
LEADER_COLOR_HSV2 = {
    "lower": np.array([170, 150, 0]),
    "upper": np.array([180, 255, 255])
}

# Cup HSV ranges — tune with color_tuner.py tomorrow
CUP_COLORS = {
    "light_blue": {
        "hsv":  {"lower": np.array([85,  80,  80]),  "upper": np.array([105, 255, 255])},
        "hsv2": None,
        "bgr":  (230, 216, 173),  # display color
    },
    "purple": {
        "hsv":  {"lower": np.array([125, 80,  80]),  "upper": np.array([155, 255, 255])},
        "hsv2": None,
        "bgr":  (211, 0, 147),
    },
    "green": {
        "hsv":  {"lower": np.array([40,  70,  70]),  "upper": np.array([80,  255, 255])},
        "hsv2": None,
        "bgr":  (0, 200, 0),
    },
}

SHEET_CORNERS_PX = None

def calibrate_sheet(frame):
    corners = []
    clone = frame.copy()
    corner_names = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]

    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
            corners.append((x, y))
            cv2.circle(clone, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(clone, corner_names[len(corners)-1], (x + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(win_name, clone)
            if len(corners) < 4:
                print(f"✓ {corner_names[len(corners)-1]} set. Now click: {corner_names[len(corners)]}")
            else:
                print("✓ All 4 corners set!")

    win_name = "Calibrate Sheet"
    cv2.imshow(win_name, clone)
    cv2.setMouseCallback(win_name, click)
    print(f"\nClick the 4 corners of the tarp in order.")
    print(f"Start with: {corner_names[0]}")

    while len(corners) < 4:
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    return np.array(corners, dtype=np.float32)

def get_transform(corners_px):
    dst = np.array([
        [0, 0],
        [SHEET_WIDTH_M, 0],
        [SHEET_WIDTH_M, SHEET_HEIGHT_M],
        [0, SHEET_HEIGHT_M]
    ], dtype=np.float32)
    return cv2.getPerspectiveTransform(corners_px, dst)

def pixel_to_meters(px, py, M):
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

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    centroid = np.array([cx, cy])
    points = c.reshape(-1, 2)
    distances = np.linalg.norm(points - centroid, axis=1)
    tip_idx = np.argmax(distances)
    tip = points[tip_idx]

    dx = tip[0] - cx
    dy = tip[1] - cy
    angle = math.degrees(math.atan2(-dy, dx))
    if angle < 0:
        angle += 360
    angle = (angle + 180) % 360

    return cx, cy, angle

def detect_blob(hsv, color_range, color_range2=None, min_area=300):
    """Detect a colored blob — returns (cx, cy) in pixels or None."""
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
    if cv2.contourArea(c) < min_area:
        return None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy

def draw_debug(frame, cx, cy, angle, label, color):
    cv2.circle(frame, (cx, cy), 8, color, -1)
    length = 60
    rad = np.radians(angle)
    ex = int(cx + length * np.cos(rad))
    ey = int(cy - length * np.sin(rad))
    cv2.arrowedLine(frame, (cx, cy), (ex, ey), (255, 255, 0), 3)
    text = f"{label} {angle:.0f}°"
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
    cv2.rectangle(frame, (cx + 10, cy - th - 4), (cx + 10 + tw, cy + 4), (0, 0, 0), -1)
    cv2.putText(frame, text, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def draw_cup(frame, cx, cy, label, bgr):
    cv2.circle(frame, (cx, cy), 14, bgr, -1)
    cv2.circle(frame, (cx, cy), 14, (255, 255, 255), 2)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    cv2.rectangle(frame, (cx - tw//2 - 2, cy + 16), (cx + tw//2 + 2, cy + 16 + th + 4), (0, 0, 0), -1)
    cv2.putText(frame, label, (cx - tw//2, cy + 16 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

class PositionTracker:
    def __init__(self, broker_host: str):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect  = self._on_connect
        self.client.on_message  = self._on_message
        self.client.connect(broker_host, 1883)
        self.client.loop_start()
        self.transform    = None
        self.last_publish = time.time()
        # Latest known cup positions {label: (x, y)}
        self.cup_positions = {}
        self.cup_warning = ""
        self.cup_warning_time = 0
        self.tracking_cup = None

    def _on_connect(self, client, userdata, flags, rc, props):
        # Subscribe to waypoint requests so we can fill in cup coordinates
        self.client.subscribe(LEADER_WAYPOINT, qos=1)

    def _on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode())
            label = payload.get("label", "")

            # Stop tracking on regular click
            if label == "click":
                self.tracking_cup = None
                return

            # Only intercept cup color requests (x=0, y=0)
            if label in CUP_COLORS:
                if payload.get("x", 0) == 0 and payload.get("y", 0) == 0:
                    if label in self.cup_positions:
                        x, y = self.cup_positions[label]
                        real_payload = make_waypoint(x, y, label)
                        self.client.publish(LEADER_WAYPOINT, json.dumps(real_payload), qos=1)
                        self.tracking_cup = label  # start live tracking
                        print(f"[tracker] Tracking cup: {label} → ({x:.3f}, {y:.3f})")
                    else:
                        print(f"[tracker] ⚠ Cup '{label}' not detected — waypoint ignored")
                        self.cup_warning = f"WARNING: {label} cup not detected!"
                        self.cup_warning_time = time.time()

        except Exception as e:
            print(f"[tracker] Waypoint resolve error: {e}")


    def publish_position(self, x: float, y: float, heading: float):
        payload = {"x": x, "y": y, "heading": heading, "timestamp": time.time()}
        self.client.publish(LEADER_POSITION, json.dumps(payload), qos=0)

    def publish_cups(self):
        payload = {
            label: {"x": x, "y": y}
            for label, (x, y) in self.cup_positions.items()
        }
        payload["timestamp"] = time.time()
        self.client.publish(CUP_POSITIONS, json.dumps(payload), qos=0)

    def run(self, camera_index: int = CAMERA_INDEX):
        global SHEET_CORNERS_PX

        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        print("Camera opened. Calibrating sheet corners...")
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return

        SHEET_CORNERS_PX = calibrate_sheet(frame)
        self.transform = get_transform(SHEET_CORNERS_PX)
        print("Calibration complete! Starting tracking...")

        cv2.namedWindow("Position Tracker")

        def on_click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                if self.transform is not None:
                    wx, wy = pixel_to_meters(x, y, self.transform)
                    payload = make_waypoint(wx, wy, label="click")
                    self.client.publish(LEADER_WAYPOINT, json.dumps(payload), qos=1)
                    print(f"[tracker] Waypoint sent: ({wx:.3f}, {wy:.3f})")

        cv2.setMouseCallback("Position Tracker", on_click)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            debug = frame.copy()

            cv2.rectangle(debug, (0, 0), (500, 50), (0, 0, 0), -1)

            # Detect leader arrow
            leader = detect_arrow(frame, hsv, LEADER_COLOR_HSV, LEADER_COLOR_HSV2)
            if leader:
                cx, cy, angle = leader
                x, y = pixel_to_meters(cx, cy, self.transform)
                draw_debug(debug, cx, cy, angle, "Leader", (0, 100, 255))
                cv2.putText(debug, f"Leader: x={x:.3f}m  y={y:.3f}m  h={angle:.1f}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if time.time() - self.last_publish > PUBLISH_RATE:
                    self.publish_position(x, y, angle)
                    self.last_publish = time.time()

            # Detect cups
            cups_updated = False
            for label, cfg in CUP_COLORS.items():
                blob = detect_blob(hsv, cfg["hsv"], cfg.get("hsv2"), min_area=300)
                if blob:
                    cx, cy = blob
                    x, y = pixel_to_meters(cx, cy, self.transform)
                    self.cup_positions[label] = (x, y)
                    draw_cup(debug, cx, cy, label, cfg["bgr"])
                    cups_updated = True

                    # Live tracking — send position updates if tracking this cup
                    if self.tracking_cup == label:
                        track_payload = make_waypoint(x, y, label)
                        track_payload["update_only"] = True
                        self.client.publish(LEADER_WAYPOINT, json.dumps(track_payload), qos=0)

            if cups_updated:
                self.publish_cups()

            # Draw corner markers
            corner_labels = [
                f"TL(0,0)",
                f"TR({SHEET_WIDTH_M},0)",
                f"BR({SHEET_WIDTH_M},{SHEET_HEIGHT_M})",
                f"BL(0,{SHEET_HEIGHT_M})"
            ]
            for cp, lbl in zip(SHEET_CORNERS_PX.astype(int), corner_labels):
                cv2.circle(debug, tuple(cp), 8, (0, 255, 255), -1)
                cv2.putText(debug, lbl, (cp[0] + 10, cp[1] + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
            # Show cup not found warning
            if self.cup_warning and time.time() - self.cup_warning_time < 3.0:
                cv2.putText(debug, self.cup_warning, (10, 460),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
            if self.tracking_cup:
                cv2.putText(debug, f"TRACKING: {self.tracking_cup}", (10, 490),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


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