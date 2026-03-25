"""
cup_recognizer.py — Freenove FNK0043 Cup Recognition Module
============================================================
Detects coloured plastic cups (Green, Blue, Orange, Purple) using the
Pi camera and steers the car toward the nearest detected cup.

Integrates with the existing project structure:
  - camera.py  → Camera / StreamingOutput
  - motor.py   → Ordinary_Car
  - video_streamer.py → can overlay detections on the live stream

Usage
-----
  # Standalone (car moves toward detected cup):
  python3 cup_recognizer.py

  # Import as a module (e.g. from main.py):
  from cup_recognizer import CupRecognizer
  detector = CupRecognizer()
  result = detector.detect_from_frame(bgr_frame)
  print(result)   # {'color': 'Blue', 'x':120, 'y':80, 'w':60, 'h':90, 'distance_cm':25.3}
"""

import cv2
import numpy as np
import os
import sys
import time

# ── make sure the rest of the project is importable ──────────────────────────
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


# ══════════════════════════════════════════════════════════════════════════════
#  HSV colour definitions
#  Tuned for the coloured Solo-style plastic cups in typical indoor lighting.
#  If detection is unreliable, run  python3 cup_recognizer.py --calibrate
#  to interactively adjust the sliders.
# ══════════════════════════════════════════════════════════════════════════════

CUP_COLORS = {
    # ── Green ──────────────────────────────────────────────────────────────
    # Bright lime / grass green cups
    "Green": [
        (np.array([35,  60,  50]), np.array([85, 255, 255])),
    ],

    # ── Blue ───────────────────────────────────────────────────────────────
    # Medium teal-blue (like the cups in the photo)
    "Blue": [
        (np.array([85, 70, 50]), np.array([135, 255, 255])),
    ],

    # ── Orange ─────────────────────────────────────────────────────────────
    # Warm orange — distinct from red; wraps around H=15-25
    "Orange": [
        (np.array([5,  120, 100]), np.array([25, 255, 255])),
    ],

    # ── Purple ─────────────────────────────────────────────────────────────
    # Violet / magenta purple
    "Purple": [
        (np.array([125, 40,  50]), np.array([165, 255, 255])),
    ],
}

# Colour of the bounding-box drawn on screen (BGR)
BOX_BGR = {
    "Green":  (0,   200,   0),
    "Blue":   (230, 100,   0),
    "Orange": (0,   140, 255),
    "Purple": (180,   0, 200),
}

# ── Distance estimation ───────────────────────────────────────────────────────
# Adjust FOCAL_LENGTH after one calibration shot:
#   hold cup at CALIB_DISTANCE_CM, measure perceived_width in pixels → set
#   FOCAL_LENGTH = perceived_width * CALIB_DISTANCE_CM / REAL_CUP_WIDTH_CM
REAL_CUP_WIDTH_CM  = 8.0    # physical diameter of the cup at its widest (~8 cm)
FOCAL_LENGTH       = 480    # pixels  ← calibrate this for your lens / resolution
MIN_CONTOUR_AREA   = 800    # px²  — ignore tiny noise blobs
STOP_DISTANCE_CM   = 18     # stop the car when cup is this close


# ══════════════════════════════════════════════════════════════════════════════
#  CupRecognizer  — pure vision, no motors
# ══════════════════════════════════════════════════════════════════════════════

class CupRecognizer:
    """
    Detects coloured cups in a single BGR frame.

    Returns a dict (or None) with keys:
        color        – 'Green' | 'Blue' | 'Orange' | 'Purple'
        x, y, w, h  – bounding box in pixels
        cx, cy      – centroid
        distance_cm – estimated distance in centimetres
        annotated   – the frame with bounding box drawn on it
    """

    def __init__(self, min_area: int = MIN_CONTOUR_AREA):
        self.min_area = min_area
        self._kernel  = np.ones((5, 5), np.uint8)

    # ── internal helpers ──────────────────────────────────────────────────────

    def _build_mask(self, hsv, ranges):
        mask = None
        for lower, upper in ranges:
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # clean up noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        return mask

    @staticmethod
    def _estimate_distance(perceived_width_px: float) -> float:
        if perceived_width_px <= 0:
            return 9999.0
        return (REAL_CUP_WIDTH_CM * FOCAL_LENGTH) / perceived_width_px

    # ── public API ────────────────────────────────────────────────────────────

    def detect_from_frame(self, bgr_frame: np.ndarray) -> dict | None:
        """
        Analyse one BGR frame and return the largest detected cup, or None.
        The returned dict also contains an 'annotated' key with the frame
        drawn on (bounding box + label).
        """
        annotated = bgr_frame.copy()
        hsv       = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        best      = None
        best_area = 0

        for color_name, ranges in CUP_COLORS.items():
            mask       = self._build_mask(hsv, ranges)
            contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area or area <= best_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                best_area   = area
                best        = dict(
                    color       = color_name,
                    x=x, y=y, w=w, h=h,
                    cx          = x + w // 2,
                    cy          = y + h // 2,
                    distance_cm = self._estimate_distance(w),
                )

        if best is None:
            return None

        # Draw annotation
        clr  = BOX_BGR[best["color"]]
        x, y, w, h = best["x"], best["y"], best["w"], best["h"]
        cv2.rectangle(annotated, (x, y), (x + w, y + h), clr, 2)
        label = f"{best['color']}  {best['distance_cm']:.0f} cm"
        cv2.putText(annotated, label, (x, max(y - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, clr, 2)
        best["annotated"] = annotated
        return best

    def detect_all_from_frame(self, bgr_frame: np.ndarray) -> list[dict]:
        """
        Return ALL detected cups (sorted by area, largest first).
        Each dict has the same keys as detect_from_frame, minus 'annotated'.
        """
        annotated = bgr_frame.copy()
        hsv       = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        results   = []

        for color_name, ranges in CUP_COLORS.items():
            mask       = self._build_mask(hsv, ranges)
            contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                results.append(dict(
                    color       = color_name,
                    x=x, y=y, w=w, h=h,
                    cx          = x + w // 2,
                    cy          = y + h // 2,
                    distance_cm = self._estimate_distance(w),
                    area        = area,
                ))

        results.sort(key=lambda d: d["area"], reverse=True)

        # draw all of them
        for det in results:
            clr   = BOX_BGR[det["color"]]
            x, y  = det["x"], det["y"]
            w, h  = det["w"], det["h"]
            cv2.rectangle(annotated, (x, y), (x + w, y + h), clr, 2)
            label = f"{det['color']} {det['distance_cm']:.0f}cm"
            cv2.putText(annotated, label, (x, max(y - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, clr, 2)

        if results:
            results[0]["annotated"] = annotated
        return results


# ══════════════════════════════════════════════════════════════════════════════
#  CupFollower  — vision + motor control
#  Steers the Freenove car toward the nearest detected cup and stops close up.
# ══════════════════════════════════════════════════════════════════════════════

class CupFollower:
    """
    Combines CupRecognizer with the Ordinary_Car motor interface.

    The car:
      • cruises forward when no cup is visible
      • steers left/right to centre the cup in frame
      • stops when the cup is within STOP_DISTANCE_CM
      • optionally announces the cup colour via espeak
    """

    # Motor speeds (0–4000 range used by Ordinary_Car)
    SPEED_FORWARD = 1200
    SPEED_TURN    = 1000
    SPEED_SLOW    = 700

    def __init__(self, camera_index: int = 0,
                 frame_width: int = 320, frame_height: int = 240,
                 use_voice: bool = True,
                 stop_distance_cm: float = STOP_DISTANCE_CM):

        self.recognizer       = CupRecognizer()
        self.stop_distance_cm = stop_distance_cm
        self.use_voice        = use_voice
        self._last_voice_t    = 0
        self._last_color      = None

        # ── camera ───────────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.frame_width = frame_width

        # ── motors ───────────────────────────────────────────────────────────
        try:
            from motor import Ordinary_Car
            self.car = Ordinary_Car()
        except ImportError:
            print("[CupFollower] WARNING: motor.py not found — running in "
                  "vision-only mode (no movement).")
            self.car = None

    # ── motor helpers ─────────────────────────────────────────────────────────

    def _stop(self):
        if self.car:
            self.car.set_motor_model(0, 0, 0, 0)

    def _forward(self, speed=None):
        s = speed or self.SPEED_FORWARD
        if self.car:
            self.car.set_motor_model(s, s, s, s)

    def _turn_left(self):
        if self.car:
            # left wheels backward, right wheels forward
            s = self.SPEED_TURN
            self.car.set_motor_model(-s, -s, s, s)

    def _turn_right(self):
        if self.car:
            s = self.SPEED_TURN
            self.car.set_motor_model(s, s, -s, -s)

    # ── voice ─────────────────────────────────────────────────────────────────

    def _speak(self, text: str):
        """Non-blocking espeak call (requires espeak installed on Pi)."""
        if not self.use_voice:
            return
        now = time.time()
        if now - self._last_voice_t > 3.0:          # throttle: max once / 3 s
            print(f"[voice] {text}")
            os.system(f'espeak "{text}" &')
            self._last_voice_t = now

    # ── main loop ────────────────────────────────────────────────────────────

    def run(self):
        print("[CupFollower] Starting. Press 'q' to quit.")
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("[CupFollower] Camera read failed — retrying...")
                    time.sleep(0.05)
                    continue

                detection = self.recognizer.detect_from_frame(frame)

                if detection is None:
                    # Nothing found — slow forward cruise
                    self._forward(self.SPEED_SLOW)
                    print("[CupFollower] Searching...")

                else:
                    color    = detection["color"]
                    dist_cm  = detection["distance_cm"]
                    cx       = detection["cx"]
                    frame_cx = self.frame_width // 2
                    offset   = cx - frame_cx          # + = cup is to the right

                    print(f"[CupFollower] {color} cup | "
                          f"dist={dist_cm:.0f} cm | offset={offset:+d} px")

                    # Announce new colour or re-announce when close
                    if color != self._last_color or dist_cm < self.stop_distance_cm:
                        self._speak(f"I see a {color} cup")
                        self._last_color = color

                    if dist_cm <= self.stop_distance_cm:
                        self._stop()
                        self._speak(f"Reached the {color} cup")
                        print(f"[CupFollower] ✓ Reached {color} cup at "
                              f"{dist_cm:.0f} cm — stopped.")
                        # Brief pause before searching again
                        time.sleep(2)
                        self._last_color = None
                    elif abs(offset) > frame_cx * 0.25:   # >25 % off-centre
                        if offset > 0:
                            self._turn_right()
                        else:
                            self._turn_left()
                    else:
                        self._forward()

                # Show annotated frame (VNC / display)
                display = detection["annotated"] if detection else frame
                cv2.imshow("Cup Recognizer", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self._stop()
            self.cap.release()
            cv2.destroyAllWindows()
            print("[CupFollower] Stopped.")


# ══════════════════════════════════════════════════════════════════════════════
#  Interactive HSV calibration tool
#  Run:  python3 cup_recognizer.py --calibrate
# ══════════════════════════════════════════════════════════════════════════════

def run_calibration():
    """
    Opens a window with HSV trackbars so you can dial in the exact colour
    ranges for your specific cups under your lighting conditions.
    Copy the printed values back into the CUP_COLORS dict above.
    """
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
    for name, val, maxv in [
        ("H_low", 0, 179), ("H_high", 179, 179),
        ("S_low", 0, 255), ("S_high", 255, 255),
        ("V_low", 0, 255), ("V_high", 255, 255),
    ]:
        cv2.createTrackbar(name, "Calibrate", val, maxv, lambda x: None)

    print("[calibrate] Adjust sliders, press 'q' when done.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_lo = cv2.getTrackbarPos("H_low",  "Calibrate")
        h_hi = cv2.getTrackbarPos("H_high", "Calibrate")
        s_lo = cv2.getTrackbarPos("S_low",  "Calibrate")
        s_hi = cv2.getTrackbarPos("S_high", "Calibrate")
        v_lo = cv2.getTrackbarPos("V_low",  "Calibrate")
        v_hi = cv2.getTrackbarPos("V_high", "Calibrate")

        mask   = cv2.inRange(hsv,
                             np.array([h_lo, s_lo, v_lo]),
                             np.array([h_hi, s_hi, v_hi]))
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Calibrate", np.hstack([frame, result]))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"\n[calibrate] Final HSV range:\n"
                  f"  lower = [{h_lo}, {s_lo}, {v_lo}]\n"
                  f"  upper = [{h_hi}, {s_hi}, {v_hi}]")
            break

    cap.release()
    cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Freenove Cup Recognizer")
    parser.add_argument("--calibrate", action="store_true",
                        help="Open HSV calibration tool")
    parser.add_argument("--vision-only", action="store_true",
                        help="Run detection without moving the car")
    parser.add_argument("--no-voice", action="store_true",
                        help="Disable espeak voice output")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default 0)")
    args = parser.parse_args()

    if args.calibrate:
        run_calibration()
    else:
        follower = CupFollower(
            camera_index=args.camera,
            use_voice=not args.no_voice,
        )
        if args.vision_only and follower.car:
            follower.car = None   # disable motors
        follower.run()



# ══════════════════════════════════════════════════════════════════════════════
# To run this program 
# Full mode (moves + talks)
# python3 cup_recognizer.py

# Vision only, no motors
# python3 cup_recognizer.py --vision-only

# Silence the voice
# python3 cup_recognizer.py --no-voice

# Interactive HSV slider tool to fine-tune colours
# python3 cup_recognizer.py --calibrate
# ══════════════════════════════════════════════════════════════════════════════
