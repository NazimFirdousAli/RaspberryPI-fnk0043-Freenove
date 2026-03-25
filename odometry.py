import time
import math

# Calibration constant — replace after measuring on real hardware
# This represents how many meters/second the car moves at motor value 1.0
# e.g. if at motor value 1500 the car moves 0.3 m/s, then SPEED_SCALE = 0.3 / 1500
SPEED_SCALE = 0.0002  # placeholder — calibrate this!

class Odometry:
    def __init__(self):
        self.x        = 0.0  # meters, positive = right
        self.y        = 0.0  # meters, positive = forward
        self.heading  = 0.0  # radians, positive = counterclockwise
        self.last_time = time.time()

    def reset(self):
        self.x       = 0.0
        self.y       = 0.0
        self.heading = 0.0
        self.last_time = time.time()

    def update(self, FL: int, BL: int, FR: int, BR: int):
        """
        Call this every loop iteration with the current wheel speeds.
        Updates x, y, heading based on elapsed time and mecanum kinematics.
        """
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        # Mecanum kinematics — decompose wheel speeds into Vx, Vy, W
        Vy =  (FL + BL + FR + BR) / 4  # forward/backward
        Vx = (-FL + BL + FR - BR) / 4  # strafe left/right
        W  = (-FL - BL + FR + BR) / 4  # rotation

        # Scale from motor units to m/s
        Vy *= SPEED_SCALE
        Vx *= SPEED_SCALE
        W  *= SPEED_SCALE

        # Rotate Vx/Vy from robot frame to world frame using current heading
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        world_Vx = Vx * cos_h - Vy * sin_h
        world_Vy = Vx * sin_h + Vy * cos_h

        # Integrate velocity into position
        self.x       += world_Vx * dt
        self.y       += world_Vy * dt
        self.heading += W * dt

    def get_position(self) -> dict:
        return {
            "x":       round(self.x, 4),
            "y":       round(self.y, 4),
            "heading": round(math.degrees(self.heading), 2)  # convert to degrees for readability
        }