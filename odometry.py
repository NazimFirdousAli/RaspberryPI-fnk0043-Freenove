import time
import math

SPEED_SCALE    = 0.00065869
ROTATION_SCALE = (2 * 3.14159) / (1.73 * 1000) 

class Odometry:
    def __init__(self):
        self.x        = 0.0
        self.y        = 0.0
        self.heading  = 0.0
        self.last_time = time.time()

    def reset(self):
        self.x       = 0.0
        self.y       = 0.0
        self.heading = 0.0
        self.last_time = time.time()

    def update(self, FL: int, BL: int, FR: int, BR: int):
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        # Mecanum kinematics
        Vy =  (FL + BL + FR + BR) / 4  # forward/backward
        Vx = (-FL + BL + FR - BR) / 4  # strafe left/right
        W  = (-FL - BL + FR + BR) / 4  # rotation

        # Scale using separate constants
        Vy *= SPEED_SCALE
        Vx *= SPEED_SCALE
        W  *= ROTATION_SCALE  # ← separate scale for rotation

        # Rotate from robot frame to world frame
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        world_Vx = Vx * cos_h - Vy * sin_h
        world_Vy = Vx * sin_h + Vy * cos_h

        # Integrate
        self.x       += world_Vx * dt
        self.y       += world_Vy * dt
        self.heading += W * dt

    def get_position(self) -> dict:
        return {
            "x":       round(self.x, 4),
            "y":       round(self.y, 4),
            "heading": round(math.degrees(self.heading), 2)
        }