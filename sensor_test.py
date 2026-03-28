import time
from motor import Ordinary_Car
from infrared import Infrared

CALIB_SPEED = 1000
LEFT_SCALE  = 1.20
RIGHT_SCALE = 1.0

def set_motors(motor, FL, BL, FR, BR):
    clamp = lambda val: max(-4000, min(4000, val))
    FL = clamp(int(FL * LEFT_SCALE))
    BL = clamp(int(BL * LEFT_SCALE))
    FR = clamp(int(FR * RIGHT_SCALE))
    BR = clamp(int(BR * RIGHT_SCALE))
    motor.set_motor_model(FL, BL, FR, BR)

motor    = Ordinary_Car()
infrared = Infrared()

input("Press ENTER to start 3 second run...")

set_motors(motor, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

t_start = time.time()
readings = []

while time.time() - t_start < 3.0:
    s1 = infrared.read_one_infrared(1)
    s2 = infrared.read_one_infrared(2)
    s3 = infrared.read_one_infrared(3)
    t  = round(time.time() - t_start, 3)
    readings.append((t, s1, s2, s3))
    time.sleep(0.005)

set_motors(motor, 0, 0, 0, 0)
infrared.close()

print("\nTime  | S1 | S2 | S3")
print("------|----|----|----|")
for t, s1, s2, s3 in readings:
    print(f"{t:.3f} |  {s1} |  {s2} |  {s3}")