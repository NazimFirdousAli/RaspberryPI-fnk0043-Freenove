import time
import math
from motor import Ordinary_Car
from infrared import Infrared

CALIB_SPEED = 1000
DEBOUNCE    = 0.05

LEFT_SCALE  = 1.20
RIGHT_SCALE = 1.0

def set_motors(motor, FL, BL, FR, BR):
    clamp = lambda val: max(-4000, min(4000, val))
    FL = clamp(int(FL * LEFT_SCALE))
    BL = clamp(int(BL * LEFT_SCALE))
    FR = clamp(int(FR * RIGHT_SCALE))
    BR = clamp(int(BR * RIGHT_SCALE))
    motor.set_motor_model(FL, BL, FR, BR)

def _sustained_detection(infrared, count=3):
    """Returns True if sensor 2 reads 1 for count consecutive readings."""
    hits = 0
    for _ in range(count):
        if infrared.read_one_infrared(2) == 1:
            hits += 1
        else:
            return False
        time.sleep(0.005)
    return hits == count

def wait_for_line(infrared):
    """Wait for sensor 2 to clear, then detect tape with sustained reading."""
    # Wait for sensor to be clear (not on tape)
    while infrared.read_one_infrared(2) == 1:
        time.sleep(0.005)
    # Wait for sustained tape detection
    while not _sustained_detection(infrared):
        time.sleep(0.005)
    time.sleep(DEBOUNCE)

def calibrate_linear(motor, infrared):
    print("\n--- LINEAR CALIBRATION ---")
    print("Place car a few cm before the first tape line.")
    print("Second tape line should be exactly 1 meter ahead.")
    input("Press ENTER to start...")

    set_motors(motor, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

    # Wait for first line
    print("Driving... waiting for first tape line...")
    while not _sustained_detection(infrared):
        time.sleep(0.005)
    t_start = time.time()
    print("First line detected! Counting...")

    # Wait for sensor to clear the first line
    while infrared.read_one_infrared(2) == 1:
        time.sleep(0.005)

    # Wait for second line
    print("Waiting for second tape line...")
    while not _sustained_detection(infrared):
        time.sleep(0.005)
    t_end = time.time()

    set_motors(motor, 0, 0, 0, 0)

    elapsed = t_end - t_start
    speed_scale = 1.0 / (elapsed * CALIB_SPEED)

    print(f"\nTime between lines : {elapsed:.3f} seconds")
    print(f"SPEED_SCALE        = {speed_scale:.8f}")
    print(f"\nPaste this into odometry.py:")
    print(f"SPEED_SCALE = {speed_scale:.8f}")

    return speed_scale

def calibrate_rotation(motor, infrared):
    print("\n--- ROTATION CALIBRATION ---")
    print("Place a single piece of tape on the ground under the middle sensor.")
    print("Start the car OFF the tape line.")
    input("Press ENTER to start...")

    set_motors(motor, -CALIB_SPEED, -CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

    # Wait for first detection (start counting)
    print("Rotating... waiting for first tape crossing...")
    while not _sustained_detection(infrared):
        time.sleep(0.005)
    t_start = time.time()
    print("First crossing detected! Counting...")

    # Wait for sensor to clear
    while infrared.read_one_infrared(2) == 1:
        time.sleep(0.005)

    # Wait for second detection (halfway — ignore)
    print("Waiting for halfway crossing (ignoring)...")
    while not _sustained_detection(infrared):
        time.sleep(0.005)
    print("Halfway detected, continuing...")

    # Wait for sensor to clear again
    while infrared.read_one_infrared(2) == 1:
        time.sleep(0.005)

    # Wait for third detection (full revolution)
    print("Waiting for full revolution crossing...")
    while not _sustained_detection(infrared):
        time.sleep(0.005)
    t_end = time.time()

    set_motors(motor, 0, 0, 0, 0)

    elapsed = t_end - t_start
    rotation_scale = (2 * math.pi) / (elapsed * CALIB_SPEED)

    print(f"\nTime for full revolution : {elapsed:.3f} seconds")
    print(f"ROTATION_SCALE           = {rotation_scale:.8f}")
    print(f"\nPaste this into odometry.py:")
    print(f"ROTATION_SCALE = {rotation_scale:.8f}")

    return rotation_scale

def calibrate_swerve(motor, infrared):
    print("\n--- SWERVE CALIBRATION ---")
    print("Place two parallel tape lines exactly 1 meter apart.")
    print("Align the car straight behind the first line.")
    input("Press ENTER to start...")

    line1_times = {1: None, 2: None, 3: None}
    line2_times = {1: None, 2: None, 3: None}

    def wait_for_all_lines(times_dict):
        """Wait for all 3 sensors to transition from 0 to 1 (tape detected)."""
        # Wait until all sensors are clear
        while True:
            all_clear = all(infrared.read_one_infrared(ch) == 0 for ch in [1, 2, 3])
            if all_clear:
                break
            time.sleep(0.005)

        detected = {1: False, 2: False, 3: False}
        while not all(detected.values()):
            for ch in [1, 2, 3]:
                if not detected[ch] and infrared.read_one_infrared(ch) == 1:
                    times_dict[ch] = time.time()
                    detected[ch] = True
                    print(f"  Sensor {ch} crossed at {times_dict[ch]:.4f}")
            time.sleep(0.005)

    set_motors(motor, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

    print("\nWaiting for first tape line...")
    wait_for_all_lines(line1_times)
    print("First line complete.")

    time.sleep(DEBOUNCE)

    print("\nWaiting for second tape line...")
    wait_for_all_lines(line2_times)

    set_motors(motor, 0, 0, 0, 0)

    travel_times = {
        ch: line2_times[ch] - line1_times[ch]
        for ch in [1, 2, 3]
    }

    avg_time = sum(travel_times.values()) / 3

    print(f"\n--- RESULTS ---")
    print(f"Sensor 1 (left)   travel time: {travel_times[1]:.4f}s")
    print(f"Sensor 2 (center) travel time: {travel_times[2]:.4f}s")
    print(f"Sensor 3 (right)  travel time: {travel_times[3]:.4f}s")
    print(f"Average           travel time: {avg_time:.4f}s")

    drift_1 = travel_times[1] - avg_time
    drift_3 = travel_times[3] - avg_time

    print(f"\n--- DRIFT ANALYSIS ---")
    if abs(drift_1 - drift_3) < 0.02:
        print("✓ No significant swerve detected. Car goes straight.")
    elif travel_times[1] < travel_times[3]:
        print(f"⚠ Car drifts RIGHT (sensor 1 is faster by {travel_times[3] - travel_times[1]:.4f}s)")
    else:
        print(f"⚠ Car drifts LEFT (sensor 3 is faster by {travel_times[1] - travel_times[3]:.4f}s)")

    if abs(drift_1 - drift_3) >= 0.02:
        ratio = travel_times[1] / travel_times[3]
        print(f"\nSuggested left/right motor compensation ratio: {ratio:.4f}")
        print(f"Apply in car_loop.py as: LEFT_SCALE = {min(ratio, 1.0):.4f}, RIGHT_SCALE = {max(ratio, 1.0):.4f}")

    return travel_times

if __name__ == "__main__":
    motor    = Ordinary_Car()
    infrared = Infrared()

    try:
        print("1. Linear calibration (two tape lines, 1 meter apart)")
        print("2. Rotation calibration (one tape line, full revolution)")
        print("3. Swerve calibration (drift detection)")
        print("4. All")
        choice = input("\nChoose (1/2/3/4): ").strip()

        if choice in ("1", "4"):
            calibrate_linear(motor, infrared)

        if choice in ("2", "4"):
            calibrate_rotation(motor, infrared)

        if choice in ("3", "4"):
            calibrate_swerve(motor, infrared)

    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    finally:
        set_motors(motor, 0, 0, 0, 0)
        infrared.close()
        print("Done.")