import time
import math
from motor import Ordinary_Car
from infrared import Infrared

CALIB_SPEED = 1500  # motor value used during calibration
DEBOUNCE    = 0.3   # seconds to wait after a detection before looking again

def wait_for_line(infrared, debounce=DEBOUNCE):
    """Block until the middle infrared sensor detects a black line."""
    while True:
        if infrared.read_one_infrared(2) == 1:
            time.sleep(debounce)  # debounce — avoid double-triggering
            return
        time.sleep(0.005)

def calibrate_linear(motor, infrared):
    print("\n--- LINEAR CALIBRATION ---")
    print("Place car before first tape line, 1 meter from second tape line.")
    input("Press ENTER to start...")

    # Drive forward
    motor.set_motor_model(CALIB_SPEED, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

    print("Waiting for first tape line...")
    wait_for_line(infrared)
    t_start = time.time()
    print("First line detected! Waiting for second...")

    wait_for_line(infrared)
    t_end = time.time()

    motor.set_motor_model(0, 0, 0, 0)

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
    input("Press ENTER to start...")

    # Rotate left
    motor.set_motor_model(-CALIB_SPEED, -CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

    print("Waiting for tape line (start)...")
    wait_for_line(infrared)
    t_start = time.time()
    print("Start detected! Waiting for one full revolution...")

    wait_for_line(infrared)
    t_end = time.time()

    motor.set_motor_model(0, 0, 0, 0)

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

    # Track crossing times for each sensor
    line1_times = {1: None, 2: None, 3: None}
    line2_times = {1: None, 2: None, 3: None}

    def wait_for_all_lines(times_dict):
        """Wait until all 3 sensors have crossed a line."""
        detected = {1: False, 2: False, 3: False}
        while not all(detected.values()):
            for ch in [1, 2, 3]:
                if not detected[ch] and infrared.read_one_infrared(ch) == 1:
                    times_dict[ch] = time.time()
                    detected[ch] = True
                    print(f"  Sensor {ch} crossed at {times_dict[ch]:.4f}")
            time.sleep(0.005)

    # Drive forward
    motor.set_motor_model(CALIB_SPEED, CALIB_SPEED, CALIB_SPEED, CALIB_SPEED)

    print("\nWaiting for first tape line...")
    wait_for_all_lines(line1_times)
    print("First line complete.")

    # Small debounce gap between lines
    time.sleep(DEBOUNCE)

    print("\nWaiting for second tape line...")
    wait_for_all_lines(line2_times)

    motor.set_motor_model(0, 0, 0, 0)

    # Compute travel time per sensor
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

    # Drift analysis
    drift_1 = travel_times[1] - avg_time
    drift_3 = travel_times[3] - avg_time

    print(f"\n--- DRIFT ANALYSIS ---")
    if abs(drift_1 - drift_3) < 0.02:
        print("✓ No significant swerve detected. Car goes straight.")
    elif travel_times[1] < travel_times[3]:
        print(f"⚠ Car drifts RIGHT (sensor 1 is faster by {travel_times[3] - travel_times[1]:.4f}s)")
    else:
        print(f"⚠ Car drifts LEFT (sensor 3 is faster by {travel_times[1] - travel_times[3]:.4f}s)")

    # Suggest motor compensation
    if abs(drift_1 - drift_3) >= 0.02:
        ratio = travel_times[1] / travel_times[3]
        print(f"\nSuggested left/right motor compensation ratio: {ratio:.4f}")
        print(f"Apply in odometry.py as: LEFT_SCALE = {min(ratio, 1.0):.4f}, RIGHT_SCALE = {max(ratio, 1.0):.4f}")

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
        motor.set_motor_model(0, 0, 0, 0)
        infrared.close()
        print("Done.")