import time

def make_command(keys: set) -> dict:
    return {
        "timestamp": time.time(),
        "keys": list(keys)  # sets aren't JSON serializable, so we convert to list
    }

def make_servo(pan: int, tilt: int) -> dict:
    return {
        "timestamp": time.time(),
        "pan": pan,
        "tilt": tilt
    }

def make_buzzer(state: bool) -> dict:
    return {
        "timestamp": time.time(),
        "state": state
    }

def make_state(speed: float, heading: float, mode: str, x: float = 0.0, y: float = 0.0) -> dict:
    return {
        "timestamp": time.time(),
        "speed": speed,
        "heading": heading,
        "mode": mode,
        "x": x,
        "y": y
    }

def make_waypoint(x: float, y: float, label: str = "") -> dict:
    return {
        "timestamp": time.time(),
        "x": x,
        "y": y,
        "label": label  # e.g. "green_cup", "home", "charging"
    }

def make_mode(mode: str) -> dict:
    return {
        "timestamp": time.time(),
        "mode": mode
    }