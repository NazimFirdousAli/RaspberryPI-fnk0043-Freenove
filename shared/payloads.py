import time

def make_command(keys: set) -> dict:
    return {
        "timestamp": time.time(),
        "keys": list(keys)  # sets aren't JSON serializable, so we convert to list
    }

def make_state(speed: float, heading: float, mode: str) -> dict:
    return {
        "timestamp": time.time(),
        "speed": speed,
        "heading": heading,
        "mode": mode
        # position will be added here when SLAM is ready
    }

def make_mode(mode: str) -> dict:
    return {
        "timestamp": time.time(),
        "mode": mode
    }