import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import json
import threading
import pygame
import paho.mqtt.client as mqtt
from shared.payloads import make_command, make_servo, make_buzzer, make_mode
from video_receiver import VideoReceiver
from shared.topics import LEADER_CMD, FOLLOWER_CMD, LEADER_SERVO, FOLLOWER_SERVO, SYSTEM_MODE, MANUAL_TARGET, LEADER_BUZZER, FOLLOWER_BUZZER, LEADER_WAYPOINT

VALID_KEYS = {
    pygame.K_w: "w",
    pygame.K_a: "a",
    pygame.K_s: "s",
    pygame.K_d: "d",
    pygame.K_q: "q",
    pygame.K_e: "e",
}

# Servo settings
PAN_CENTER  = 60
TILT_CENTER = 0
PAN_MIN     = 0
PAN_MAX     = 120
TILT_MIN    = 0
TILT_MAX    = 180
SERVO_STEP  = 5

# Window
WINDOW_W      = 800
WINDOW_H      = 620  # 500 video + 60 mode buttons + 60 cup buttons
BUTTON_HEIGHT = 40
BUTTON_MARGIN = 10

BUTTONS = [
    {"label": "MANUAL",      "mode": "manual"},
    {"label": "AUTO",        "mode": "auto"},
    {"label": "RETURN HOME", "mode": "return_home"},
]

CUP_BUTTONS = [
    {"label": "LIGHT BLUE", "color_label": "light_blue", "color": (173, 216, 230)},
    {"label": "PURPLE",     "color_label": "purple",     "color": (147, 0,   211)},
    {"label": "GREEN",      "color_label": "green",      "color": (0,   200, 0  )},
]

class ManualController:
    def __init__(self, broker_host: str, pi_host: str):
        self.broker_host  = broker_host
        self.pressed_keys = set()
        self.servo_keys   = set()
        self.target       = "leader"
        self.running      = True
        self.current_mode = "manual"

        self.video = VideoReceiver(host=pi_host)
        self.video.start()

        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self._on_connect
        self.client.connect(broker_host, 1883)

        self.thread = threading.Thread(target=self.client.loop_forever, daemon=True)
        self.thread.start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[controller] Connected to broker")

    def _get_cmd_topic(self):
        return LEADER_CMD if self.target == "leader" else FOLLOWER_CMD

    def _get_servo_topic(self):
        return LEADER_SERVO if self.target == "leader" else FOLLOWER_SERVO

    def _get_buzzer_topic(self):
        return LEADER_BUZZER if self.target == "leader" else FOLLOWER_BUZZER

    def _publish_keys(self):
        payload = make_command(self.pressed_keys)
        self.client.publish(self._get_cmd_topic(), json.dumps(payload), qos=0)

    def _publish_servo(self):
        payload = make_servo(self.pan, self.tilt)
        self.client.publish(self._get_servo_topic(), json.dumps(payload), qos=0)

    def _publish_buzzer(self, state: bool):
        payload = make_buzzer(state)
        self.client.publish(self._get_buzzer_topic(), json.dumps(payload), qos=0)

    def _publish_mode(self, mode: str):
        payload = make_mode(mode)
        self.client.publish(SYSTEM_MODE, json.dumps(payload), qos=1)
        self.current_mode = mode
        print(f"[controller] Mode → {mode}")

    def _publish_cup_waypoint(self, color_label: str):
        """Send car to a cup by color label — position tracker will fill in coordinates."""
        payload = {"x": 0, "y": 0, "label": color_label, "timestamp": __import__('time').time()}
        self.client.publish(LEADER_WAYPOINT, json.dumps(payload), qos=1)
        print(f"[controller] Go to cup: {color_label}")

    def _clamp_pan(self, val):
        return max(PAN_MIN, min(PAN_MAX, val))

    def _clamp_tilt(self, val):
        return max(TILT_MIN, min(TILT_MAX, val))

    def _draw(self, screen, font):
        screen.fill((30, 30, 30))

        # Video feed
        frame = self.video.get_surface()
        if frame is not None:
            frame = pygame.transform.scale(frame, (WINDOW_W, 500))
            screen.blit(frame, (0, 0))

        # HUD
        title    = font.render("Freenove Manual Controller", True, (255, 255, 255))
        target   = font.render(f"Target: {self.target}", True, (0, 255, 100))
        keys     = font.render(f"Keys: {sorted(self.pressed_keys)}", True, (255, 255, 0))
        servo    = font.render(f"Servo — Pan: {self.pan}°  Tilt: {self.tilt}°", True, (100, 200, 255))
        controls = font.render("WASD/QE: move  |  Arrows: servo  |  SPACE: horn  |  TAB: switch  |  ESC: quit", True, (180, 180, 180))

        screen.blit(title,    (20, 20))
        screen.blit(target,   (20, 60))
        screen.blit(keys,     (20, 100))
        screen.blit(servo,    (20, 140))
        screen.blit(controls, (20, 190))

        # Mode buttons — row 1
        button_w = (WINDOW_W - BUTTON_MARGIN * (len(BUTTONS) + 1)) // len(BUTTONS)
        for i, btn in enumerate(BUTTONS):
            x = BUTTON_MARGIN + i * (button_w + BUTTON_MARGIN)
            y = 500 + BUTTON_MARGIN
            is_active = (self.current_mode == btn["mode"])
            color = (0, 180, 100) if is_active else (70, 70, 70)
            pygame.draw.rect(screen, color, (x, y, button_w, BUTTON_HEIGHT), border_radius=6)
            label = font.render(btn["label"], True, (255, 255, 255))
            label_rect = label.get_rect(center=(x + button_w // 2, y + BUTTON_HEIGHT // 2))
            screen.blit(label, label_rect)

        # Cup buttons — row 2
        cup_button_w = (WINDOW_W - BUTTON_MARGIN * (len(CUP_BUTTONS) + 1)) // len(CUP_BUTTONS)
        for i, btn in enumerate(CUP_BUTTONS):
            x = BUTTON_MARGIN + i * (cup_button_w + BUTTON_MARGIN)
            y = 500 + BUTTON_MARGIN * 2 + BUTTON_HEIGHT
            pygame.draw.rect(screen, btn["color"], (x, y, cup_button_w, BUTTON_HEIGHT), border_radius=6)
            label = font.render(btn["label"], True, (0, 0, 0))
            label_rect = label.get_rect(center=(x + cup_button_w // 2, y + BUTTON_HEIGHT // 2))
            screen.blit(label, label_rect)

        pygame.display.flip()

    def start(self):
        pygame.init()
        screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("Manual Controller")
        font = pygame.font.SysFont("monospace", 18)
        clock = pygame.time.Clock()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.WINDOWFOCUSLOST:
                    self.pressed_keys.clear()
                    self.servo_keys.clear()
                    self._publish_keys()

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    button_w = (WINDOW_W - BUTTON_MARGIN * (len(BUTTONS) + 1)) // len(BUTTONS)
                    # Mode buttons
                    for i, btn in enumerate(BUTTONS):
                        x = BUTTON_MARGIN + i * (button_w + BUTTON_MARGIN)
                        y = 500 + BUTTON_MARGIN
                        if x <= mx <= x + button_w and y <= my <= y + BUTTON_HEIGHT:
                            self._publish_mode(btn["mode"])

                    # Cup buttons
                    cup_button_w = (WINDOW_W - BUTTON_MARGIN * (len(CUP_BUTTONS) + 1)) // len(CUP_BUTTONS)
                    for i, btn in enumerate(CUP_BUTTONS):
                        x = BUTTON_MARGIN + i * (cup_button_w + BUTTON_MARGIN)
                        y = 500 + BUTTON_MARGIN * 2 + BUTTON_HEIGHT
                        if x <= mx <= x + cup_button_w and y <= my <= y + BUTTON_HEIGHT:
                            self._publish_cup_waypoint(btn["color_label"])

                elif event.type == pygame.KEYDOWN:
                    if event.key in VALID_KEYS:
                        self.pressed_keys.add(VALID_KEYS[event.key])
                        self._publish_keys()
                    elif event.key in (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP, pygame.K_DOWN):
                        self.servo_keys.add(event.key)
                    elif event.key == pygame.K_SPACE:
                        self._publish_buzzer(True)
                    elif event.key == pygame.K_TAB:
                        self.target = "follower" if self.target == "leader" else "leader"
                        self.client.publish(MANUAL_TARGET, json.dumps({"target": self.target}), qos=1)
                    elif event.key == pygame.K_ESCAPE:
                        self.running = False

                elif event.type == pygame.KEYUP:
                    if event.key in VALID_KEYS:
                        self.pressed_keys.discard(VALID_KEYS[event.key])
                        self._publish_keys()
                    elif event.key == pygame.K_SPACE:
                        self._publish_buzzer(False)
                    elif event.key in (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP, pygame.K_DOWN):
                        self.servo_keys.discard(event.key)

            # Handle held servo keys
            servo_moved = False
            if pygame.K_LEFT in self.servo_keys:
                self.pan = self._clamp_pan(self.pan - SERVO_STEP)
                servo_moved = True
            if pygame.K_RIGHT in self.servo_keys:
                self.pan = self._clamp_pan(self.pan + SERVO_STEP)
                servo_moved = True
            if pygame.K_UP in self.servo_keys:
                self.tilt = self._clamp_tilt(self.tilt + SERVO_STEP)
                servo_moved = True
            if pygame.K_DOWN in self.servo_keys:
                self.tilt = self._clamp_tilt(self.tilt - SERVO_STEP)
                servo_moved = True
            if servo_moved:
                self._publish_servo()

            self._publish_keys()
            self._draw(screen, font)
            clock.tick(20)

        # Clean up
        self.pressed_keys.clear()
        # Only publish keys if pygame window has focus
        if pygame.key.get_focused():
            self._publish_keys()
        elif self.pressed_keys:
            self.pressed_keys.clear()
            self._publish_keys()
        self.client.disconnect()
        pygame.quit()

if __name__ == "__main__":
    broker_host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    pi_host = sys.argv[2] if len(sys.argv) > 2 else "10.56.45.66"
    controller = ManualController(broker_host=broker_host, pi_host=pi_host)
    controller.start()