import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import json
import threading
import pygame
import paho.mqtt.client as mqtt
from shared.topics import LEADER_CMD, FOLLOWER_CMD, MANUAL_TARGET, LEADER_SERVO, FOLLOWER_SERVO
from shared.payloads import make_command, make_servo

VALID_KEYS = {
    pygame.K_w: "w",
    pygame.K_a: "a",
    pygame.K_s: "s",
    pygame.K_d: "d",
    pygame.K_q: "q",
    pygame.K_e: "e",
}

# Servo settings
PAN_CENTER  = 90
TILT_CENTER = 90
SERVO_MIN   = 0
SERVO_MAX   = 180
SERVO_STEP  = 5

class ManualController:
    def __init__(self, broker_host: str):
        self.broker_host = broker_host
        self.pressed_keys = set()
        self.target = "leader"
        self.running = True

        # Servo state
        self.pan  = PAN_CENTER
        self.tilt = TILT_CENTER

        # MQTT setup
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

    def _publish_keys(self):
        payload = make_command(self.pressed_keys)
        self.client.publish(self._get_cmd_topic(), json.dumps(payload), qos=0)

    def _publish_servo(self):
        payload = make_servo(self.pan, self.tilt)
        self.client.publish(self._get_servo_topic(), json.dumps(payload), qos=0)

    def _clamp_servo(self, val):
        return max(SERVO_MIN, min(SERVO_MAX, val))

    def _draw(self, screen, font):
        screen.fill((30, 30, 30))

        title    = font.render("Freenove Manual Controller", True, (255, 255, 255))
        target   = font.render(f"Target: {self.target}", True, (0, 255, 100))
        keys     = font.render(f"Keys: {sorted(self.pressed_keys)}", True, (255, 255, 0))
        servo    = font.render(f"Servo — Pan: {self.pan}°  Tilt: {self.tilt}°", True, (100, 200, 255))
        controls = font.render("WASD/QE: move  |  Arrows: servo  |  TAB: switch  |  ESC: quit", True, (180, 180, 180))

        screen.blit(title,    (20, 20))
        screen.blit(target,   (20, 60))
        screen.blit(keys,     (20, 100))
        screen.blit(servo,    (20, 140))
        screen.blit(controls, (20, 190))

        pygame.display.flip()

    def start(self):
        pygame.init()
        screen = pygame.display.set_mode((560, 230))
        pygame.display.set_caption("Manual Controller")
        font = pygame.font.SysFont("monospace", 18)
        clock = pygame.time.Clock()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key in VALID_KEYS:
                        self.pressed_keys.add(VALID_KEYS[event.key])
                        self._publish_keys()
                    elif event.key == pygame.K_LEFT:
                        self.pan = self._clamp_servo(self.pan - SERVO_STEP)
                        self._publish_servo()
                    elif event.key == pygame.K_RIGHT:
                        self.pan = self._clamp_servo(self.pan + SERVO_STEP)
                        self._publish_servo()
                    elif event.key == pygame.K_UP:
                        self.tilt = self._clamp_servo(self.tilt + SERVO_STEP)
                        self._publish_servo()
                    elif event.key == pygame.K_DOWN:
                        self.tilt = self._clamp_servo(self.tilt - SERVO_STEP)
                        self._publish_servo()
                    elif event.key == pygame.K_TAB:
                        self.target = "follower" if self.target == "leader" else "leader"
                        self.client.publish(MANUAL_TARGET, json.dumps({"target": self.target}), qos=1)
                    elif event.key == pygame.K_ESCAPE:
                        self.running = False

                elif event.type == pygame.KEYUP:
                    if event.key in VALID_KEYS:
                        self.pressed_keys.discard(VALID_KEYS[event.key])
                        self._publish_keys()

            self._publish_keys()
            self._draw(screen, font)
            clock.tick(20)

        # Clean up
        self.pressed_keys.clear()
        self._publish_keys()
        self.client.disconnect()
        pygame.quit()

if __name__ == "__main__":
    broker_host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    controller = ManualController(broker_host=broker_host)
    controller.start()