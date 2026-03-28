import json
import threading
import paho.mqtt.client as mqtt
from shared.topics import *

from shared.payloads import make_state

class CarClient:
    def __init__(self, car_id: str, broker_host: str, on_message=None):
        self.car_id = car_id
        self.on_message = on_message

        self.cmd_topic   = LEADER_CMD   if car_id == "leader" else FOLLOWER_CMD
        self.position_topic = LEADER_POSITION if car_id == "leader" else FOLLOWER_POSITION
        self.state_topic = LEADER_STATE if car_id == "leader" else FOLLOWER_STATE
        self.servo_topic = LEADER_SERVO if car_id == "leader" else FOLLOWER_SERVO
        self.waypoint_topic = LEADER_WAYPOINT if car_id == "leader" else FOLLOWER_WAYPOINT

        self.buzzer_topic = LEADER_BUZZER if car_id == "leader" else FOLLOWER_BUZZER

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(broker_host, 1883)

        self.thread = threading.Thread(target=self.client.loop_forever, daemon=True)
        self.thread.start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[{self.car_id}] Connected to broker")
        self.client.subscribe(self.cmd_topic, qos=0)
        self.client.subscribe(self.servo_topic, qos=0)
        self.client.subscribe(SYSTEM_MODE, qos=1)
        self.client.subscribe(self.buzzer_topic, qos=0)
        self.client.subscribe(self.position_topic, qos=0)
        self.client.subscribe(self.waypoint_topic, qos=1)


    def _on_message(self, client, userdata, message):
        payload = json.loads(message.payload.decode())
        if self.on_message:
            self.on_message(message.topic, payload)

    def publish_state(self, speed: float, heading: float, mode: str, x: float = 0.0, y: float = 0.0):
        payload = make_state(speed, heading, mode, x, y)
        self.client.publish(self.state_topic, json.dumps(payload), qos=0)