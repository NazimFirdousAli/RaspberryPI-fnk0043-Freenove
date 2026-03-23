import json
import threading
import paho.mqtt.client as mqtt
from shared.topics import LEADER_CMD, FOLLOWER_CMD, LEADER_STATE, FOLLOWER_STATE, LEADER_SERVO, FOLLOWER_SERVO, SYSTEM_MODE
from shared.payloads import make_state

class CarClient:
    def __init__(self, car_id: str, broker_host: str, on_message=None):
        self.car_id = car_id
        self.on_message = on_message

        self.cmd_topic   = LEADER_CMD   if car_id == "leader" else FOLLOWER_CMD
        self.state_topic = LEADER_STATE if car_id == "leader" else FOLLOWER_STATE
        self.servo_topic = LEADER_SERVO if car_id == "leader" else FOLLOWER_SERVO


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

    def _on_message(self, client, userdata, message):
        payload = json.loads(message.payload.decode())
        if self.on_message:
            self.on_message(message.topic, payload)

    def publish_state(self, speed: float, heading: float, mode: str):
        payload = make_state(speed, heading, mode)
        self.client.publish(self.state_topic, json.dumps(payload), qos=0)