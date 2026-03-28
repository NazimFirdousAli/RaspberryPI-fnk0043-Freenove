import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import paho.mqtt.client as mqtt
from shared.topics import LEADER_STATE, FOLLOWER_STATE

def on_message(client, userdata, message):
    try:
        payload = json.loads(message.payload.decode())
        car = "LEADER  " if message.topic == LEADER_STATE else "FOLLOWER"
        print(
            f"[{car}] "
            f"x: {payload.get('x', 0):>8.4f}m  "
            f"y: {payload.get('y', 0):>8.4f}m  "
            f"heading: {payload.get('heading', 0):>8.2f}°  "
            f"mode: {payload.get('mode', '?'):<20}"
            ,end='\r'
        )
    except Exception as e:
        print(f"Parse error: {e}")

if __name__ == "__main__":
    broker_host = sys.argv[1] if len(sys.argv) > 1 else "localhost"

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(broker_host, 1883)
    client.subscribe(LEADER_STATE)
    client.subscribe(FOLLOWER_STATE)

    print(f"Monitoring car state from {broker_host}...")
    print("Press Ctrl+C to stop.\n")

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nDone.")
        