# Freenove RC Car — Startup Steps

## 1. Broker (Laptop — Terminal 1)

```bash
mosquitto -c controller/mosquitto.conf -v
```

## 2. Position Tracker (Laptop — Terminal 2)

```bash
cd path/to/RaspberryPI-fnk0043-Freenove
source venv/bin/activate
python3 controller/position_tracker.py localhost
```

Click the 4 corners of the tarp when prompted: **Top-Left → Top-Right → Bottom-Right → Bottom-Left**

## 3. Video Streamer (Pi — Terminal 1)

```bash
cd ~/teamProject/RaspberryPI-fnk0043-Freenove
python3 video_streamer.py
```

## 4. Car Loop (Pi — Terminal 2)

```bash
cd ~/teamProject/RaspberryPI-fnk0043-Freenove
python3 car_loop.py leader 10.56.45.88
```

## 5. Manual Controller (Laptop — Terminal 3)

```bash
cd path/to/RaspberryPI-fnk0043-Freenove
source venv/bin/activate
python3 controller/manual_controller.py localhost 10.56.45.66
```

---

## Key Info

| Device          | IP          |
| --------------- | ----------- |
| Laptop (broker) | 10.56.45.88 |
| Pi (car)        | 10.56.45.66 |
| Video port      | 9999        |
| MQTT port       | 1883        |

---

## Controls

| Key        | Action         |
| ---------- | -------------- |
| W          | Forward        |
| S          | Backward       |
| A          | Rotate left    |
| D          | Rotate right   |
| Q          | Strafe left    |
| E          | Strafe right   |
| Arrow keys | Pan/tilt servo |
| SPACE      | Horn           |
| ESC        | Quit           |

## Mode Buttons

| Button      | Action                                              |
| ----------- | --------------------------------------------------- |
| MANUAL      | Manual control                                      |
| AUTO        | Obstacle avoidance                                  |
| RETURN HOME | Navigate to home position (left boundary, center y) |

## Cup Buttons

| Button     | Action                                     |
| ---------- | ------------------------------------------ |
| LIGHT BLUE | Navigate to light blue cup (live tracking) |
| PURPLE     | Navigate to purple cup (live tracking)     |
| GREEN      | Navigate to green cup (live tracking)      |

## Camera View

| Action                       | Result                       |
| ---------------------------- | ---------------------------- |
| Click on tarp                | Add waypoint to path queue   |
| Click cup button             | Start live tracking that cup |
| Click on tarp while tracking | Stop tracking, add to queue  |

---

## Cup HSV Tuning

Run color tuner to find HSV values for each cup:

```bash
python3 controller/color_tuner.py
```

Update values in `controller/position_tracker.py` under `CUP_COLORS`.

---

## If something is stuck on a port

```bash
lsof -i :1883   # broker port
lsof -i :9999   # video port
kill -9 <PID>
```

## If GPIO is busy on the Pi

```bash
sudo pkill -f car_loop.py
sudo pkill -f video_streamer.py
```

## If git repo is corrupted on Pi

```bash
cd ~
rm -rf ~/teamProject/RaspberryPI-fnk0043-Freenove
git clone https://github.com/NazimFirdousAli/RaspberryPI-fnk0043-Freenove.git ~/teamProject/RaspberryPI-fnk0043-Freenove
cd ~/teamProject/RaspberryPI-fnk0043-Freenove
git checkout location-mapping
pip install paho-mqtt --break-system-packages
```
