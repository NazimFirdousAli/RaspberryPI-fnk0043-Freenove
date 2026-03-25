# Freenove RC Car — Startup Steps

## 1. Broker (Laptop — Terminal 1)
```bash
mosquitto -c controller/mosquitto.conf -v
```

## 2. Video Streamer (Pi — Terminal 1)
```bash
cd ~/teamProject/RaspberryPI-fnk0043-Freenove
python3 video_streamer.py
```

## 3. Car Loop (Pi — Terminal 2)
```bash
cd ~/teamProject/RaspberryPI-fnk0043-Freenove
python3 car_loop.py leader 10.56.45.88
```

## 4. Manual Controller (Laptop — Terminal 2)
```bash
cd path/to/RaspberryPI-fnk0043-Freenove
source venv/bin/activate
python3 controller/manual_controller.py localhost 10.56.45.66
```

---

## Key Info
| Device | IP |
|--------|----|
| Laptop (broker) | 10.56.45.88 |
| Pi (car) | 10.56.45.66 |
| Video port | 9999 |
| MQTT port | 1883 |

---

## Controls
| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| A | Rotate left |
| D | Rotate right |
| Q | Strafe left |
| E | Strafe right |
| Arrow keys | Pan/tilt servo |
| SPACE | Horn |
| TAB | Switch target car |
| ESC | Quit |

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
