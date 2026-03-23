import socket
import struct
import threading
import numpy as np
import cv2
import pygame

PORT = 9999

class VideoReceiver:
    def __init__(self, host: str):
        self.host = host
        self.frame_surface = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()
        print(f"[video] Connecting to {self.host}:{PORT}")

    def stop(self):
        self.running = False

    def _receive(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self.host, PORT))
            print("[video] Connected to stream")

            while self.running:
                # Read frame length first
                raw_len = self._recv_exact(sock, 4)
                if not raw_len:
                    break
                frame_len = struct.unpack('<I', raw_len)[0]

                # Read the frame
                raw_frame = self._recv_exact(sock, frame_len)
                if not raw_frame:
                    break

                # Decode JPEG to numpy array then to pygame surface
                np_arr = np.frombuffer(raw_frame, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # Convert BGR (OpenCV) to RGB (pygame)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                surface = pygame.surfarray.make_surface(np.transpose(frame, (1, 0, 2)))

                with self.lock:
                    self.frame_surface = surface

        except Exception as e:
            print(f"[video] Receiver error: {e}")
        finally:
            sock.close()

    def _recv_exact(self, sock, n):
        data = b""
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def get_surface(self):
        with self.lock:
            return self.frame_surface