import socket
import struct
import threading
from camera import Camera

PORT = 9999

class VideoStreamer:
    def __init__(self):
        self.camera = Camera(stream_size=(400, 300))
        self.running = False
        self.server_thread = None

    def start(self):
        self.running = True
        self.server_thread = threading.Thread(target=self._serve, daemon=True)
        self.server_thread.start()
        print(f"[video] Streaming server started on port {PORT}")

    def stop(self):
        self.running = False
        self.camera.stop_stream()
        self.camera.close()
        print("[video] Streaming server stopped")

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", PORT))
        server.listen(1)
        server.settimeout(1.0)

        self.camera.start_stream()

        while self.running:
            try:
                conn, addr = server.accept()
                print(f"[video] Client connected: {addr}")
                self._stream_to(conn)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[video] Server error: {e}")

        server.close()

    def _stream_to(self, conn):
        try:
            while self.running:
                frame = self.camera.get_frame()
                if frame is None:
                    continue
                # Send frame length first, then the frame
                length = struct.pack('<I', len(frame))
                conn.sendall(length)
                conn.sendall(frame)
        except Exception as e:
            print(f"[video] Client disconnected: {e}")
        finally:
            conn.close()

if __name__ == "__main__":
    import time
    streamer = VideoStreamer()
    streamer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        streamer.stop()