import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import threading
import sys
import queue
import time

# --- KONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 9000
PACKET_FORMAT = "<I 3f 3f 3f 4f 3f 3f f f f f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# --- ARCHITEKTUR: DATENPUFFER ---
data_queue = queue.Queue(maxsize=1)

# --- ARCHITEKTUR: RECEIVER THREAD ---
class UDPReceiver(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((UDP_IP, UDP_PORT))
        except Exception as e:
            print(f"Port Error: {e}")
            sys.exit(1)
        self.running = True
        print(f"[Receiver] Listening on {UDP_PORT}...")

    def run(self):
        print(f"[Receiver] Thread Started. Packet Size Expected: {PACKET_SIZE}")
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
                
                if len(data) == PACKET_SIZE:
                    try:
                        data_queue.get_nowait()
                    except queue.Empty:
                        pass
                    data_queue.put(data)
                else:
                    msg = f"Status: ERROR Size {len(data)} != {PACKET_SIZE}"
                    if data_queue.empty(): 
                        data_queue.put(msg)
            
            except Exception as e:
                if data_queue.empty():
                    data_queue.put(f"Status: Socket Error {e}")
                time.sleep(0.1)

    def stop(self):
        self.running = False
        self.sock.close()

# Start Receiver
receiver = UDPReceiver()
receiver.start()

# --- ARCHITEKTUR: GUI CONSUMER ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.canvas.manager.set_window_title("Hexapod IMU - Stable")

# Dummy-Startwert
current_data_unpacked = (0, 0.,0.,0., 0.,0.,0., 0.,0.,0., 1.,0.,0.,0., 0.,0.,0., 0., 0., 0., 0.)
last_status = "Status: Init..."

def update(frame):
    global current_data_unpacked, last_status
    
    try:
        # 1. Daten holen (Queue drain)
        try:
            while not data_queue.empty():
                item = data_queue.get_nowait()
                if isinstance(item, str):
                    last_status = item
                else:
                    current_data_unpacked = struct.unpack(PACKET_FORMAT, item)
                    last_status = f"Status: OK (TS: {current_data_unpacked[0]})"
        except queue.Empty:
            pass

        if current_data_unpacked is None:
            return

        q = current_data_unpacked[10:14]
        ts = current_data_unpacked[0]
        
        try:
            rot = R.from_quat([q[1], q[2], q[3], q[0]]) 
        except ValueError:
            return

        x_axis = rot.apply([1, 0, 0])
        y_axis = rot.apply([0, 1, 0])
        z_axis = rot.apply([0, 0, 1])
        
        ax.cla()
        ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        
        ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='r', lw=2, label='X')
        ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='g', lw=2, label='Y')
        ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='b', lw=2, label='Z')
        
        # Text NACH cla() zeichnen
        ax.text2D(0.05, 0.95, last_status, transform=ax.transAxes, color='black', backgroundcolor='white')
        ax.legend(loc='upper right')
        
    except Exception as e:
        print(f"Frame Error: {e}")

# 60ms = ~16 FPS -> Sehr stabil
ani = FuncAnimation(fig, update, interval=60, blit=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    receiver.stop()
