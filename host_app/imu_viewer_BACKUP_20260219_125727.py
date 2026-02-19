import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import threading
import sys
import select

# UDP Configuration
UDP_IP = "0.0.0.0" 
UDP_PORT = 9000

# Packet Structure
PACKET_FORMAT = "<I 3f 3f 3f 4f 3f 3f f f f f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

class IMUReceiver:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Increase Receive Buffer (SO_RCVBUF) to prevent drops, but we will drain it anyway
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        try:
            self.sock.bind((UDP_IP, UDP_PORT))
        except Exception as e:
            print(f"Port Error: {e}")
            sys.exit(1)
        self.sock.settimeout(0.001) # Non-blocking-ish
        self.data = None
        self.running = True
        
    def receive(self):
        print(f"Listening for ESP32 on UDP {UDP_PORT}...")
        while self.running:
            # Drain the buffer! We only want the LATEST packet.
            # Reading 500Hz packets at 50Hz display rate means we must discard 9 packets.
            last_valid_data = None
            
            while True:
                ready = select.select([self.sock], [], [], 0.005) # Check if data available
                if ready[0]:
                    try:
                        data, addr = self.sock.recvfrom(1024)
                        if len(data) == PACKET_SIZE:
                            last_valid_data = data
                    except:
                        break
                else:
                    break # Buffer empty
            
            if last_valid_data:
                unpacked = struct.unpack(PACKET_FORMAT, last_valid_data)
                self.data = unpacked
            
            # Sleep slightly to not burn 100% CPU in this thread loop
            # But short enough to keep draining
            # Actually, `select` with timeout handles the pacing.

    def stop(self):
        self.running = False
        self.sock.close()

receiver = IMUReceiver()
recv_thread = threading.Thread(target=receiver.receive, daemon=True)
recv_thread.start()

# Visualization Setup
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.canvas.manager.set_window_title("Hexapod IMU Live - LOW LATENCY MODE")

def update(frame):
    if receiver.data is None:
        return
    
    # Extract Quaternion
    q = receiver.data[10:14]
    try:
        rot = R.from_quat([q[1], q[2], q[3], q[0]]) # [x, y, z, w]
    except:
        return
    
    # Create coordinate axes
    x_axis = rot.apply([1, 0, 0])
    y_axis = rot.apply([0, 1, 0])
    z_axis = rot.apply([0, 0, 1])
    
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Vectors
    ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='r', linewidth=2, label='X (Front)')
    ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='g', linewidth=2, label='Y (Left)')
    ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='b', linewidth=2, label='Z (Up)')
    
    # Stats
    mag_x = receiver.data[7]
    mag_y = receiver.data[8]
    mag_z = receiver.data[9]
    temp = receiver.data[21]
    yaw = receiver.data[23]
    
    ax.text2D(0.05, 0.95, f"Yaw: {yaw:.2f}°\nTemp: {temp:.1f}°C\nMag: {mag_x:.0f}/{mag_y:.0f}/{mag_z:.0f}", transform=ax.transAxes)
    ax.legend()

# Interval 50ms = 20 FPS (Smooth enough, low burden)
ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()

receiver.stop()
