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
        # Puffer klein halten
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        # Reuse erlauben für schnellen Neustart
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((UDP_IP, UDP_PORT))
        except Exception as e:
            print(f"Port Error: {e}")
            sys.exit(1)
        self.running = True
        print(f"[Receiver] Listening on {UDP_PORT}...")

    def run(self):
        print(f"[Receiver] Started. Packet Size: {PACKET_SIZE}")
        while self.running:
            try:
                # Blockierendes Lesen (Effizient im Thread)
                data, _ = self.sock.recvfrom(2048)
                
                if len(data) == PACKET_SIZE:
                    # Queue leeren (Drop Oldest)
                    try:
                        data_queue.get_nowait()
                    except queue.Empty:
                        pass
                    # Neues Paket rein
                    data_queue.put(data)
                else:
                    # Fehler nur senden wenn Queue leer (Anti-Spam)
                    if data_queue.empty():
                        data_queue.put(f"Status: ERROR Size {len(data)} != {PACKET_SIZE}")
            
            except Exception as e:
                # Socket Fehler
                time.sleep(0.1)

    def stop(self):
        self.running = False
        self.sock.close()

# Start Receiver
receiver = UDPReceiver()
receiver.start()

# --- ARCHITEKTUR: GUI (High Performance Mode) ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.canvas.manager.set_window_title("Hexapod IMU - High Performance")

# 1. Statisches Setup (Wird nur EINMAL ausgeführt)
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Linien initialisieren (X=Rot, Y=Grün, Z=Blau)
line_x, = ax.plot([0, 1], [0, 0], [0, 0], 'r-', lw=4, label='X (Front)')
line_y, = ax.plot([0, 0], [0, 1], [0, 0], 'g-', lw=4, label='Y (Left)')
line_z, = ax.plot([0, 0], [0, 0], [0, 1], 'b-', lw=4, label='Z (Up)')

# Status Text initialisieren
status_text = ax.text2D(0.05, 0.95, "Warte auf Daten...", transform=ax.transAxes, color='black', fontsize=12, backgroundcolor='white')

ax.legend(loc='upper right')

# Cache Variablen
current_data_unpacked = None
last_status_str = "Init..."

def update(frame):
    global current_data_unpacked, last_status_str
    
    # 2. Daten holen (Schnell)
    try:
        # Wir verarbeiten alles was in der Queue ist (normalerweise nur 1 Item)
        while not data_queue.empty():
            item = data_queue.get_nowait()
            if isinstance(item, str):
                last_status_str = item
            else:
                current_data_unpacked = struct.unpack(PACKET_FORMAT, item)
                last_status_str = f"Status: OK | TS: {current_data_unpacked[0]}"
    except queue.Empty:
        pass

    # 3. Status Text Update (Sehr schnell)
    # Heartbeat Check
    if not hasattr(update, "last_packet_ts"): update.last_packet_ts = time.time()
    
    if current_data_unpacked:
        update.last_packet_ts = time.time()
        status_text.set_text(last_status_str)
        status_text.set_color('black')
        status_text.set_backgroundcolor('white')
    elif time.time() - update.last_packet_ts > 0.5:
        status_text.set_text("NO SIGNAL / LAG")
        status_text.set_color('white')
        status_text.set_backgroundcolor('red')

    if current_data_unpacked is None:
        return

    # 4. Berechnung
    q = current_data_unpacked[10:14] # Quaternion
    
    # SENSOR FREEZE CHECK
    if not hasattr(update, "last_q"): update.last_q = q
    # Wenn TS neu ist, aber Q exakt gleich -> Sensor sendet alte Werte
    if q == update.last_q and frame > 10: 
         status_text.set_text(last_status_str + " [SENSOR STUCK]")
         status_text.set_backgroundcolor('orange')
    update.last_q = q

    try:
        rot = R.from_quat([q[1], q[2], q[3], q[0]]) 
    except ValueError:
        return

    # Achsen rotieren
    x_vec = rot.apply([1, 0, 0])
    y_vec = rot.apply([0, 1, 0])
    z_vec = rot.apply([0, 0, 1])
    
    # 5. Grafik Update (Nur Daten ändern, KEIN Neuzeichnen der Szene!)
    # X-Achse
    line_x.set_data([0, x_vec[0]], [0, x_vec[1]])
    line_x.set_3d_properties([0, x_vec[2]])
    
    # Y-Achse
    line_y.set_data([0, y_vec[0]], [0, y_vec[1]])
    line_y.set_3d_properties([0, y_vec[2]])
    
    # Z-Achse
    line_z.set_data([0, z_vec[0]], [0, z_vec[1]])
    line_z.set_3d_properties([0, z_vec[2]])
    
    # FORCE REDRAW
    # Hilft gegen "Einfrieren" des Bildes trotz laufendem Loop
    # fig.canvas.draw_idle()  <-- Animation macht das normalerweise
    # Wir probieren es ohne expliziten Draw, da blit=False das eh macht.
    
    # Debug: Wenn wir hier ankommen, lief das Update durch.

# Animation starten
# 40ms = 25 FPS. Da update() jetzt extrem effizient ist, langweilt sich die CPU.
ani = FuncAnimation(fig, update, interval=40, blit=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    receiver.stop()
