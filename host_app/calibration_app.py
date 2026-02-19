import sys
import os
import struct
import socket
import threading
import time
import json
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import select
from scipy.optimize import least_squares
import queue

# --- KONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 9000
PACKET_FORMAT = "<I 3f 3f 3f 4f 3f 3f f f f f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
CALIBRATION_FILE = "calibration.json"

class IMUCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Sim2Real IMU Kalibrierung (MPU6050 + QMC5883P)")
        self.root.geometry("1100x850")
        self.style = ttk.Style(); self.style.theme_use('clam')
        
        # Thread-Safe Communication
        self.gui_queue = queue.Queue(maxsize=1) # Für die Anzeige (Last-is-Best)
        self.data_lock = threading.Lock() # Für die Aufzeichnung
        
        self.raw_data = { 'gyro': [], 'acc': [], 'mag': [] }
        
        self.calibration = {
            'gyro_bias': [0.0, 0.0, 0.0],
            'acc_bias': [0.0, 0.0, 0.0],
            'acc_scale': [1.0, 1.0, 1.0],
            'mag_offset': [0.0, 0.0, 0.0],
            'mag_scale': [1.0, 1.0, 1.0]
        }
        
        self.collecting = False
        self.collection_mode = None
        
        # Receiver starten
        self.running = True
        self.thread = threading.Thread(target=self.udp_thread_loop, daemon=True)
        self.thread.start()

        self.setup_gui()
        self.update_gui_loop()

    def udp_thread_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # FIX
        try:
            sock.bind((UDP_IP, UDP_PORT))
        except Exception as e:
            print(f"Socket Error: {e}")
            return
            
        print("Calibration Receiver Started")
        
        while self.running:
            try:
                data, _ = sock.recvfrom(2048) # Blocking Read
                if len(data) == PACKET_SIZE:
                    # 1. Decode
                    unpacked = struct.unpack(PACKET_FORMAT, data)
                    gyro = list(unpacked[1:4])
                    acc = list(unpacked[4:7])
                    mag = list(unpacked[7:10])
                    
                    # 2. Record (High Freq)
                    if self.collecting:
                        with self.data_lock:
                            if self.collection_mode == 'gyro': self.raw_data['gyro'].append(gyro)
                            elif self.collection_mode == 'mag': self.raw_data['mag'].append(mag)
                            elif self.collection_mode == 'acc': self.raw_data['acc'].append(acc)
                    
                    # 3. Update GUI (Low Freq via Queue Drop)
                    try:
                        self.gui_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.gui_queue.put((gyro, acc, mag))
                    
            except Exception as e:
                print(e)
                time.sleep(0.1)
        sock.close()

    def update_gui_loop(self):
        try:
            # Hole neustes Paket
            g, a, m = self.gui_queue.get_nowait()
            self.lbl_telemetry.config(text=f"G: {g[0]:6.2f} {g[1]:6.2f} {g[2]:6.2f}\n"
                                           f"A: {a[0]:6.2f} {a[1]:6.2f} {a[2]:6.2f}\n"
                                           f"M: {m[0]:6.1f} {m[1]:6.1f} {m[2]:6.1f}")
            
            # Live Plot (Mag) - Nur jedes 5. Update zeichnen um GUI zu schonen
            if self.collection_mode == 'mag':
                self.plot_mag_live_optimized(m)
                
        except queue.Empty:
            pass
        
        self.root.after(30, self.update_gui_loop)

    def plot_mag_live_optimized(self, latest_mag):
        # Performance Hack: Nicht scatter() nutzen, sondern set_data() wenn möglich
        # Oder einfach nur jeden N-ten Punkt plotten.
        # Hier vereinfacht: Wir sammeln Punkte in einem Cache und plotten selten neu
        # (Tkinter Canvas ist langsam)
        pass # Live Plotting in Matplotlib in Tkinter ist der Lag-Killer #1. Wir lassen es minimal.

    def setup_gui(self):
        # Header
        header_frame = ttk.Frame(self.root, padding="10")
        header_frame.pack(fill=tk.X)
        ttk.Label(header_frame, text="Sim2Real Sensor Calibration", font=("Helvetica", 16, "bold")).pack(side=tk.LEFT)
        
        # Main Layout
        self.main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.left_frame = ttk.Frame(self.main_pane, padding="10", width=400)
        self.main_pane.add(self.left_frame, weight=1)
        
        self.right_frame = ttk.Frame(self.main_pane, padding="10")
        self.main_pane.add(self.right_frame, weight=3)
        
        self.lbl_step = ttk.Label(self.left_frame, text="Schritt 1: Initialisierung", font=("Helvetica", 12, "bold"))
        self.lbl_step.pack(pady=(0, 10), anchor="w")
        
        self.txt_instruction = tk.Text(self.left_frame, height=15, width=40, wrap=tk.WORD, font=("Arial", 10))
        self.txt_instruction.pack(fill=tk.X, pady=10)
        self.txt_instruction.insert(tk.END, "Willkommen.\n")
        self.txt_instruction.config(state=tk.DISABLED)
        
        self.progress = ttk.Progressbar(self.left_frame, mode='determinate')
        self.progress.pack(fill=tk.X, pady=20)
        
        self.lbl_telemetry = ttk.Label(self.left_frame, text="Warte auf Daten...", font=("Courier", 9))
        self.lbl_telemetry.pack(pady=10, anchor="w")
        
        self.btn_next = ttk.Button(self.left_frame, text="Start: Gyro Kalibrierung", command=self.start_gyro_cal)
        self.btn_next.pack(fill=tk.X, pady=5)
        
        self.btn_skip = ttk.Button(self.left_frame, text="Diesen Schritt überspringen", command=self.next_phase)
        self.btn_skip.pack(fill=tk.X, pady=5)
        
        ttk.Separator(self.left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=20)
        ttk.Button(self.left_frame, text="Kalibrierung Speichern", command=self.save_calibration).pack(fill=tk.X, side=tk.BOTTOM)

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def set_instruction(self, text):
        self.txt_instruction.config(state=tk.NORMAL)
        self.txt_instruction.delete(1.0, tk.END)
        self.txt_instruction.insert(tk.END, text)
        self.txt_instruction.config(state=tk.DISABLED)

    # --- STEPS ---
    def start_gyro_cal(self):
        self.set_instruction("SCHRITT 1: GYRO BIAS\n\nLege den Roboter STILL hin.\n5 Sekunden Messung.")
        self.btn_next.config(text="Messung läuft...", state=tk.DISABLED)
        self.raw_data['gyro'] = []
        self.collection_mode = 'gyro'
        self.collecting = True
        self.progress.start(10)
        self.root.after(5000, self.finish_gyro_cal)

    def finish_gyro_cal(self):
        self.collecting = False
        self.progress.stop()
        self.collection_mode = None
        with self.data_lock:
            data = np.array(self.raw_data['gyro'])
        
        if len(data) > 0:
            bias = np.mean(data, axis=0)
            self.calibration['gyro_bias'] = bias.tolist()
            messagebox.showinfo("Gyro Result", f"Bias gefunden:\nX: {bias[0]:.4f}\nY: {bias[1]:.4f}\nZ: {bias[2]:.4f}")
        else:
            messagebox.showerror("Fehler", "Keine Daten empfangen!")
        
        self.btn_next.config(text="Weiter: Magnetometer", state=tk.NORMAL, command=self.start_mag_cal)
        self.lbl_step.config(text="Schritt 2: Magnetometer")

    def start_mag_cal(self):
        self.set_instruction("SCHRITT 2: MAG\n\nDrehe den Sensor um ALLE Achsen (Kugel).\nZiel: >1000 Punkte.")
        self.btn_next.config(text="Start Aufnahme", command=self.record_mag)
        self.ax.clear()

    def record_mag(self):
        self.raw_data['mag'] = []
        self.collection_mode = 'mag'
        self.collecting = True
        self.btn_next.config(text="Fertig (Berechnen)", command=self.finish_mag_cal)

    def finish_mag_cal(self):
        self.collecting = False
        self.collection_mode = None
        with self.data_lock:
            data = np.array(self.raw_data['mag'])
        
        if len(data) < 100:
            messagebox.showwarning("Zu wenig Daten", "Bitte mehr rotieren!")
            return
            
        # Sphere Fit (Simplified)
        min_v = np.min(data, axis=0)
        max_v = np.max(data, axis=0)
        offset = (max_v + min_v) / 2
        self.calibration['mag_offset'] = offset.tolist()
        self.calibration['mag_scale'] = [1.0, 1.0, 1.0] # Scale ist komplexer, wir nehmen erst mal Offset
        
        # Visualisierung Resultat
        self.ax.clear()
        self.ax.scatter(data[:,0], data[:,1], data[:,2], s=1, c='r')
        self.canvas.draw()
        
        self.btn_next.config(text="Speichern", command=self.save_calibration)
        self.lbl_step.config(text="Schritt 3: Speichern")

    def next_phase(self):
        if self.lbl_step.cget("text").startswith("Schritt 1"):
            self.finish_gyro_cal()
        elif self.lbl_step.cget("text").startswith("Schritt 2"):
            self.btn_next.config(text="Speichern", command=self.save_calibration)
            self.lbl_step.config(text="Schritt 3: Speichern")

    def save_calibration(self):
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(self.calibration, f, indent=4)
            
        header_path = "calibration_values.h"
        fw_path = os.path.join(os.path.dirname(__file__), "..", "firmware_complete", "calibration_values.h")
        if os.path.exists(os.path.dirname(fw_path)): header_path = fw_path
            
        with open(header_path, "w") as f:
            f.write("#ifndef CALIBRATION_VALUES_H\n#define CALIBRATION_VALUES_H\n\n")
            gb = self.calibration['gyro_bias']
            f.write(f"const float GYRO_BIAS[] = {{ {gb[0]:.6f}f, {gb[1]:.6f}f, {gb[2]:.6f}f }};\n")
            mo = self.calibration['mag_offset']
            f.write(f"const float MAG_OFFSET[] = {{ {mo[0]:.6f}f, {mo[1]:.6f}f, {mo[2]:.6f}f }};\n")
            ms = self.calibration['mag_scale']
            f.write(f"const float MAG_SCALE[] = {{ {ms[0]:.6f}f, {ms[1]:.6f}f, {ms[2]:.6f}f }};\n")
            f.write("\n#endif\n")
            
        messagebox.showinfo("Gespeichert", 
                            f"Kalibrierung erfolgreich!\n\n" 
                            f"1. JSON: {CALIBRATION_FILE}\n" 
                            f"2. Header: {header_path}\n\n"
                            f"BITTE FIRMWARE NEU KOMPILIEREN, UM DIE WERTE ZU NUTZEN!")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = IMUCalibrationApp(root)
    root.mainloop()
