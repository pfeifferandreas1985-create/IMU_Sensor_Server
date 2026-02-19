import serial
import time

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    # Reset ESP32
    ser.dtr = False
    ser.rts = False
    time.sleep(0.1)
    ser.dtr = False
    ser.rts = True  # Pull RTS low to reset
    time.sleep(0.1)
    ser.rts = False
    
    print("Listening on /dev/ttyUSB0 (Resetting board)...")
    
    start = time.time()
    while time.time() - start < 10: # Listen for 10 seconds
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='ignore').strip())
except Exception as e:
    print(e)
