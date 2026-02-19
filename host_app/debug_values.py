import socket
import struct
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 9000
PACKET_FORMAT = "<I 3f 3f 3f 4f 3f 3f f f f f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(True) # Wir wollen einfach nur sehen OB was kommt

print("Warte auf ESP32 Daten...")

while True:
    try:
        data, addr = sock.recvfrom(2048)
        if len(data) == PACKET_SIZE:
            unpacked = struct.unpack(PACKET_FORMAT, data)
            # [ts, gx,gy,gz, ax,ay,az, mx,my,mz, qw,qx,qy,qz, ...]
            ts = unpacked[0]
            gyro = unpacked[1:4]
            acc = unpacked[4:7]
            quat = unpacked[10:14] # Quaternion
            
            print(f"TS: {ts} | Gyro: {gyro[0]:.2f} {gyro[1]:.2f} {gyro[2]:.2f} | Acc: {acc[0]:.2f} | Quat: {quat[0]:.2f} {quat[1]:.2f} {quat[2]:.2f} {quat[3]:.2f}", end="\r")
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(e)
