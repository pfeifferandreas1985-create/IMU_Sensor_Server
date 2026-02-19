import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Waiting for packet...")
data, _ = sock.recvfrom(2048)
print(f"Received: {len(data)} bytes")

PACKET_FORMAT = "<I 3f 3f 3f 4f 3f 3f f f f f"
expected = struct.calcsize(PACKET_FORMAT)
print(f"Expected: {expected} bytes")

if len(data) != expected:
    print("MISMATCH! Check firmware struct definition.")
else:
    print("MATCH! Protocol is fine.")