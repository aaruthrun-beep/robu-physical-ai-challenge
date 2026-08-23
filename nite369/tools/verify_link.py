import socket
import struct
import time

# Pico Configuration
PICO_IP = "192.168.1.100"  # Matches IP in astra_master.c
PICO_PORT = 5000           # Matches PORT_UDP in astra_master.c

# Create socket and bind a local port to listen for responses
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0) # 1 second timeout

# astra_udp_cmd_t structure:
# - magic: 1 byte (0xAC)
# - sequence: 4 bytes (unsigned int)
# - target: 8 x 32-bit signed ints (32 bytes)
# - enable_bits: 1 byte
# - reserve: 2 bytes
# - crc: 1 byte
# Total size: 41 bytes
# Format string: "<B I 8i B 2s B"

def astra_crc8(data):
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

print(f"--- NITE 369 Connection Verifier ---")
print(f"Target Pico: {PICO_IP}:{PICO_PORT}")
print("Attempting to send packets and read telemetry feedback...")
print("Press Ctrl+C to exit.\n")

try:
    sequence = 0
    while True:
        # Dummy targets for 8 axes
        targets = [100, 200, 300, 400, 500, 600, 700, 800]
        enable_bits = 0x01
        reserve = b'\x00\x00'
        magic = 0xAC # ASTRA_MAGIC
        
        # Build payload without CRC first
        header = struct.pack("<BI8iB2s", magic, sequence, *targets, enable_bits, reserve)
        crc = astra_crc8(header)
        payload = header + struct.pack("B", crc)

        # Send command
        sock.sendto(payload, (PICO_IP, PICO_PORT))
        
        try:
            # Wait for telemetry packet (55 bytes)
            data, addr = sock.recvfrom(1024)
            
            # Unpack astra_udp_telemetry_t
            # - magic: 1 byte
            # - sequence_ack: 4 bytes
            # - actual: 8 x 32-bit signed ints (32 bytes)
            # - current_ma: 8 x 16-bit unsigned ints (16 bytes)
            # - status_flags: 1 byte
            # - crc: 1 byte
            if len(data) == 55:
                magic_rx, seq_ack, *actual, status_flags, crc_rx = struct.unpack("<BI8i8HB", data)
                print(f"[Success] Received packet #{seq_ack} from {addr[0]}")
                print(f"  └─ Axis Positions: {actual[0:6]}")
                print(f"  └─ Status Flags : {bin(status_flags)}")
            else:
                print(f"[Warning] Received packet with invalid size ({len(data)} bytes)")
                
        except socket.timeout:
            print("[Timeout] No response from Pico. Is it turned on, wired correctly, and on IP 192.168.1.100?")

        sequence += 1
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nVerification stopped.")
finally:
    sock.close()
