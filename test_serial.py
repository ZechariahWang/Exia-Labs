import serial, struct, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
ser.reset_input_buffer()

buf = bytearray()
packet_counts = {}

print("Decoding TM171 packets for 5 seconds...\n")
start = time.time()

while time.time() - start < 5:
    n = ser.in_waiting
    if n > 0:
        buf.extend(ser.read(n))
    else:
        buf.extend(ser.read(1))

    while len(buf) >= 5:
        idx = buf.find(b'\xAA\x55')
        if idx == -1:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]

        if len(buf) < 3:
            break

        payload_len = buf[2]
        packet_total = 2 + 1 + payload_len + 2
        if len(buf) < packet_total:
            break

        packet = buf[:packet_total]
        del buf[:packet_total]

        ptype = payload_len
        packet_counts[ptype] = packet_counts.get(ptype, 0) + 1

        if packet_counts[ptype] <= 3:
            payload = packet[3:3 + payload_len]
            print(f"--- Packet len=0x{ptype:02X} ({payload_len} bytes payload) ---")
            print(f"  Raw: {' '.join(f'{b:02X}' for b in packet)}")

            num_floats = payload_len // 4
            floats = []
            for i in range(num_floats):
                f = struct.unpack('<f', payload[i*4:(i+1)*4])[0]
                floats.append(f)
            print(f"  As float32 ({num_floats} values):")
            for i, f in enumerate(floats):
                print(f"    [{i}] = {f:12.6f}")

            remainder = payload_len % 4
            if remainder:
                extra = payload[num_floats*4:]
                print(f"  Remaining bytes: {' '.join(f'{b:02X}' for b in extra)}")

            cksum = packet[-2:]
            print(f"  Checksum bytes: {cksum[0]:02X} {cksum[1]:02X}")
            print()

ser.close()

print("\n=== Packet Summary ===")
for ptype, count in sorted(packet_counts.items()):
    print(f"  len=0x{ptype:02X} ({ptype} bytes): {count} packets")

total = sum(packet_counts.values())
elapsed = time.time() - start
print(f"\nTotal: {total} packets in {elapsed:.1f}s ({total/elapsed:.1f} Hz)")
