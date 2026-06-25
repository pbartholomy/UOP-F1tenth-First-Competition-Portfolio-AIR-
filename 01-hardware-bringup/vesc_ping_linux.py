import struct
import time
import serial

VESC_PORT = "/dev/vesc"
VESC_BAUDRATE = 115200

COMM_FW_VERSION = 0


def crc16(data):
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_packet(payload):
    crc = crc16(payload)
    return (
        bytes([0x02, len(payload)])
        + payload
        + bytes([crc >> 8, crc & 0xFF, 0x03])
    )


print(f"Opening {VESC_PORT}...")

with serial.Serial(
    VESC_PORT,
    VESC_BAUDRATE,
    timeout=0.5,
    write_timeout=0.5
) as ser:
    time.sleep(2.0)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    packet = build_packet(bytes([COMM_FW_VERSION]))

    print("Sending FW_VERSION request...")
    print("TX:", packet.hex(" "))

    written = ser.write(packet)
    ser.flush()

    print("Bytes written:", written)

    time.sleep(0.5)

    data = ser.read(100)

    print("RX length:", len(data))
    print("RX:", data.hex(" "))

print("Done.")
