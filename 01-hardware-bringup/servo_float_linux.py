import struct
import time
import serial

VESC_PORT = "/dev/vesc"
VESC_BAUDRATE = 115200

COMM_SET_CURRENT = 6
COMM_SET_SERVO_POS = 12

SERVO_CENTER = 0.50
SERVO_LEFT = 0.15
SERVO_RIGHT = 0.85


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


def send_packet(ser, payload):
    packet = build_packet(payload)
    ser.write(packet)
    ser.flush()


def send_current_zero(ser):
    payload = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
    send_packet(ser, payload)


def send_servo_float(ser, position):
    # Try VESC float32 servo payload: command + big-endian float
    payload = bytes([COMM_SET_SERVO_POS]) + struct.pack(">f", float(position))
    send_packet(ser, payload)
    print(f"Sent FLOAT servo position: {position:.2f}")


def hold_servo(ser, position, seconds):
    end = time.time() + seconds
    while time.time() < end:
        send_current_zero(ser)
        send_servo_float(ser, position)
        time.sleep(0.05)


print("JETSON SERVO FLOAT TEST")
print(f"Opening {VESC_PORT}...")

with serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.1, write_timeout=0.1) as ser:
    time.sleep(1.0)

    print("Center")
    hold_servo(ser, SERVO_CENTER, 2.0)

    print("Left")
    hold_servo(ser, SERVO_LEFT, 2.0)

    print("Right")
    hold_servo(ser, SERVO_RIGHT, 2.0)

    print("Center")
    hold_servo(ser, SERVO_CENTER, 2.0)

print("Done.")
