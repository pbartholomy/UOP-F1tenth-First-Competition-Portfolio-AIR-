import struct
import time
import serial

VESC_PORT = "/dev/ttyACM0"
# If this does not work, change to:
# VESC_PORT = "/dev/ttyACM0"

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


def send_servo_once(ser, position):
    value = int(position * 1000)
    payload = bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", value)
    send_packet(ser, payload)


def hold_servo(ser, position, seconds):
    print(f"Holding servo at {position:.2f}")
    end_time = time.time() + seconds

    while time.time() < end_time:
        send_current_zero(ser)
        send_servo_once(ser, position)
        time.sleep(0.05)


print("JETSON SERVO HOLD TEST")
print(f"Opening {VESC_PORT}...")

with serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.1, write_timeout=0.1) as ser:
    time.sleep(1.0)

    hold_servo(ser, SERVO_CENTER, 2.0)
    hold_servo(ser, SERVO_LEFT, 2.0)
    hold_servo(ser, SERVO_RIGHT, 2.0)
    hold_servo(ser, SERVO_CENTER, 2.0)

print("Done.")
