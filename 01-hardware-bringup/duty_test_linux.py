import struct
import time
import serial

VESC_PORT = "/dev/vesc"
VESC_BAUDRATE = 115200

COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6

TEST_DUTY = 0.04   # very low, 4% duty


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
    ser.write(build_packet(payload))
    ser.flush()


def send_duty(ser, duty):
    value = int(duty * 100000)
    payload = bytes([COMM_SET_DUTY]) + struct.pack(">i", value)
    send_packet(ser, payload)
    print(f"Sent duty: {duty:+.3f}")


def stop_motor(ser):
    payload = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
    send_packet(ser, payload)
    print("Stopped motor")


print("JETSON DUTY MOTOR TEST")
print("PUT CAR ON A STAND FIRST")
print(f"Opening {VESC_PORT}...")

with serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.1, write_timeout=0.1) as ser:
    time.sleep(1.0)

    send_duty(ser, TEST_DUTY)
    time.sleep(1.5)

    stop_motor(ser)
    time.sleep(0.5)

print("Done.")
