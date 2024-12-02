import serial
import time
import struct
import binascii
import time


class Instruction:
    READ = 0x00
    WRITE = 0x01


class Register:
    TARGET_POSITION = 0x00
    TARGET_RPM = 0x01
    MOVE = 0x02
    DRV_STATUS = 0x04
    MOTOR_STATUS = 0x05
    EMERGENCY_STOP = 0x06
    STOP_VELOCITY = 0x07
    ENABLE_STEPPER = 0x08
    OPERATION_MODE = 0x09
    ACEL_TIME = 0x0A
    DECEL_TIME = 0x0B
    CURRENT_RPM = 0x0C
    CURRENT_POS = 0x0D
    ACTUAL_ACCELERATION_TIME = 0x0E
    ACTUAL_DECCELERATION_TIME = 0x0F
    STOP_ON_STALL = 0x10
    MICROSTEPPING = 0x11
    RUNNING_CURRENT = 0x12
    HOLDING_CURRENT_PERCENTAGE = 0x13
    DISABLE_STEPPER = 0x14


class OpMode:
    POSITION = 0
    VELOCITY = 1
    INVERSE_TIME = 2


# Initialize serial communication (change 'COM12' to your port and set baudrate)
ser = serial.Serial("COM22", 115200, timeout=1, dsrdtr=None)
ser.setRTS(False)
ser.setDTR(False)
time.sleep(1)  # Wait for serial to be ready


def construct_serial_message(
    instruction: Instruction,
    register: Register,
    command: int = 0,
    stepper_id: int = 0x00,
    device_id: int = 0x01,
) -> bytearray:
    START_BYTE = 0xAA
    END_BYTE = 0x55

    # Prepare data for CRC calculation (1 for Device ID + 4 for Command)
    data = bytearray()
    data.extend(struct.pack(">i", command))  # Pack command as big-endian 4 bytes

    # Calculate CRC32
    crc = binascii.crc32(data) & 0xFFFFFFFF  # Ensure it is unsigned 32-bit

    # Construct the message
    message = bytearray()
    message.append(START_BYTE)  # Start byte
    message.append(device_id)  # Device ID
    message.append(instruction)  # Device ID
    message.append(stepper_id)  # Stepper ID
    message.append(register)  # Stepper ID
    message.extend(data)  # Command data
    message.extend(struct.pack(">I", crc))  # Pack CRC32 as big-endian 4 bytes
    message.append(END_BYTE)  # End byte

    # print("Data:", data.hex())
    # print("Message to send:", message.hex())

    return message


def send_serial(message) -> str:
    global ser
    # Send the message and read response
    ser.write(message)  # Send the message

    time.sleep(0.05)
    # Read response from Arduino
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)  # Read all available data
        decoded_response = response.decode("utf-8")  # Decode the byte data to string using UTF-8
        # print(decoded_response)  # Print the decoded string

        return decoded_response


def clear_serial_buffer():
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)  # Read all available data
        decoded_response = response.decode("utf-8")  # Decode the byte data to string using UTF-8
        print(decoded_response)  # Print the decoded string


def read_serial():
    while True:
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)  # Read all available data
            decoded_response = response.decode("utf-8")  # Decode the byte data to string using UTF-8
            print(decoded_response)  # Print the decoded string


def read(register: Register):
    message = construct_serial_message(Instruction.READ, register)
    return send_serial(message)


def write(register: Register, value: int = 0):
    message = construct_serial_message(Instruction.WRITE, register, value)
    return send_serial(message)


def get_motor_status():
    return read(Register.MOTOR_STATUS)


def read_motor_status(time_s):
    time_stamp = time.time()
    while time.time() - time_stamp < time_s:
        read(Register.MOTOR_STATUS)


def read_current_rpm(time_s):
    time_stamp = time.time()
    while time.time() - time_stamp < time_s:
        read(Register.CURRENT_RPM)
