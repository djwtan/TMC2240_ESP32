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


class OperationMode:
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


# ======================================= TEST ======================================= #
if __name__ == "__main__":
    import threading
    from openpyxl import Workbook

    # read(Register.DRV_STATUS)
    # read(Register.OPERATION_MODE)
    # read(Register.ACEL_TIME)
    # read(Register.DECEL_TIME)
    # read(Register.MOTOR_STATUS)
    # read(Register.CURRENT_RPM)
    # read(Register.CURRENT_POS)
    # read(Register.ACTUAL_ACCELERATION_TIME)
    # read(Register.ACTUAL_DECCELERATION_TIME)
    # read_motor_status(2)
    # write(Register.ENABLE_STEPPER)

    # threading.Thread(target=read_motor_status, args=(10,)).start()
    # write(Register.ENABLE_STEPPER)
    # write(Register.OPERATION_MODE, OperationMode.VELOCITY)
    # ------------------------------------------------------------------------------------ #
    wb = Workbook()

    ws = wb.active
    ws.title = "Sheet1"
    ws.append(["rpm", "acel", "acel_a", "decel", "decel_a"])

    # ------------------------------------------------------------------------------------ #
    write(Register.ACEL_TIME, int(4 * 1000))
    write(Register.DECEL_TIME, int(4 * 1000))

    rpm = 400
    while rpm <= 3000:
        print("run {}".format(rpm))
        write(Register.TARGET_POSITION, round(30 * 200 if rpm < 500 else 70 * 200))
        write(Register.TARGET_RPM, rpm)

        write(Register.MOVE)

        motor_status = None
        while motor_status != "idle":
            motor_status = get_motor_status()
            # print(motor_status)

        acel_actual = read(Register.ACTUAL_ACCELERATION_TIME)
        decel_actual = read(Register.ACTUAL_DECCELERATION_TIME)

        ws.append([rpm, 4000000, acel_actual, 4000000, decel_actual])
        print("({}) Acel: {}/4000000   Decel: {}/4000000".format(rpm, acel_actual, decel_actual))

        rpm += 50

    file_name = "rpm_acel_decel_time_corrected_2.xlsx"
    wb.save(file_name)
    # ------------------------------------------------------------------------------------ #

    # clear_serial_buffer()
    # read_serial()
    # write(Register.EMERGENCY_STOP)
    # write(Register.STOP_VELOCITY)
    # clear_serial_buffer()
    # read_current_rpm(10)
    #
    # write(Register.ENABLE_STEPPER)

    # while True:
    #     # read(Register.DRV_STATUS)
    #     read(Register.CURRENT_RPM)
    #     # read(Register.MOTOR_STATUS)
    #     time.sleep(0.1)
