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
    STALL_VALUE = 0x15
    HOMING_METHOD = 0x16
    HOMING_SENSOR_TRIGGER_VALUE = 0x17
    REQUEST_HOMING = 0x18
    HOMED = 0x19


class OpMode:
    POSITION = 0
    VELOCITY = 1
    INVERSE_TIME = 2


class HomingMethod:
    IMMEDIATE = 0
    TORQUE = 1
    SENSOR = 2


class HomingTriggerValue:
    HIGH = 1
    LOW = 0


class ESP32_TMC2240_API:
    WAIT_TIME = 0.05

    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=1, dsrdtr=None)
        # Prevents the ESP32 from resetting
        self.ser.setRTS(False)
        self.ser.setDTR(False)

    @staticmethod
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

        return message

    def send_serial(self, message) -> str:
        # Send the message and read response
        self.ser.write(message)  # Send the message

        # Wait
        time.sleep(ESP32_TMC2240_API.WAIT_TIME)

        # Read response from Arduino
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)  # Read all available data
            decoded_response = response.decode("utf-8")  # Decode the byte data to string using UTF-8
            # print(decoded_response)  # Print the decoded string

            return decoded_response

    def clear_serial_buffer(self):
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)  # Read all available data
            decoded_response = response.decode("utf-8")  # Decode the byte data to string using UTF-8
            print(decoded_response)  # Print the decoded string

    def read_serial(self):
        while True:
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)  # Read all available data
                decoded_response = response.decode("utf-8")  # Decode the byte data to string using UTF-8
                print(decoded_response)  # Print the decoded string

    def read(self, register: Register, stepper_id: int = 0x00):
        message = ESP32_TMC2240_API.construct_serial_message(Instruction.READ, register, stepper_id=stepper_id)
        return self.send_serial(message)

    def write(self, register: Register, value: int = 0, stepper_id: int = 0x00):
        message = ESP32_TMC2240_API.construct_serial_message(Instruction.WRITE, register, value, stepper_id=stepper_id)
        return self.send_serial(message)
