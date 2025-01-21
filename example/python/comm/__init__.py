import serial
import time
import struct
import binascii
import threading
import queue
from typing import Optional, List

from .define import *  # Ensure proper imports for constants like START_BYTE, END_BYTE, etc.


class ESP32_TMC2240_API:
    """
    API class for interfacing with an ESP32 controlling a TMC2240 stepper driver.
    Provides methods for sending and receiving data over a serial connection.
    """

    WAIT_TIME = 0.008  # Delay between commands to ensure the ESP32 processes data correctly

    def __init__(self, comm_port: serial.Serial, lock: threading.Lock, device_id: int):
        """
        Initialize the API instance.

        :param comm_port: The serial port for communication.
        :param lock: A threading lock to prevent concurrent access to the serial port.
        :param device_id: The ID of the device to communicate with.
        """
        self.ser_lock = lock
        self.ser = comm_port

        # Prevents the ESP32 from resetting during serial initialization
        self.ser.setRTS(False)
        self.ser.setDTR(False)

        # Storing
        self.__device_id = device_id
        self.__operation_mode = [OpMode.POSITION] * 4

        # Clear any residual log messages from the ESP32's serial buffer
        self.clear_serial_buffer()
        time.sleep(0.1)

    def create_message(
        self, instruction: Instruction, register: Register, value: int = 0, stepper_id: int = 0x00
    ) -> bytearray:
        """
        Create a message packet for communication with the ESP32.

        :param instruction: The instruction type (e.g., READ, WRITE).
        :param register: The register to read/write.
        :param value: The value to write (default is 0).
        :param stepper_id: The ID of the stepper motor (default is 0x00).
        :return: The constructed message as a bytearray.
        """
        # Prepare the data payload
        data = struct.pack(">i", value)  # Pack the value as a 32-bit big-endian integer

        # Calculate CRC32 for error detection
        crc = binascii.crc32(data) & 0xFFFFFFFF

        # Construct the message packet
        message = bytearray(
            [
                START_BYTE,  # Start byte
                self.__device_id,  # Device ID
                instruction,  # Instruction type
                stepper_id,  # Stepper ID
                register,  # Register address
            ]
        )
        message.extend(data)  # Append the 32-bit data
        message.extend(struct.pack(">I", crc))  # Append the CRC32
        message.append(END_BYTE)  # End byte

        return message

    def send_serial(self, message: bytearray) -> Optional[int]:
        """
        Send a message over the serial port and parse the response.

        :param message: The message to send.
        :return: The result from the ESP32, or None if an error occurred.
        """
        try:
            # Send the message
            self.ser.write(message)

            # Wait for the ESP32 to process and respond
            time.sleep(self.WAIT_TIME)

            # Check if the expected number of bytes is available
            if self.ser.in_waiting != 12:  # Expected response length is 12 bytes
                return None

            # Read the response
            data = self.ser.read(12)

            # Parse the response
            start_byte, device_id, validity = data[:3]
            result = int.from_bytes(data[3:7], byteorder="big", signed=False)
            crc32_recv = int.from_bytes(data[7:11], byteorder="big", signed=False)
            end_byte = data[11]

            # Validate response components
            if (
                start_byte != START_BYTE
                or device_id != self.__device_id
                or validity != Validity.GOOD_INSTRUCTION
                or end_byte != END_BYTE
            ):
                return None

            # Validate CRC32
            calculated_crc = binascii.crc32(struct.pack(">I", result)) & 0xFFFFFFFF
            if crc32_recv != calculated_crc:
                return None

            return result

        except Exception as e:
            print(f"Error during serial communication: {e}")
            return None

        finally:
            time.sleep(self.WAIT_TIME)

    def clear_serial_buffer(self) -> Optional[str]:
        """
        Clear the serial buffer of any residual data.

        :return: The decoded content of the cleared buffer (if any).
        """
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            return response.decode("utf-8")
        return None

    def read(self, stepper_id: int, register: Register) -> Optional[int]:
        """
        Read data from a specific register.

        :param register: The register to read from.
        :param stepper_id: The ID of the stepper motor (default is 0x00).
        :return: The value read, or None if an error occurred.
        """
        message = self.create_message(Instruction.READ, register, stepper_id=stepper_id)

        with self.ser_lock:
            return self.send_serial(message)

    def write(self, stepper_id: int, register: Register, value: int = 0) -> Optional[int]:
        """
        Write data to a specific register.

        :param register: The register to write to.
        :param value: The value to write.
        :param stepper_id: The ID of the stepper motor (default is 0x00).
        :return: The result of the write operation, or None if an error occurred.
        """
        message = self.create_message(Instruction.WRITE, register, value, stepper_id=stepper_id)

        with self.ser_lock:
            return self.send_serial(message)

    def init_stepper(
        self,
        stepper_id: int,
        stop_on_stall: bool = False,
        microstepping: int = 4,
        current: int = 31,
        holding_current_percentage: int = 50,
        operation_mode: OpMode = OpMode.POSITION,
        positioning_mode: PositioningMode = PositioningMode.ABSOLUTE,
    ) -> bool:
        """
        Initialize a stepper motor with specified settings.
        Returns True if all operations succeed, otherwise False.
        """
        # List of configuration steps with success status
        results = [
            self.write(stepper_id, Register.ENABLE_STEPPER),
            self.write(stepper_id, Register.STOP_ON_STALL, int(stop_on_stall)),
            self.write(stepper_id, Register.MICROSTEPPING, microstepping),
            self.write(stepper_id, Register.RUNNING_CURRENT, current),
            self.write(stepper_id, Register.HOLDING_CURRENT_PERCENTAGE, holding_current_percentage),
            self.write(stepper_id, Register.OPERATION_MODE, operation_mode),
            self.write(stepper_id, Register.POSITIONING_MODE, positioning_mode),
        ]

        # Verify all operations succeeded
        if all(results):
            self.__operation_mode[stepper_id] = operation_mode
            return True

        return False

    def configure_motion(
        self,
        stepper_id: int,
        target_position: int,
        rpm: int,
        acceleration_time_ms: int = None,
        decceleration_time_ms: int = None,
    ) -> bool:
        """
        Configure motion parameters for a stepper motor.
        Returns True if all operations succeed, otherwise False.
        """
        results = [
            self.write(stepper_id, Register.TARGET_POSITION, target_position),
            self.write(stepper_id, Register.TARGET_RPM, rpm),
        ]

        if acceleration_time_ms is not None:
            results.append(self.write(stepper_id, Register.ACEL_TIME, acceleration_time_ms))

        if decceleration_time_ms is not None:
            results.append(self.write(stepper_id, Register.DECEL_TIME, decceleration_time_ms))

        return all(results)

    def emergency_stop(self, stepper_id: int) -> bool:
        """
        Immediately stop the motor.
        Returns True if successful, otherwise False.
        """
        return self.write(stepper_id, Register.EMERGENCY_STOP) == 1

    def enable_stepper(self, stepper_id: int) -> bool:
        """
        Enables stepper (clear faults)
        """
        res = self.write(stepper_id, Register.ENABLE_STEPPER)

        return res == 1

    def is_running(self, stepper_id: int) -> bool:
        """
        Check if the motor is currently running.
        Returns True if running, otherwise False.
        """
        return self.read(stepper_id, Register.MOTOR_STATUS) == MotorStatus.RUNNING

    def is_stalled(self, stepper_id: int) -> bool:
        """
        Check if the motor is currently stalled.
        Returns True if stalled, otherwise False.
        """
        return self.read(stepper_id, Register.MOTOR_STATUS) == MotorStatus.STALLED

    # Internal blocker method
    def __blocker(self, stepper_id: int, result: queue.Queue, stop_event: threading.Event = None):
        """
        Block until the motor operation is complete or a stop event is triggered.
        Puts the result into the provided queue.
        """
        # Wait for the motor to stop running or for the stop event
        while self.is_running(stepper_id):
            if stop_event and stop_event.is_set():
                self.emergency_stop(stepper_id)
                break

        # Read motor status and positions
        motor_status = self.read(stepper_id, Register.MOTOR_STATUS)
        final_position = self.read(stepper_id, Register.CURRENT_POS)
        target_position = self.read(stepper_id, Register.TARGET_POSITION)

        # Validate communication results
        if motor_status is None or final_position is None or target_position is None:
            result.put([])
            return

        # Convert to signed integers if necessary
        final_position = final_position - 0x100000000 if final_position > 0x7FFFFFF else final_position
        target_position = target_position - 0x100000000 if target_position > 0x7FFFFFF else target_position

        # Return results as [stepper_id, success, position_error]
        result.put(
            [
                motor_status == MotorStatus.IDLE,
                target_position - final_position,
            ]
        )

    # Position mode blocker
    def position_mode_blocker(self, stepper_id: int, stop_event: threading.Event = None):
        """
        Block until the motor reaches the target position or a stop event is triggered.
        Returns [success, position_error].
        """
        # Ensure the motor is in position mode
        if self.__operation_mode[stepper_id] != OpMode.POSITION:
            return None

        result_queue = queue.Queue()
        worker = threading.Thread(target=self.__blocker, args=(stepper_id, result_queue, stop_event), daemon=True)
        worker.start()
        worker.join()

        return result_queue.get()
