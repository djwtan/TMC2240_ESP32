START_BYTE = 0xAA
END_BYTE = 0x55


# ===================================== Response ===================================== #
class Validity:
    BAD_INSTRUCTION = 0x00
    GOOD_INSTRUCTION = 0x01


class MotorStatus:
    STALLED = 0
    OVERSPEED = 1
    IDLE = 2
    RUNNING = 3
    POWER_ERR = 4
    NOT_INIT = 5

    @staticmethod
    def get_name(m_state: int):
        """for printing only"""
        if m_state == 0:
            return "stall"
        if m_state == 1:
            return "overspeed"
        if m_state == 2:
            return "idle"
        if m_state == 3:
            return "running"
        if m_state == 4:
            return "power error"
        if m_state == 5:
            return "not init"

        return None


class HomingMethod:
    IMMEDIATE = 0
    SENSOR = 1
    TORQUE = 2


class OperationMode:
    POSITION = 0
    VELOCITY = 1
    INVERSE_TIME = 2


class PositioningMode:
    ABSOLUTE = 0
    RELATIVE = 1


class SensorHomingCondition:
    HIGH_TO_LOW_EDGE = 0
    LOW_TO_HIGH_EDGE = 1


# ====================================== Message ===================================== #
class Instruction:
    READ = 0x00
    WRITE = 0x01


class Register:
    TARGET_POSITION = 0x00
    TARGET_RPM = 0x01
    MOVE = 0x02
    TEMPERATURE = 0x03
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
    POSITIONING_MODE = 0x1A


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
