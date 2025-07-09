from comm import *
import serial
import threading

# ==================================================================================== #
#                                       Settings                                       #
# ==================================================================================== #
PORT      = "COM7"
DEVICE_ID = 0x01
STEPPERS  = [
    0x00,
    0x01,
    0x02,
    0x03,
]
MOTION = {
    0x00: (700, 10, 1000, 1000),
    0x01: (1600, 200, 1000, 1000),
    0x02: (-1600, 200, 1000, 1000),
    0x03: (1600, 200, 50, 0),
}  # (position, rpm, acceleration, decceleration)


if __name__ == "__main__":
    # =============================== Initialize Controller ============================== #
    PORT               = serial.Serial(PORT, 115200, timeout=1, dsrdtr=None)
    LOCK               = threading.Lock()
    STEPPER_CONTROLLER = ESP32_TMC2240_API(PORT, LOCK, DEVICE_ID)

    # ================================ Initialize stepper ================================ #
    res = []
    for stepper in STEPPERS:
        res.append(
            STEPPER_CONTROLLER.init_stepper(
                stepper,
                stop_on_stall              = False,
                microstepping              = 4,
                current                    = 31,
                holding_current_percentage = 50,
                operation_mode             = OpMode.POSITION,
                positioning_mode           = PositioningMode.RELATIVE,
            )
        )
        STEPPER_CONTROLLER.enable_stepper(0)

    if not all(res):
        print("init failed")
        exit()

    # ================================= Configure Motion ================================= #
    res = []
    for stepper in STEPPERS:
        res.append(STEPPER_CONTROLLER.configure_motion(stepper, *MOTION[stepper]))

    if not all(res):
        print("motion configuration failed")
        exit()

    # ======================================= Move ======================================= #
    res = []
    for stepper in STEPPERS:
        res.append(STEPPER_CONTROLLER.write(stepper, Register.MOVE))

    if not all(res):
        print("move failed")
        exit()

    # ====================================== Blocker ===================================== #
    MSG = "({}) | Status: ({:^5}) | Pos: {:^10} / {:^10} | Rpm: {:^10} / {:^10} |"
    while True:
        try:
            for stepper in STEPPERS:
                motor_status     = STEPPER_CONTROLLER.read(stepper, Register.MOTOR_STATUS)
                current_position = STEPPER_CONTROLLER.read_current_position(stepper)
                target_position  = STEPPER_CONTROLLER.read_target_position(stepper)
                current_rpm      = STEPPER_CONTROLLER.read_current_rpm(stepper)
                target_rpm       = STEPPER_CONTROLLER.read_target_rpm(stepper)

                print(MSG.format(
                    stepper, 
                    MotorStatus.get_name(motor_status), 
                    current_position, 
                    target_position, 
                    current_rpm, 
                    target_rpm)
                )

        except KeyboardInterrupt:
            for stepper in STEPPERS:
                STEPPER_CONTROLLER.emergency_stop(stepper)
            break
    # ------------------------------------------------------------------------------------ #