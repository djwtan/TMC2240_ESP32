from ttSb import *
import serial
import threading

# ==================================================================================== #
#                                       Settings                                       #
# ==================================================================================== #
PORT = "COM7"
DEVICE_ID = 0x01
STEPPERS = [
    0x00,
    0x01,
    0x02,
    0x03,
]
MOTION_1 = {
    0x00: (1, 10, 1000, 1000),
    0x01: (-1, 800, 1000, 1000),
    0x02: (1, 50, 1000, 1000),
    0x03: (1, 800, 1000, 1000),
}  # (position, rpm, acceleration, decceleration)
MOTION_2 = {
    0x00: (1, 1400, 1000, 1000),
    0x01: (-1, 50, 1000, 1000),
    0x02: (1, 800, 1000, 1000),
    0x03: (1, 50, 1000, 1000),
}  # (position, rpm, acceleration, decceleration)
MOTION_3 = {
    0x00: (1, 400, 1000, 1000),
    0x01: (-1, 400, 1000, 1000),
    0x02: (1, 400, 1000, 1000),
    0x03: (1, 400, 1000, 1000),
}  # (position, rpm, acceleration, decceleration)

# ==================================================================================== #
#                                         Print                                        #
# ==================================================================================== #
print_status = False


def read_status(controller: ttSbAPI, steppers):
    global print_status
    MSG = "({}) | Status: ({:^5}) | Rpm: {:^10} / {:^10} |"

    while True:
        if not print_status:
            continue

        for stepper in steppers:
            motor_status = controller.read(stepper, Register.MOTOR_STATUS)
            current_rpm = controller.read_current_rpm(stepper)
            target_rpm = controller.read_target_rpm(stepper)

            print(MSG.format(stepper, MotorStatus.get_name(motor_status), current_rpm, target_rpm))


if __name__ == "__main__":
    # =============================== Initialize Controller ============================== #
    PORT = serial.Serial(PORT, 115200, timeout=1, dsrdtr=None)
    LOCK = threading.Lock()
    STEPPER_CONTROLLER = ttSbAPI(PORT, LOCK, DEVICE_ID)

    # ==================================== Read thread =================================== #
    threading.Thread(
        target=read_status,
        daemon=True,
        args=(
            STEPPER_CONTROLLER,
            STEPPERS,
        ),
    ).start()

    # ================================ Initialize stepper ================================ #
    res = []
    for stepper in STEPPERS:
        res.append(
            STEPPER_CONTROLLER.init_stepper(
                stepper,
                stop_on_stall=False,
                microstepping=4,
                current=31,
                holding_current_percentage=50,
                operation_mode=OpMode.VELOCITY,
            )
        )
        STEPPER_CONTROLLER.enable_stepper(0)

    if not all(res):
        print("init failed")
        exit()

    # ==================================================================================== #
    #                                         Main                                         #
    # ==================================================================================== #
    while True:
        try:
            choice = input("1/2/3/4/y/n").strip()

            # ===================================== Re-enable ==================================== #
            for stepper in STEPPERS:
                status = STEPPER_CONTROLLER.read(stepper, Register.MOTOR_STATUS)
                if status == MotorStatus.STALLED:
                    if STEPPER_CONTROLLER.enable_stepper(stepper):
                        print("({} Re-enabled)".format(stepper))
                    else:
                        print("({} Re-enable failed!)".format(stepper))

            # ==================================== Run motion ==================================== #
            if choice == "1":
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.configure_motion(stepper, *MOTION_1[stepper])
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.write(stepper, Register.MOVE)

            elif choice == "2":
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.configure_motion(stepper, *MOTION_2[stepper])
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.write(stepper, Register.MOVE)

            elif choice == "3":
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.configure_motion(stepper, *MOTION_3[stepper])
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.write(stepper, Register.MOVE)

            elif choice == "4":
                for stepper in STEPPERS:
                    STEPPER_CONTROLLER.write(stepper, Register.STOP_VELOCITY)

            elif choice == "y":
                print_status = True

            elif choice == "n":
                print_status = False
            # ------------------------------------------------------------------------------------ #

        except KeyboardInterrupt:
            break

    for stepper in STEPPERS:
        STEPPER_CONTROLLER.write(stepper, Register.STOP_VELOCITY)
