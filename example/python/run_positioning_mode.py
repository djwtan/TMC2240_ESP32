from comm import *
import serial
import threading

steppers = [0x00, 0x01, 0x02, 0x03]

# (position, rpm, acceleration, decceleration)
stepper_motion = {
    0x00: (-1600 * 1, 150, 50, 0),
    0x01: (1600 * 1, 200, 1000, 1000),
    0x02: (-1600 * 1, 200, 1000, 1000),
    0x03: (1600 * 1, 200, 50, 0),
}

if __name__ == "__main__":

    # =============================== Initialize Controller ============================== #
    comm_port = serial.Serial("COM22", 115200, timeout=1, dsrdtr=None)
    comm_lock = threading.Lock()
    device_id = 0x01

    stepper_controller = ESP32_TMC2240_API(comm_port, comm_lock, device_id)

    # ================================ Initialize stepper ================================ #
    res = []
    for stepper in steppers:
        res.append(
            stepper_controller.init_stepper(
                stepper, stop_on_stall=True, operation_mode=OpMode.POSITION, positioning_mode=PositioningMode.RELATIVE
            )
        )

    if not all(res):
        print("init failed")
        exit()

    # ================================= Configure Motion ================================= #
    res = []
    for stepper in steppers:
        res.append(stepper_controller.configure_motion(stepper, *stepper_motion[stepper]))

    if not all(res):
        print("motion configuration failed")
        exit()

    # ======================================= Move ======================================= #
    res = []
    for stepper in steppers:
        res.append(stepper_controller.write(stepper, Register.MOVE))

    if not all(res):
        print("move failed")
        exit()

    # ====================================== Blocker ===================================== #
    outcome = []

    for stepper in steppers:
        outcome.append(stepper_controller.position_mode_blocker(stepper))

    print(outcome)

    # =================================== Auto Correct =================================== #
    # ! Not reliable
    for stepper in steppers:
        if not outcome[stepper][0]:
            print("correct stepper {} by {} steps".format(stepper, outcome[stepper][1]))

            res = []
            res.append(stepper_controller.enable_stepper(stepper))
            time.sleep(1)
            res.append(stepper_controller.configure_motion(stepper, outcome[stepper][1], 100))
            res.append(stepper_controller.write(stepper, Register.MOVE))

            if not all(res):
                print("move failed")
                exit()
