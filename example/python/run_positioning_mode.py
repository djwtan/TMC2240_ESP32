from comm import *
import serial
import threading

# (acceleration, decceleration, position, rpm)
stepper_motion = {
    0x00: (1000, 1000, 80000, 1000),
    0x01: (1000, 1000, 80000 * 4, 400),
    0x02: (1000, 1000, 80000 * 8, 800),
    0x03: (1000, 1000, 80000 * 8, 800),
}


if __name__ == "__main__":

    # =============================== Initialize Controller ============================== #
    comm_port = serial.Serial("COM22", 115200, timeout=1, dsrdtr=None)
    comm_lock = threading.Lock()
    device_id = 0x01

    stepper_controller = ESP32_TMC2240_API(comm_port, comm_lock, device_id)

    # ================================ Initialize stepper ================================ #
    res = []
    res.append(
        stepper_controller.init_stepper(
            0,
            stop_on_stall=True,
            operation_mode=OpMode.POSITION,
            positioning_mode=PositioningMode.RELATIVE,
        )
    )
    res.append(
        stepper_controller.init_stepper(
            1,
            stop_on_stall=True,
            operation_mode=OpMode.POSITION,
            positioning_mode=PositioningMode.RELATIVE,
        )
    )
    res.append(
        stepper_controller.init_stepper(
            2,
            stop_on_stall=True,
            operation_mode=OpMode.POSITION,
            positioning_mode=PositioningMode.RELATIVE,
        )
    )
    res.append(
        stepper_controller.init_stepper(
            3,
            stop_on_stall=True,
            operation_mode=OpMode.POSITION,
            positioning_mode=PositioningMode.RELATIVE,
        )
    )

    if not all(res):
        print("init failed")
        exit()

    # ================================= Configure Motion ================================= #
    res = []
    for stepper in stepper_motion:
        res.append(stepper_controller.configure_motion(stepper, *stepper_motion[stepper]))

    if not all(res):
        print("motion configuration failed")
        exit()

    # ======================================= Move ======================================= #
    res = []
    for stepper in stepper_motion:
        res.append(stepper_controller.write(stepper, Register.MOVE))

    if not all(res):
        print("move failed")
        exit()

    # ==================================== Monitoring ==================================== #
    while True:
        try:
            stopped = []
            for stepper in stepper_motion:
                stopped.append(not stepper_controller.is_running(stepper))

            print(
                "stepper {} -> {} || {}".format(
                    0,
                    stepper_controller.read_motor_status(stepper),
                    stepper_controller.read(0, Register.CURRENT_POS),
                )
            )

            if all(stopped):
                break

        except KeyboardInterrupt:
            break

    for stepper in stepper_motion:
        print("stepper {} -> {}".format(stepper, stepper_controller.read_motor_status(stepper)))
