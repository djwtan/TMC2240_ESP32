from comm import *
import serial
import threading

steppers = [0x00, 0x01, 0x02, 0x03]

# (position, rpm, acceleration, decceleration)
stepper_motion_1 = {
    0x00: (-1, 50, 1000, 1000),
    0x01: (-1, 800, 1000, 1000),
    0x02: (1, 50, 1000, 1000),
    0x03: (1, 800, 1000, 1000),
}
stepper_motion_2 = {
    0x00: (-1, 800, 1000, 1000),
    0x01: (-1, 50, 1000, 1000),
    0x02: (1, 800, 1000, 1000),
    0x03: (1, 50, 1000, 1000),
}
stepper_motion_3 = {
    0x00: (-1, 400, 1000, 1000),
    0x01: (-1, 400, 1000, 1000),
    0x02: (1, 400, 1000, 1000),
    0x03: (1, 400, 1000, 1000),
}


if __name__ == "__main__":

    # =============================== Initialize Controller ============================== #
    comm_port = serial.Serial("COM22", 115200, timeout=1, dsrdtr=None)
    comm_lock = threading.Lock()
    device_id = 0x01

    stepper_controller = ESP32_TMC2240_API(comm_port, comm_lock, device_id)

    # ================================ Initialize stepper ================================ #
    res = []
    res.append(stepper_controller.init_stepper(0, stop_on_stall=True, operation_mode=OpMode.VELOCITY))
    res.append(stepper_controller.init_stepper(1, stop_on_stall=True, operation_mode=OpMode.VELOCITY))
    res.append(stepper_controller.init_stepper(2, stop_on_stall=True, operation_mode=OpMode.VELOCITY))
    res.append(stepper_controller.init_stepper(3, stop_on_stall=True, operation_mode=OpMode.VELOCITY))

    if not all(res):
        print("init failed")
        exit()

    while True:
        try:
            choice = input("Enter your choice (1/2/3): ").strip()

            # ===================================== Reenable ===================================== #
            for stepper in steppers:
                if stepper_controller.is_stalled(stepper):
                    stepper_controller.enable_stepper(stepper)

            # ==================================== Run motion ==================================== #
            if choice == "1":
                for stepper in steppers:
                    stepper_controller.configure_motion(stepper, *stepper_motion_1[stepper])
                for stepper in steppers:
                    stepper_controller.write(stepper, Register.MOVE)

            elif choice == "2":
                for stepper in steppers:
                    stepper_controller.configure_motion(stepper, *stepper_motion_2[stepper])
                for stepper in steppers:
                    stepper_controller.write(stepper, Register.MOVE)

            elif choice == "3":
                for stepper in steppers:
                    stepper_controller.configure_motion(stepper, *stepper_motion_3[stepper])
                for stepper in steppers:
                    stepper_controller.write(stepper, Register.MOVE)

            elif choice == "q":
                break

        except KeyboardInterrupt:
            break

    for stepper in steppers:
        stepper_controller.write(stepper, Register.STOP_VELOCITY)
