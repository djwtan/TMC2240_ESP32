from comm import *
import serial
import threading

steppers = [
    0x00,
    # 0x01,
    # 0x02,
    # 0x03,
]

# (position, rpm, acceleration, decceleration)
stepper_motion_1 = {
    0x00: (1, 10, 1000, 1000),
    0x01: (-1, 800, 1000, 1000),
    0x02: (1, 50, 1000, 1000),
    0x03: (1, 800, 1000, 1000),
}
stepper_motion_2 = {
    0x00: (1, 1400, 1000, 1000),
    0x01: (-1, 50, 1000, 1000),
    0x02: (1, 800, 1000, 1000),
    0x03: (1, 50, 1000, 1000),
}
stepper_motion_3 = {
    0x00: (1, 400, 1000, 1000),
    0x01: (-1, 400, 1000, 1000),
    0x02: (1, 400, 1000, 1000),
    0x03: (1, 400, 1000, 1000),
}

def read_status(stepper_controller: ESP32_TMC2240_API, steppers):
    import struct
    while True:
        for stepper in steppers:
            # sv = stepper_controller.read(stepper, Register.OPERATION_MODE)
            # float_value = struct.unpack('f', struct.pack('I', sv))[0]
            # print(f"stepper {stepper}: {float_value}")
            # print(f"stepper {stepper}: {sv}")

            # stall_value = stepper_controller.read(stepper, Register.STALL_VALUE)
            # # print(f"stepper {stepper}: {(stall_value)}")
            # if stall_value == 0:
            sv = stepper_controller.read(stepper, Register.CURRENT_RPM)
            float_value = struct.unpack('f', struct.pack('I', sv))[0]
            print(f"stall at {float_value}")


            # status = stepper_controller.read(stepper, Register.DRV_STATUS)
            # stall_bit = (status >> 2) & 1
            # print(f"stepper {stepper}: {(stall_bit)}")

            # ms = stepper_controller.read(stepper, Register.MOTOR_STATUS)
            # # print(f"stepper {stepper}: {MotorStatus.get_name(ms)}")
            # if MotorStatus.get_name(ms) == "stall":
            #     sv = stepper_controller.read(stepper, Register.CURRENT_RPM)
            #     float_value = struct.unpack('f', struct.pack('I', sv))[0]
            #     print(f"stall at {float_value}")


if __name__ == "__main__":

    # =============================== Initialize Controller ============================== #
    comm_port = serial.Serial("COM7", 115200, timeout=1, dsrdtr=None)
    comm_lock = threading.Lock()
    device_id = 0x01

    stepper_controller = ESP32_TMC2240_API(comm_port, comm_lock, device_id)

    # ================================ Initialize stepper ================================ #
    res = []
    for stepper in steppers:
        res.append(stepper_controller.init_stepper(stepper, stop_on_stall=False, operation_mode=OpMode.VELOCITY, holding_current_percentage=100))

    if not all(res):
        print("init failed")
        exit()


    threading.Thread(target=read_status, args=(stepper_controller, steppers,), daemon=True).start()

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
