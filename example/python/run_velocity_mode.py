from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")


def init_driver(stepper_id: int):
    # can skip. Needs to be called after stall
    response = stepper_controller.write(Register.ENABLE_STEPPER, stepper_id=stepper_id)
    print(response)

    # ====================== Set acceleration and decceleration time ===================== #
    acceleration_time = 2
    decceleration_time = 2
    response = stepper_controller.write(Register.ACEL_TIME, int(acceleration_time * 1000), stepper_id=stepper_id)
    print(response)
    response = stepper_controller.write(Register.DECEL_TIME, int(decceleration_time * 1000), stepper_id=stepper_id)
    print(response)

    # ================================== Driver Settings ================================= #
    response = stepper_controller.write(Register.STOP_ON_STALL, 1, stepper_id=stepper_id)  # stop motor on stall
    print(response)
    response = stepper_controller.write(Register.MICROSTEPPING, 4, stepper_id=stepper_id)
    print(response)
    response = stepper_controller.write(Register.RUNNING_CURRENT, 31, stepper_id=stepper_id)  # 1-31
    print(response)
    response = stepper_controller.write(Register.HOLDING_CURRENT_PERCENTAGE, 80, stepper_id=stepper_id)  # 0-100
    print(response)
    response = stepper_controller.write(Register.OPERATION_MODE, OpMode.VELOCITY, stepper_id=stepper_id)
    print(response)


if __name__ == "__main__":
    # ==================================== Initialize ==================================== #
    init_driver(0x00)
    init_driver(0x01)

    # =============================== Run stepper at n rpm =============================== #
    rpm = 500  # 10 - 2400
    stepper_controller.write(Register.TARGET_POSITION, 5000, stepper_id=0x00)  # dummy value
    stepper_controller.write(Register.TARGET_RPM, rpm, stepper_id=0x00)
    stepper_controller.write(Register.MOVE, stepper_id=0x00)  # needs to be called to initialize movement

    rpm = 500  # 10 - 2400
    stepper_controller.write(Register.TARGET_POSITION, -5000, stepper_id=0x01)  # dummy value
    stepper_controller.write(Register.TARGET_RPM, rpm, stepper_id=0x01)
    stepper_controller.write(Register.MOVE, stepper_id=0x01)  # needs to be called to initialize movement

    while True:
        try:
            target_pos = stepper_controller.read(Register.TARGET_POSITION, stepper_id=0x00)
            current_pos = stepper_controller.read(Register.CURRENT_POS, stepper_id=0x00)
            print("POS {} : {}".format(current_pos, target_pos))
            target_pos = stepper_controller.read(Register.TARGET_POSITION, stepper_id=0x01)
            current_pos = stepper_controller.read(Register.CURRENT_POS, stepper_id=0x01)
            print("POS {} : {}".format(current_pos, target_pos))

            # target_rpm = stepper_controller.read(Register.TARGET_RPM)
            # current_rpm = stepper_controller.read(Register.CURRENT_RPM)
            # print("RPM {} : {}".format(current_rpm, target_rpm))

            # res = stepper_controller.read(Register.STALL_VALUE)
            # print(res)

            # motor_status = stepper_controller.read(Register.MOTOR_STATUS, stepper_id=0x00)
            # print("{}".format(motor_status))
            # motor_status = stepper_controller.read(Register.MOTOR_STATUS, stepper_id=0x01)
            # print("{}".format(motor_status))

        except KeyboardInterrupt:
            break

    stepper_controller.write(Register.STOP_VELOCITY, stepper_id=0x00)
    stepper_controller.write(Register.STOP_VELOCITY, stepper_id=0x01)
    # stepper_controller.write(Register.EMERGENCY_STOP)
