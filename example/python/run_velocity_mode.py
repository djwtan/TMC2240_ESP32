from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")


def init_driver():
    response = stepper_controller.write(Register.ENABLE_STEPPER)  # can skip. Needs to be called after stall
    print(response)

    # ====================== Set acceleration and decceleration time ===================== #
    acceleration_time = 2
    decceleration_time = 2
    response = stepper_controller.write(Register.ACEL_TIME, int(acceleration_time * 1000))
    print(response)
    response = stepper_controller.write(Register.DECEL_TIME, int(decceleration_time * 1000))
    print(response)

    # ================================== Driver Settings ================================= #
    response = stepper_controller.write(Register.STOP_ON_STALL, 1)  # stop motor on stall
    print(response)
    response = stepper_controller.write(Register.MICROSTEPPING, 4)
    print(response)
    response = stepper_controller.write(Register.RUNNING_CURRENT, 31)  # 1-31
    print(response)
    response = stepper_controller.write(Register.HOLDING_CURRENT_PERCENTAGE, 50)  # 0-100
    print(response)
    response = stepper_controller.write(Register.OPERATION_MODE, OpMode.VELOCITY)  # velocity mode
    print(response)


if __name__ == "__main__":
    # ==================================== Initialize ==================================== #
    init_driver()

    # =============================== Run stepper at n rpm =============================== #
    rpm = 800  # 10 - 2400

    stepper_controller.write(Register.TARGET_POSITION, 200)  # dummy value
    stepper_controller.write(Register.TARGET_RPM, rpm)
    stepper_controller.write(Register.MOVE)  # needs to be called to initialize movement

    while True:
        try:
            # target_pos = stepper_controller.read(Register.TARGET_POSITION)
            # current_pos = stepper_controller.read(Register.CURRENT_POS)
            # print("POS {} : {}".format(current_pos, target_pos))

            # target_rpm = stepper_controller.read(Register.TARGET_RPM)
            # current_rpm = stepper_controller.read(Register.CURRENT_RPM)
            # print("RPM {} : {}".format(current_rpm, target_rpm))

            motor_status = stepper_controller.read(Register.MOTOR_STATUS)
            print("{}".format(motor_status))

        except KeyboardInterrupt:
            break

    stepper_controller.write(Register.STOP_VELOCITY)
    # stepper_controller.write(Register.EMERGENCY_STOP)
