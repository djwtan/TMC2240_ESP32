from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")

STEPPER0 = True
STEPPER1 = True
STEPPER2 = True
STEPPER3 = True


def init_driver(stepper_id: int):
    # can skip. Needs to be called after stall
    response = stepper_controller.write(Register.ENABLE_STEPPER, stepper_id=stepper_id)
    print(response)

    # ====================== Set acceleration and decceleration time ===================== #
    acceleration_time = 0.5
    decceleration_time = 0.5
    response = stepper_controller.write(Register.ACEL_TIME, int(acceleration_time * 1000), stepper_id=stepper_id)
    print(response)
    response = stepper_controller.write(Register.DECEL_TIME, int(decceleration_time * 1000), stepper_id=stepper_id)
    print(response)

    # ================================== Driver Settings ================================= #
    response = stepper_controller.write(Register.STOP_ON_STALL, 0, stepper_id=stepper_id)  # stop motor on stall
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
    if STEPPER0:
        init_driver(0x00)
    if STEPPER1:
        init_driver(0x01)
    if STEPPER2:
        init_driver(0x02)
    if STEPPER3:
        init_driver(0x03)

    # =============================== Run stepper at n rpm =============================== #
    if STEPPER0:
        rpm = 500  # 10 - 2400
        stepper_controller.write(Register.TARGET_POSITION, 5000, stepper_id=0x00)  # dummy value
        stepper_controller.write(Register.TARGET_RPM, rpm, stepper_id=0x00)

    if STEPPER1:
        rpm = 500  # 10 - 2400
        stepper_controller.write(Register.TARGET_POSITION, -5000, stepper_id=0x01)  # dummy value
        stepper_controller.write(Register.TARGET_RPM, rpm, stepper_id=0x01)

    if STEPPER2:
        rpm = 500  # 10 - 2400
        stepper_controller.write(Register.TARGET_POSITION, 5000, stepper_id=0x02)  # dummy value
        stepper_controller.write(Register.TARGET_RPM, rpm, stepper_id=0x02)

    if STEPPER3:
        rpm = 500  # 10 - 2400
        stepper_controller.write(Register.TARGET_POSITION, -5000, stepper_id=0x03)  # dummy value
        stepper_controller.write(Register.TARGET_RPM, rpm, stepper_id=0x03)

    # ======================================= Move ======================================= #
    if STEPPER0:
        stepper_controller.write(Register.MOVE, stepper_id=0x00)  # needs to be called to initialize movement
    if STEPPER1:
        stepper_controller.write(Register.MOVE, stepper_id=0x01)  # needs to be called to initialize movement
    if STEPPER2:
        stepper_controller.write(Register.MOVE, stepper_id=0x02)  # needs to be called to initialize movement
    if STEPPER3:
        stepper_controller.write(Register.MOVE, stepper_id=0x03)  # needs to be called to initialize movement

    while True:
        try:
            if STEPPER0:
                motor_status = stepper_controller.read(Register.MOTOR_STATUS, stepper_id=0x00)
                print("{}".format(motor_status))
            if STEPPER1:
                motor_status = stepper_controller.read(Register.MOTOR_STATUS, stepper_id=0x01)
                print("{}".format(motor_status))
            if STEPPER2:
                motor_status = stepper_controller.read(Register.MOTOR_STATUS, stepper_id=0x02)
                print("{}".format(motor_status))
            if STEPPER3:
                motor_status = stepper_controller.read(Register.MOTOR_STATUS, stepper_id=0x03)
                print("{}".format(motor_status))

        except KeyboardInterrupt:
            break

    if STEPPER0:
        stepper_controller.write(Register.STOP_VELOCITY, stepper_id=0x00)
    if STEPPER1:
        stepper_controller.write(Register.STOP_VELOCITY, stepper_id=0x01)
    if STEPPER2:
        stepper_controller.write(Register.STOP_VELOCITY, stepper_id=0x02)
    if STEPPER3:
        stepper_controller.write(Register.STOP_VELOCITY, stepper_id=0x03)
