from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")

if __name__ == "__main__":
    # res = stepper_controller.read(Register.DRV_STATUS, 0x00)
    # print(res)

    # res = stepper_controller.read(Register.OPERATION_MODE)
    # res = stepper_controller.read(Register.ACEL_TIME)
    # res = stepper_controller.read(Register.DECEL_TIME)
    # res = stepper_controller.read(Register.MOTOR_STATUS)
    # res = stepper_controller.read(Register.CURRENT_RPM)
    # res = stepper_controller.read(Register.CURRENT_POS)
    # res = stepper_controller.read(Register.ACTUAL_ACCELERATION_TIME)
    # res = stepper_controller.read(Register.ACTUAL_DECCELERATION_TIME)
    res = stepper_controller.write(Register.RUNNING_CURRENT, 16, 0x00)
    print(res)
    res = stepper_controller.write(Register.RUNNING_CURRENT, 16, 0x01)
    print(res)
    res = stepper_controller.read(Register.RUNNING_CURRENT, 0x00)
    print(res)
    res = stepper_controller.read(Register.RUNNING_CURRENT, 0x01)
    print(res)
    # res = stepper_controller.write(Register.ENABLE_STEPPER, 0x01)

    # res = stepper_controller.write(Register.MICROSTEPPING, 1)
    # print(res)
    # res = stepper_controller.write(Register.RUNNING_CURRENT, 31)
    # print(res)
    # res = stepper_controller.write(Register.HOLDING_CURRENT_PERCENTAGE, 50)
    # print(res)

    # while True:
    #     res = stepper_controller.read(Register.STALL_VALUE)
    #     print(res)

    # res = stepper_controller.write(Register.DISABLE_STEPPER, 0)
    # print(res)
    # stepper_controller.write(Register.OPERATION_MODE, OpMode.VELOCITY)

    # stepper_controller.write(Register.TARGET_POSITION, round(30 * 200))
    # stepper_controller.write(Register.TARGET_RPM, 500)

    # stepper_controller.write(Register.MOVE)

    # acel_actual = stepper_controller.read(Register.ACTUAL_ACCELERATION_TIME)
    # decel_actual = stepper_controller.read(Register.ACTUAL_DECCELERATION_TIME)
