from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")

if __name__ == "__main__":
    # stepper_controller.read(Register.DRV_STATUS)
    # stepper_controller.read(Register.OPERATION_MODE)
    # stepper_controller.read(Register.ACEL_TIME)
    # stepper_controller.read(Register.DECEL_TIME)
    # stepper_controller.read(Register.MOTOR_STATUS)
    # stepper_controller.read(Register.CURRENT_RPM)
    # stepper_controller.read(Register.CURRENT_POS)
    # stepper_controller.read(Register.ACTUAL_ACCELERATION_TIME)
    # stepper_controller.read(Register.ACTUAL_DECCELERATION_TIME)
    # stepper_controller.write(Register.ENABLE_STEPPER)

    # res = stepper_controller.write(Register.MICROSTEPPING, 1)
    # print(res)
    # res = stepper_controller.write(Register.RUNNING_CURRENT, 31)
    # print(res)
    # res = stepper_controller.write(Register.HOLDING_CURRENT_PERCENTAGE, 50)
    # print(res)

    while True:
        res = stepper_controller.read(Register.STALL_VALUE)
        print(res)

    # res = stepper_controller.write(Register.DISABLE_STEPPER, 0)
    # print(res)
    # stepper_controller.write(Register.OPERATION_MODE, OpMode.VELOCITY)

    # stepper_controller.write(Register.TARGET_POSITION, round(30 * 200))
    # stepper_controller.write(Register.TARGET_RPM, 500)

    # stepper_controller.write(Register.MOVE)

    # acel_actual = stepper_controller.read(Register.ACTUAL_ACCELERATION_TIME)
    # decel_actual = stepper_controller.read(Register.ACTUAL_DECCELERATION_TIME)
