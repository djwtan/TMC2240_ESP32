from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")

acceleration_time = 5
decceleration_time = 5


if __name__ == "__main__":
    # res = stepper_controller.write(Register.ENABLE_STEPPER)
    # print(res)
    stepper_controller.write(Register.ACEL_TIME, int(acceleration_time * 1000))
    stepper_controller.write(Register.DECEL_TIME, int(decceleration_time * 1000))
    stepper_controller.write(Register.OPERATION_MODE, OpMode.VELOCITY)

    res = stepper_controller.write(Register.STOP_ON_STALL, 1)  # stop motor on stall
    print(res)

    res = stepper_controller.write(Register.TARGET_POSITION, 200)  # dummy value
    print(res)
    res = stepper_controller.write(Register.TARGET_RPM, 800)
    print(res)
    res = stepper_controller.write(Register.MOVE)
    print(res)

    # while True:
    #     try:
    #         # target_pos = stepper_controller.read(Register.TARGET_POSITION)
    #         # current_pos = stepper_controller.read(Register.CURRENT_POS)
    #         # print("POS {} : {}".format(current_pos, target_pos))

    #         # target_rpm = stepper_controller.read(Register.TARGET_RPM)
    #         # current_rpm = stepper_controller.read(Register.CURRENT_RPM)
    #         # print("RPM {} : {}".format(current_rpm, target_rpm))

    #         drv_status = stepper_controller.read(Register.DRV_STATUS)
    #         motor_status = stepper_controller.read(Register.MOTOR_STATUS)
    #         print("{}({}) : {}".format(bin(int(drv_status)), drv_status, motor_status))
    #         # print(drv_status)
    #         # print(motor_status)
    #     except KeyboardInterrupt:
    #         # res = stepper_controller.write(Register.STOP_VELOCITY)
    #         # print(res)
    #         break

    # res = stepper_controller.write(Register.STOP_VELOCITY)
    # print(res)
    # res = stepper_controller.write(Register.ENABLE_STEPPER)
    # print(res)
    # res = stepper_controller.write(Register.EMERGENCY_STOP)
    # print(res)

    # res = stepper_controller.read(Register.MOTOR_STATUS)
    # print(res)
