from comm import *

acceleration_time = 5
decceleration_time = 5


if __name__ == "__main__":
    # res = write(Register.ENABLE_STEPPER)
    # print(res)
    # write(Register.ACEL_TIME, int(acceleration_time * 1000))
    # write(Register.DECEL_TIME, int(decceleration_time * 1000))
    # write(Register.OPERATION_MODE, OpMode.VELOCITY)

    # res = write(Register.STOP_ON_STALL, 1)  # stop motor on stall
    # print(res)

    # res = write(Register.TARGET_POSITION, 200)  # dummy value
    # print(res)
    # res = write(Register.TARGET_RPM, 800)
    # print(res)
    # res = write(Register.MOVE)
    # print(res)

    # while True:
    #     try:
    #         # target_pos = read(Register.TARGET_POSITION)
    #         # current_pos = read(Register.CURRENT_POS)
    #         # print("POS {} : {}".format(current_pos, target_pos))

    #         # target_rpm = read(Register.TARGET_RPM)
    #         # current_rpm = read(Register.CURRENT_RPM)
    #         # print("RPM {} : {}".format(current_rpm, target_rpm))

    #         drv_status = read(Register.DRV_STATUS)
    #         motor_status = read(Register.MOTOR_STATUS)
    #         print("{}({}) : {}".format(bin(int(drv_status)), drv_status, motor_status))
    #         # print(drv_status)
    #         # print(motor_status)
    #     except KeyboardInterrupt:
    #         # res = write(Register.STOP_VELOCITY)
    #         # print(res)
    #         break

    res = write(Register.STOP_VELOCITY)
    print(res)
    # res = write(Register.ENABLE_STEPPER)
    # print(res)
    # res = write(Register.EMERGENCY_STOP)
    # print(res)

    # res = read(Register.MOTOR_STATUS)
    # print(res)
