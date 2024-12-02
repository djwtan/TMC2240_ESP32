from comm import *

acceleration_time = 5
decceleration_time = 5


if __name__ == "__main__":
    # write(Register.ACEL_TIME, int(acceleration_time * 1000))
    # write(Register.DECEL_TIME, int(decceleration_time * 1000))
    # write(Register.OPERATION_MODE, OperationMode.VELOCITY)

    # res = write(Register.TARGET_POSITION, 500000)  # dummy value
    # print(res)
    # res = write(Register.TARGET_RPM, 800)
    # print(res)

    # write(Register.MOVE)

    # while True:
    #     try:
    #         # target_pos = read(Register.TARGET_POSITION)
    #         # current_pos = read(Register.CURRENT_POS)
    #         target_rpm = read(Register.TARGET_RPM)
    #         current_rpm = read(Register.CURRENT_RPM)
    #         # print("POS {} : {}".format(current_pos, target_pos))
    #         print("RPM {} : {}".format(current_rpm, target_rpm))
    #     except KeyboardInterrupt:
    #         res = write(Register.STOP_VELOCITY)
    #         print(res)

    # res = write(Register.STOP_VELOCITY)
    # print(res)

    # res = read(Register.MOTOR_STATUS)
    # print(res)

    # motor_status = None
    # while motor_status != "idle":
    #     motor_status = get_motor_status()
    #     # print(motor_status)

    # acel_actual = read(Register.ACTUAL_ACCELERATION_TIME)
    # decel_actual = read(Register.ACTUAL_DECCELERATION_TIME)
