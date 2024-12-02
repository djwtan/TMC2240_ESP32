from comm import *

if __name__ == "__main__":
    read(Register.DRV_STATUS)
    read(Register.OPERATION_MODE)
    read(Register.ACEL_TIME)
    read(Register.DECEL_TIME)
    read(Register.MOTOR_STATUS)
    read(Register.CURRENT_RPM)
    read(Register.CURRENT_POS)
    read(Register.ACTUAL_ACCELERATION_TIME)
    read(Register.ACTUAL_DECCELERATION_TIME)
    read_motor_status(2)
    write(Register.ENABLE_STEPPER)

    write(Register.ENABLE_STEPPER)
    write(Register.OPERATION_MODE, OperationMode.VELOCITY)

    write(Register.TARGET_POSITION, round(30 * 200))
    write(Register.TARGET_RPM, 500)

    write(Register.MOVE)

    motor_status = None
    while motor_status != "idle":
        motor_status = get_motor_status()
        # print(motor_status)

    acel_actual = read(Register.ACTUAL_ACCELERATION_TIME)
    decel_actual = read(Register.ACTUAL_DECCELERATION_TIME)
