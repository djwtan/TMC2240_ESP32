import threading
from comm import *

if __name__ == "__main__":
    from openpyxl import Workbook

    wb = Workbook()

    ws = wb.active
    ws.title = "Sheet1"
    ws.append(["rpm", "acel", "acel_a", "decel", "decel_a"])

    # ------------------------------------------------------------------------------------ #
    write(Register.ACEL_TIME, int(1 * 1000))
    write(Register.DECEL_TIME, int(1 * 1000))

    rpm = 400
    while rpm <= 3000:
        print("run {}".format(rpm))
        write(Register.TARGET_POSITION, round(30 * 200 if rpm < 500 else 70 * 200))
        write(Register.TARGET_RPM, rpm)

        write(Register.MOVE)

        motor_status = None
        while motor_status != "idle":
            motor_status = get_motor_status()
            # print(motor_status)

        acel_actual = read(Register.ACTUAL_ACCELERATION_TIME)
        decel_actual = read(Register.ACTUAL_DECCELERATION_TIME)

        ws.append([rpm, 4000000, acel_actual, 4000000, decel_actual])
        print("({}) Acel: {}/4000000   Decel: {}/4000000".format(rpm, acel_actual, decel_actual))

        rpm += 50

    file_name = "rpm_acel_decel_time_corrected_2.xlsx"
    wb.save(file_name)
