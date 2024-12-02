import threading
from comm import *

stepper_controller = ESP32_TMC2240_API(port="COM22")

# =================================== TEST SETTINGS ================================== #
acceleration_time = 1
decceleration_time = 1
microstepping = 32

if __name__ == "__main__":
    from openpyxl import Workbook

    wb = Workbook()

    ws = wb.active
    ws.title = "Sheet1"
    ws.append(
        [
            "rpm",
            "acel",
            "acel_a",
            "decel",
            "decel_a",
        ]
    )

    # ------------------------------------------------------------------------------------ #
    stepper_controller.write(Register.ACEL_TIME, int(acceleration_time * 1000))
    stepper_controller.write(Register.DECEL_TIME, int(decceleration_time * 1000))

    rpm = 400
    while rpm <= 3000:
        print("run {}".format(rpm))
        stepper_controller.write(
            Register.TARGET_POSITION, round(30 * 200 * microstepping if rpm < 500 else 70 * 200 * microstepping)
        )
        stepper_controller.write(Register.TARGET_RPM, rpm)
        stepper_controller.write(Register.MOVE)

        motor_status = None
        while motor_status != "idle":
            motor_status = stepper_controller.read(Register.MOTOR_STATUS)
            # print(motor_status)

        acel_actual = stepper_controller.read(Register.ACTUAL_ACCELERATION_TIME)
        decel_actual = stepper_controller.read(Register.ACTUAL_DECCELERATION_TIME)

        ws.append(
            [
                rpm,
                round(acceleration_time * 1e6),
                acel_actual,
                round(decceleration_time * 1e6),
                decel_actual,
            ]
        )
        print(
            "({}) Acel: {}/{}   Decel: {}/{}".format(
                rpm,
                acel_actual,
                round(acceleration_time * 1e6),
                decel_actual,
                round(decceleration_time * 1e6),
            )
        )

        rpm += 50

    file_name = "rpm_acel_decel_2_microstepping.xlsx"
    wb.save(file_name)
