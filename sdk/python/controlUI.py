from ttSb import *
import serial
import threading
import tkinter as tk
import threading
import time
from tkinter import ttk

# ==================================================================================== #
#                                       Settings                                       #
# ==================================================================================== #
SERIAL_PORT = "COM7"
BAUDRATE = 115200
DEVICE_ID = 0x01
STEPPERS = [0x00, 0x01, 0x02, 0x03]

POLL_INTERVAL_MS = 300

# ==================================================================================== #
#                                  Controller Setup                                    #
# ==================================================================================== #
PORT = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1, dsrdtr=None)
LOCK = threading.Lock()
CTRL = ttSbAPI(PORT, LOCK, DEVICE_ID)

for stepper in STEPPERS:
    CTRL.init_stepper(
        stepper,
        stop_on_stall=False,
        microstepping=4,
        current=31,
        holding_current=100,
        operation_mode=OpMode.POSITION,
        positioning_mode=PositioningMode.ABSOLUTE,
    )
    CTRL.enable_stepper(stepper)

motion_queue = []
queue_thread = None
stop_queue_flag = False


# ==================================================================================== #
#                                       UI                                             #
# ==================================================================================== #
root = tk.Tk()
root.title("Stepper Motion Controller")

entries = {}

header = [
    "Stepper",
    "Position",
    "RPM",
    "Accel",
    "Decel",
    "Status",
    "Cur Pos",
    "Cur RPM",
]

for col, text in enumerate(header):
    ttk.Label(root, text=text, font=("Segoe UI", 10, "bold")).grid(row=0, column=col, padx=6)

for row, stepper in enumerate(STEPPERS, start=1):
    ttk.Label(root, text=f"{stepper}").grid(row=row, column=0)

    pos = ttk.Entry(root, width=10)
    rpm = ttk.Entry(root, width=10)
    acc = ttk.Entry(root, width=10)
    dec = ttk.Entry(root, width=10)

    status = ttk.Label(root, text="UNKNOWN", width=10)
    cur_pos = ttk.Label(root, text="—", width=12)
    cur_rpm = ttk.Label(root, text="—", width=12)

    pos.grid(row=row, column=1)
    rpm.grid(row=row, column=2)
    acc.grid(row=row, column=3)
    dec.grid(row=row, column=4)
    status.grid(row=row, column=5)
    cur_pos.grid(row=row, column=6)
    cur_rpm.grid(row=row, column=7)

    entries[stepper] = {
        "position": pos,
        "rpm": rpm,
        "accel": acc,
        "decel": dec,
        "status": status,
        "cur_pos": cur_pos,
        "cur_rpm": cur_rpm,
    }

move_btn = ttk.Button(root, text="MOVE", state="disabled")
move_btn.grid(
    row=len(STEPPERS) + 1,
    column=0,
    columnspan=len(header),
    pady=10,
)
rst_btn = ttk.Button(root, text="RESET", state="disabled")
rst_btn.grid(
    row=len(STEPPERS) + 1,
    column=0,
    columnspan=len(header) // 2,
    pady=10,
)
emergency_btn = ttk.Button(root, text="EMERGENCY STOP", style="Danger.TButton")
emergency_btn.grid(
    row=len(STEPPERS) + 2,
    column=0,
    columnspan=len(header),
    pady=10,
)
add_queue_btn = ttk.Button(root, text="ADD TO QUEUE")
add_queue_btn.grid(
    row=len(STEPPERS) + 3,
    column=0,
    columnspan=len(header),
    pady=10,
)
empty_queue_btn = ttk.Button(root, text="EMPTY QUEUE")
empty_queue_btn.grid(
    row=len(STEPPERS) + 3,
    column=0,
    columnspan=len(header) // 2,
    pady=10,
)

move_queue_btn = ttk.Button(root, text="MOVE QUEUE")
move_queue_btn.grid(
    row=len(STEPPERS) + 4,
    column=0,
    columnspan=len(header),
    pady=10,
)

stop_queue_btn = ttk.Button(root, text="STOP QUEUE")
stop_queue_btn.grid(
    row=len(STEPPERS) + 4,
    column=0,
    columnspan=len(header) // 2,
    pady=10,
)


# ==================================================================================== #
#                                   Logic                                              #
# ==================================================================================== #
def poll_status():
    all_idle = True

    for stepper in STEPPERS:
        status_val = CTRL.read(stepper, Register.MOTOR_STATUS)
        status_enum = MotorStatus(status_val)

        cur_pos = CTRL.read_current_position(stepper)
        cur_rpm = CTRL.read_current_rpm(stepper)

        entries[stepper]["status"].config(text=status_enum.name)
        entries[stepper]["cur_pos"].config(text=f"{cur_pos:.2f}")
        entries[stepper]["cur_rpm"].config(text=f"{cur_rpm:.2f}")

        if status_enum != MotorStatus.IDLE:
            all_idle = False

    state = "normal" if all_idle else "disabled"
    move_btn.config(state=state)
    rst_btn.config(state=state)
    root.after(POLL_INTERVAL_MS, poll_status)


def move():
    valid_steppers = []

    for stepper in STEPPERS:
        try:
            position = int(entries[stepper]["position"].get())
            rpm = int(entries[stepper]["rpm"].get())
            accel = int(entries[stepper]["accel"].get())
            decel = int(entries[stepper]["decel"].get())
        except ValueError:
            print(f"[WARN] Invalid input for stepper {stepper}, skipping")
            continue

        ok = CTRL.configure_motion(stepper, position, rpm, accel, decel)
        if ok:
            valid_steppers.append(stepper)
        else:
            print(f"[WARN] Failed to configure stepper {stepper}")

    for stepper in valid_steppers:
        CTRL.write(stepper, Register.MOVE)


move_btn.config(command=move)

# ------------------------------------------------------------------------------------ #


def reset(stepper: int):
    CTRL.init_stepper(
        stepper,
        stop_on_stall=False,
        microstepping=4,
        current=31,
        holding_current=100,
        operation_mode=OpMode.POSITION,
        positioning_mode=PositioningMode.ABSOLUTE,
    )
    CTRL.enable_stepper(stepper)


def on_reset():
    for stepper in STEPPERS:
        reset(stepper)


rst_btn.config(command=on_reset)


# ------------------------------------------------------------------------------------ #
def on_emergency_stop():
    for stepper in STEPPERS:
        CTRL.emergency_stop(stepper)
    print("[EMERGENCY] All steppers stopped!")


emergency_btn.config(command=on_emergency_stop)


# ------------------------------------------------------------------------------------ #
def add_to_queue():
    motion_item = {}
    for stepper in STEPPERS:
        try:
            position = int(entries[stepper]["position"].get())
            rpm = int(entries[stepper]["rpm"].get())
            accel = int(entries[stepper]["accel"].get())
            decel = int(entries[stepper]["decel"].get())
        except ValueError:
            print(f"[WARN] Invalid input for stepper {stepper}, skipping")
            continue
        motion_item[stepper] = (position, rpm, accel, decel)

    if motion_item:
        motion_queue.append(motion_item)
        print(f"[QUEUE] Added: {motion_item}")


def empty_queue():
    motion_queue.clear()
    print(f"[QUEUE] CLeared.")


# ------------------------------------------------------------------------------------ #


def move_queue_loop():
    global stop_queue_flag
    while not stop_queue_flag:
        for motion_item in motion_queue:
            if stop_queue_flag:
                break

            # Configure motion
            for stepper, params in motion_item.items():
                CTRL.configure_motion(stepper, *params)

            # Start move
            for stepper in motion_item.keys():
                CTRL.write(stepper, Register.MOVE)

            # Wait until all motors in this motion are IDLE
            while True:
                all_idle = True
                for stepper in motion_item.keys():
                    status = MotorStatus(CTRL.read(stepper, Register.MOTOR_STATUS))
                    if status != MotorStatus.IDLE:
                        all_idle = False
                        break
                if all_idle:
                    break
                time.sleep(0.1)  # avoid busy loop

        # End of queue iteration; loop continues unless stop flag is set
    print("[QUEUE] Looping stopped.")


def start_queue_loop():
    global queue_thread, stop_queue_flag
    if not motion_queue:
        print("[QUEUE] Motion queue is empty!")
        return
    if queue_thread and queue_thread.is_alive():
        print("[QUEUE] Queue already running!")
        return
    stop_queue_flag = False
    queue_thread = threading.Thread(target=move_queue_loop, daemon=True)
    queue_thread.start()


def stop_queue():
    global stop_queue_flag
    stop_queue_flag = True
    print("[QUEUE] Stop requested — will finish current motion then halt.")


empty_queue_btn.config(command=empty_queue)
add_queue_btn.config(command=add_to_queue)
move_queue_btn.config(command=start_queue_loop)
stop_queue_btn.config(command=stop_queue)

# ==================================================================================== #
#                                    Start                                             #
# ==================================================================================== #
poll_status()
root.mainloop()
