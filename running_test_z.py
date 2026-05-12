from mks_motor import MKSMotor
import threading

# --- Port assignment (verify with CAN2USBAdapterDeviceRecognition.py) ---
PORT_Z_A = 0
PORT_Z_B = 1

# --- Motion parameters ---
Z_TARGET_MM = 50
SPEED_PCT = 25
ACCEL_PCT = 10

# Two motors on separate USB2CAN adapters (port 0 and 1).
# Hardware-level CAN sync (0x4B broadcast) requires a shared
# bus, so threading is used instead for parallel operation.
motor_a = MKSMotor.open(port=PORT_Z_A)
motor_b = MKSMotor.open(port=PORT_Z_B)

# --- Initialization in parallel ---
# Homing is slow (~seconds); running both concurrently
# halves total startup time.
t_init_a = threading.Thread(target=lambda: (motor_a.setup(), motor_a.home()))
t_init_b = threading.Thread(target=lambda: (motor_b.setup(), motor_b.home()))
t_init_a.start()
t_init_b.start()
t_init_a.join()
t_init_b.join()

try:
    MKSMotor.move_sync([motor_a, motor_b], [(Z_TARGET_MM, SPEED_PCT, ACCEL_PCT)])
finally:
    motor_a.close()
    motor_b.close()
