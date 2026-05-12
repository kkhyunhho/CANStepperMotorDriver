from mks_motor import MKSMotor
import threading

# --- Port assignment (verify with CAN2USBAdapterDeviceRecognition.py) ---
PORT_Z_A = 0
PORT_Z_B = 1
PORT_X = 2

# --- Motion parameters ---
SPEED_PCT = 25
ACCEL_PCT = 10

# --- Cycle positions (absolute, mm) ---
Z_ASCEND_MM = 200
Z_DESCEND_MM = 50

X_TARGET_MM_1 = 100


motor_z_a = MKSMotor.open(port=PORT_Z_A)
motor_z_b = MKSMotor.open(port=PORT_Z_B)
motor_x = MKSMotor.open(port=PORT_X)

try:
    # Step 1: Z setup + home (parallel)
    t_z_a = threading.Thread(
        target=lambda: (motor_z_a.setup(), motor_z_a.home())
    )
    t_z_b = threading.Thread(
        target=lambda: (motor_z_b.setup(), motor_z_b.home())
    )
    t_z_a.start()
    t_z_b.start()
    t_z_a.join()
    t_z_b.join()

    # Step 2: X setup + home
    motor_x.setup()
    motor_x.home()

    # Step 3: Z → Z_ASCEND_MM
    MKSMotor.move_sync([motor_z_a, motor_z_b], [(Z_ASCEND_MM, SPEED_PCT, ACCEL_PCT)])

    # Step 4: X → X_TARGET_MM_1
    motor_x.move_to(X_TARGET_MM_1, SPEED_PCT, ACCEL_PCT)

    # Step 5: Z → Z_DESCEND_MM
    MKSMotor.move_sync([motor_z_a, motor_z_b], [(Z_DESCEND_MM, SPEED_PCT, ACCEL_PCT)])

finally:
    motor_z_a.close()
    motor_z_b.close()
    motor_x.close()
