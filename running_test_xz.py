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
Z_RISE_MM = 200
X_TARGET_MM = 100
Z_TARGET_MM = 50


def run_z_sync(motor_z_a, motor_z_b, mm):
    """Move both Z motors to the same absolute position in sync."""
    barrier = threading.Barrier(2)
    t_a = threading.Thread(
        target=motor_z_a.run,
        args=(([(mm, SPEED_PCT, ACCEL_PCT)], barrier)),
    )
    t_b = threading.Thread(
        target=motor_z_b.run,
        args=(([(mm, SPEED_PCT, ACCEL_PCT)], barrier)),
    )
    t_a.start()
    t_b.start()
    t_a.join()
    t_b.join()


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

    # Step 3: Z → Z_RISE_MM
    run_z_sync(motor_z_a, motor_z_b, Z_RISE_MM)

    # Step 4: X → X_TARGET_MM
    motor_x.move_to(X_TARGET_MM, SPEED_PCT, ACCEL_PCT)

    # Step 5: Z → Z_TARGET_MM
    run_z_sync(motor_z_a, motor_z_b, Z_TARGET_MM)

finally:
    motor_z_a.close()
    motor_z_b.close()
    motor_x.close()
