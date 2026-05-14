from mks_motor import MKSMotor
import threading

# Two motors on separate USB2CAN adapters (port 0 and 1).
# Hardware-level CAN sync (0x4B broadcast) requires a shared
# bus, so threading is used instead for parallel operation.
motor_a = MKSMotor.open(port=0)
motor_b = MKSMotor.open(port=1)

# --- Initialization in parallel ---
# Homing is slow (~seconds); running both concurrently
# halves total startup time.
t_init_a = threading.Thread(target=lambda: (motor_a.setup(), motor_a.home()))
t_init_b = threading.Thread(target=lambda: (motor_b.setup(), motor_b.home()))
t_init_a.start()
t_init_b.start()
t_init_a.join()
t_init_b.join()

# --- Move in parallel ---
# Barrier ensures both motors start moving at the same time
# after each has finished homing.
barrier = threading.Barrier(2)

thread_a = threading.Thread(target=motor_a.run, args=([(50, 10, 0)], barrier))
thread_b = threading.Thread(target=motor_b.run, args=([(50, 10, 0)], barrier))

try:
    thread_a.start()
    thread_b.start()
    thread_a.join()
    thread_b.join()
finally:
    motor_a.close()
    motor_b.close()
