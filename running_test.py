from mks_motor import MKSMotor
import threading

motor_a = MKSMotor.open(port=0)
motor_b = MKSMotor.open(port=1)

# --- Initialization in parallel ---
t_init_a = threading.Thread(
    target=lambda: (motor_a.setup(), motor_a.home())
)
t_init_b = threading.Thread(
    target=lambda: (motor_b.setup(), motor_b.home())
)
t_init_a.start()
t_init_b.start()
t_init_a.join()
t_init_b.join()

# --- Move in parallel ---
barrier = threading.Barrier(2)

thread_a = threading.Thread(
    target=motor_a.run, args=([(50, 10, 0)], barrier)
)
thread_b = threading.Thread(
    target=motor_b.run, args=([(50, 10, 0)], barrier)
)

try:
    thread_a.start()
    thread_b.start()
    thread_a.join()
    thread_b.join()
finally:
    motor_a.close()
    motor_b.close()
