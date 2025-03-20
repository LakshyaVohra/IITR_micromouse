from motion_pid import *
def run():
    choices = move_with_pid_pulses()
    if choices == [1, 1, 1]:
        motors.stop_motors()
        rotate()
        utime.sleep(0.1)
        rotate()
        utime.sleep(0.1)
    elif (choices[0] == 0 and sum(choices)!=0):
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(0.1)
    elif choices == [0, 1, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(0.1)
         # Stop movement
    elif choices == [0, 1, 1]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(0.1)
    elif choices == [1, 1, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate()
        utime.sleep(0.1)
    elif choices == [1, 0, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate()
        utime.sleep(0.1)