from motion_pid import *
DURATION = 0.3
def run():
    choices = move_with_pid_pulses()
    if choices == [1, 1, 1]:
        motors.stop_motors()
        rotate()
        utime.sleep(DURATION)
        rotate()
        utime.sleep(DURATION)
    elif (choices[0] == 0 and sum(choices)!=0):
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(DURATION)
    elif choices == [0, 1, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(DURATION)
         # Stop movement
    elif choices == [0, 1, 1]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(DURATION)
    elif choices == [1, 1, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate()
        utime.sleep(DURATION)
    elif choices == [1, 0, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate()
        utime.sleep(DURATION)