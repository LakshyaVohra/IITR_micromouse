from machine import Pin, I2C #type: ignore
import utime #type: ignore
from mpu6050 import MPU6050
from motors import MotorControl
from sensors import DistanceSensors
from pid import PID

i2c = I2C(0, scl=Pin(1), sda=Pin(0))  # Adjust pins based on your board
mpu = MPU6050(i2c)
motors = MotorControl(68)
distance_sensors = DistanceSensors()
distance_pid = PID(0.4,0,0.2)

TARGET_DISTANCE = 35

def move_with_pid_pulses(target_pulses=550):
    """
    Move forward for a specific number of encoder pulses while maintaining distance from walls using PID.
    """
    # Reset encoders
    motors.Encodervalue1 = 0
    motors.Encodervalue2 = 0

    # Start moving forward
    motors.move(1, 1, None)

    while abs(motors.Encodervalue1) < target_pulses and abs(motors.Encodervalue2) < target_pulses:
        distances, values = distance_sensors.read_distances()
        left_distance = distances[0]  # Left wall distance
        right_distance = distances[2]  # Right wall distance
        left_valid = values[0]  # 1 if left wall is detected, else 0
        right_valid = values[2]

        if left_valid and right_valid:
            error = left_distance - right_distance

        elif not left_valid and right_valid:
            error = TARGET_DISTANCE - right_distance  # Keep 3.5 cm from right wall

        elif left_valid and not right_valid:
            error = left_distance - TARGET_DISTANCE  # Keep 3.5 cm from left wall

        else:
            error = 0

        # Compute PID correction
        correction = distance_pid.compute(error)

        # Adjust motor speed based on correction
        base_speed = 65
        left_speed = max(40, min(90, base_speed + correction))
        right_speed = max(40, min(90, base_speed - correction))

        # Apply adjusted speeds
        motors.PWM1.duty_u16(int(left_speed * 65535 / 100))
        motors.PWM2.duty_u16(int(right_speed * 65535 / 100))

        print(f"Encoder1: {motors.Encodervalue1}, Encoder2: {motors.Encodervalue2}, "
              f"L: {left_distance}mm, R: {right_distance}mm, Correction: {correction:.2f}")

        # utime.sleep_ms(50)

    motors.stop_motors()
    return values

def rotate(target_angle=60.25):
    """
    Rotates the bot to a given angle using the gyroscope.
    Positive angle → Right turn, Negative angle → Left turn
    """
    current_angle = 0
    last_time = utime.ticks_ms()  # Track time

    # Determine motor directions based on angle sign
    a, b = (1, -1) if target_angle > 0 else (-1, 1)
    motors.move(a, b, 0)  # Start rotation

    while abs(current_angle) < abs(target_angle):
        gyro_z = mpu.get_gyro()["gz"]

        # Use time difference for angle integration
        current_time = utime.ticks_ms()
        dt = utime.ticks_diff(current_time, last_time) / 1000.0  # FIXED: Use ticks_diff()
        last_time = current_time

        current_angle += gyro_z * dt
        print(f"Current Angle: {current_angle:.2f}°")

    # Gradual Stop for Stability
    motors.stop_motors()
    print("Rotation complete!")


while True:
    choices = move_with_pid_pulses()
    if choices == [1, 1, 1]:
        motors.stop_motors()
        rotate()
        utime.sleep(0.2)
        rotate()
        utime.sleep(0.2)
    elif (choices[0] == 0 and sum(choices)!=0):
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(0.2)
    elif choices == [0, 1, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(0.2)
         # Stop movement
    elif choices == [0, 1, 1]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate(-60.25)
        utime.sleep(0.2)
    elif choices == [1, 1, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate()
        utime.sleep(0.2)
    elif choices == [1, 0, 0]:
        # utime.sleep_ms(250)
        motors.stop_motors()
        rotate()
        utime.sleep(0.2)
    
    
# -----------------------------------------------------------------------

# from machine import Pin, I2C
# import time
# import math

# class MPU6050:
#     def __init__(self, i2c, addr=0x68):
#         self.i2c = i2c
#         self.addr = addr
#         self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')  # Wake up MPU6050

#     def read_raw(self, reg):
#         data = self.i2c.readfrom_mem(self.addr, reg, 2)
#         val = int.from_bytes(data, 'big')
#         if val > 32767:
#             val -= 65536
#         return val

#     def get_accel_gyro(self):
#         ax = self.read_raw(0x3B) / 16384.0  # Accel scaling
#         ay = self.read_raw(0x3D) / 16384.0
#         az = self.read_raw(0x3F) / 16384.0
#         gx = self.read_raw(0x43) / 131.0    # Gyro scaling
#         gy = self.read_raw(0x45) / 131.0
#         gz = self.read_raw(0x47) / 131.0
#         return ax, ay, az, gx, gy, gz

# # Setup I2C
# i2c = I2C(0, scl=Pin(1), sda=Pin(0))
# mpu = MPU6050(i2c)

# alpha = 0.95
# calibration_time = 3  # seconds
# pitch = roll = yaw = 0.0
# gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0
# acc_pitch_offset = acc_roll_offset = 0.0

# print("Calibrating... Stay still for 3 seconds")
# calib_samples = 0
# start_time = time.ticks_ms()
# while time.ticks_diff(time.ticks_ms(), start_time) < calibration_time * 1000:
#     ax, ay, az, gx, gy, gz = mpu.get_accel_gyro()
#     gyro_bias_x += gx
#     gyro_bias_y += gy
#     gyro_bias_z += gz
#     acc_pitch_offset += math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
#     acc_roll_offset += math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
#     calib_samples += 1
#     time.sleep(0.01)

# # Average calibration
# gyro_bias_x /= calib_samples
# gyro_bias_y /= calib_samples
# gyro_bias_z /= calib_samples
# acc_pitch_offset /= calib_samples
# acc_roll_offset /= calib_samples

# print("Calibration Complete!")
# print("Gyro Bias -> X: {:.3f}, Y: {:.3f}, Z: {:.3f}".format(gyro_bias_x, gyro_bias_y, gyro_bias_z))
# print("Initial Pitch Offset: {:.3f}°, Roll Offset: {:.3f}°".format(acc_pitch_offset, acc_roll_offset))

# last_time = time.ticks_ms()

# while True:
#     now = time.ticks_ms()
#     dt = time.ticks_diff(now, last_time) / 1000.0  # Accurate dt in seconds
#     last_time = now

#     ax, ay, az, gx, gy, gz = mpu.get_accel_gyro()

#     # Remove gyro bias
#     gx -= gyro_bias_x
#     gy -= gyro_bias_y
#     gz -= gyro_bias_z

#     # Accelerometer angles
#     acc_pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2))) - acc_pitch_offset
#     acc_roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2))) - acc_roll_offset

#     # Gyro integration
#     pitch += gy * dt
#     roll  += gx * dt
#     yaw   += gz * dt  # WARNING: yaw will still drift without magnetometer

#     # Complementary filter fusion
#     pitch = alpha * pitch + (1 - alpha) * acc_pitch
#     roll  = alpha * roll + (1 - alpha) * acc_roll

#     print("Pitch: {:.2f}° | Roll: {:.2f}° | Yaw (Drift): {:.2f}°".format(pitch, roll, yaw))
#     time.sleep(0.005)  # Optional sleep to ease CPU, maintain responsiveness

# -------------------------------------------------------------------------------------

# from sensors import *
# priority_l = 1
# utime.sleep(1)
# move(1200)
# s = DistanceSensors()    #000 001 010 011 100 101 110 111
# def run():
#     move(550)
#     utime.sleep(1)
#     y,x = s.read_distances()
#     if x==[1,1,1]:
#         rotate(60)
#         utime.sleep(1)
#         rotate(60)
#     elif x==[1,1,0]:
#         rotate(60)
#     elif x==[0,1,1]:
#         rotate(-60)
#     elif x==[0,1,0]:
#         if priority_l:
#             rotate(-60)
#         else:
#             rotate(60)
#     utime.sleep(1)    
        
# while True: 
#     run()

# ----------------------------------------------------------------------------------------

################## CHECKING AVAILABLE I2C DEVICES FOR WIRE DEBUGGING #####################

# from machine import Pin, I2C

# i2c = I2C(1, scl=Pin(15), sda=Pin(14))  # Adjust pins
# print("Scanning I2C devices...")
# devices = i2c.scan()

# if devices:
#     print("I2C devices found:", [hex(dev) for dev in devices])
# else:
#     print("No I2C devices found! Check wiring.")

# ----------------------------------------------------------------------------------------
