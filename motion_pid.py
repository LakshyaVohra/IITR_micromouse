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
distance_pid = PID(0.4,0,0.3)

TARGET_DISTANCE = 35

def move_with_pid_pulses(target_pulses=550):
    """
    Move forward for a specific number of encoder pulses while maintaining distance from walls using PID.
    """
    # Reset encoders
    motors.Encodervalue1 = 0
    motors.Encodervalue2 = 0

    # Start moving forward
    motors.move(1, 1)
    

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
    motors.move(a, b)  # Start rotation

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