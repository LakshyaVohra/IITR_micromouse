from machine import Pin, I2C #type: ignore
import utime #type: ignore
from mpu6050 import MPU6050
from motors import MotorControl  # Import your motor class

# Initialize MPU6050 and Motors globally
i2c = I2C(0, scl=Pin(1), sda=Pin(0))  # Adjust pins based on your board
mpu = MPU6050(i2c)
motors = MotorControl(speed=65)  # Adjust speed as needed

def rotate(target_angle=60):
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

def move(pulses = 550):
    """
    Moves the bot forward or backward based on direction and encoder pulses.
    dir1, dir2: 1 for forward, -1 for backward.
    pulses: Encoder count to move.
    """
    dir1,dir2 = (1, 1) if pulses > 0 else (-1, -1)
    print(f"Moving: Dir1={dir1}, Dir2={dir2}, Pulses={pulses}")
    motors.move(dir1,dir2, pulses)
    