from machine import Pin, I2C #type: ignore
import utime #type: ignore
from mpu6050 import MPU6050
from motors import MotorControl  # Import your motor class
from sensors import DistanceSensors  # Import your sensor class
from pid import PID  # Import your PID class

# Initialize MPU6050, Motors, and Distance Sensors
i2c = I2C(0, scl=Pin(1), sda=Pin(0))  # Adjust pins based on your board
mpu = MPU6050(i2c)
motors = MotorControl(speed=68)  # Fixed speed at 65

distance_sensors = DistanceSensors(i2c_bus=1, sda_pin=14, scl_pin=15)  # Adjust I2C bus and pins accordingly

# PID Controller for wall correction
distance_pid = PID(kp=0.4, ki=0.0, kd=0.2)

TARGET_DISTANCE = 35  # in millimeters, desired distance from both walls 

def move_continuous():
    """
    Moves the bot continuously while maintaining a distance of 3.5 cm from both walls using PID.
    """
    while True:
        distances,values = distance_sensors.read_distances()
        left_distance = distances[0]
        right_distance = distances[2]
        if values==[1,1,1]:
            motors.stop_motors()
            rotate()
            utime.sleep(1)
            rotate()
            utime.sleep(1)
            # break
        # Calculate error (difference from setpoint)
        error = (left_distance - right_distance)

        # Get PID correction value
        correction = distance_pid.compute(error)

        # Adjust motor directions based on correction
        left_speed = max(40, min(90, 65 + correction))
        right_speed = max(40, min(90, 65 - correction))

        # Move motors with adjusted speeds
        motors.move(1, 1, None)  # Move forward continuously
        motors.PWM1.duty_u16(int(left_speed * 65535 / 100))
        motors.PWM2.duty_u16(int(right_speed * 65535 / 100))

        print(f"Left: {left_distance} mm, Right: {right_distance} mm, Correction: {correction}, values: {values}")
        # utime.sleep_ms(50) 
        
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
