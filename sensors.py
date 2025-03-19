from machine import Pin, I2C  #type: ignore
import utime                  #type: ignore
from vl53l0x import VL53L0X  #type: ignore
class DistanceSensors:
    def __init__(self, i2c_bus=1, sda_pin=14, scl_pin=15, xshut_pins=[12,13,11,9]):
        self.i2c = I2C(i2c_bus, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.xshut_pins = [Pin(pin, Pin.OUT) for pin in xshut_pins]
        self.sensors = []
        self.offsets = [60,75,43]
        self.initialize_sensors()
    def initialize_sensors(self):
        """Initializes and assigns unique addresses to all sensors."""
        # Shutdown all sensors first
        for pin in self.xshut_pins:
            pin.value(0)
            utime.sleep_ms(100)
        # Power up sensors one by one and set unique addresses
        for i, pin in enumerate(self.xshut_pins):
            pin.value(1)
            utime.sleep_ms(100)
            sensor = VL53L0X(i2c=self.i2c)
            sensor.set_measurement_timing_budget(60000)
            sensor.set_address(0x30 + i)  # Assign unique address
            self.sensors.append(sensor)
    
    def read_distances(self):
        distances = [0,0,0]
        values = [0,0,0]
        """Reads distances from all sensors and returns a list of values."""
        for i,sensor in enumerate(self.sensors[1:]):
            distance = sensor.read()  # Read the distance
            reading = (distance-self.offsets[i])
            if reading<0: 
                reading = 0  
            distances[i] = reading
            if reading<=55:
                values[i] = 1
            # utime.sleep_ms(1)
        return distances, values
    
# -----------------------------------------------------------------------------------------------------

# from machine import Pin, I2C
# import time
# from vl53l0x import VL53L0X  # type: ignore

# class DistanceSensors:
#     def __init__(self, i2c_bus=1, sda_pin=14, scl_pin=15, xshut_pins=[13, 11, 9]):
#         self.i2c = I2C(i2c_bus, sda=Pin(sda_pin), scl=Pin(scl_pin))
#         self.xshut_pins = [Pin(pin, Pin.OUT) for pin in xshut_pins]
#         self.sensors = []
#         self.offsets = [0,0,0]  # Placeholder for calibration offsets
#         self.initialize_sensors()

#     def initialize_sensors(self):
#         """Initializes and assigns unique addresses to all sensors."""
#         # Shutdown all sensors first
#         for pin in self.xshut_pins:
#             pin.value(0)
#             time.sleep(0.2)

#         # Power up sensors one by one and set unique addresses
#         for i, pin in enumerate(self.xshut_pins):
#             pin.value(1)
#             time.sleep(0.2)
#             sensor = VL53L0X(i2c=self.i2c)
#             sensor.set_address(0x30 + i)  # Assign unique address
#             self.sensors.append(sensor)

#     def calibrate_sensors(self, samples=30):
#         """Calibrates each sensor by finding the minimum reading across multiple samples."""
#         print("Calibrating sensors...")

#         for i, sensor in enumerate(self.sensors):
#             print(f"Calibrating sensor {i+1}\n")
#             time.sleep(1.5)
#             min_value = float("inf")  # Start with a high value

#             for _ in range(samples):
#                 distance = sensor.read()
#                 min_value = min(min_value, distance)
#                 print(distance)
#                 time.sleep_ms(50)  # Small delay to stabilize readings

#             self.offsets[i] = min_value  # Store minimum as offset

#         print(self.offsets)
#         print("Calibration complete.")
        

#     def read_distances(self):
#         """Reads distances from all sensors, applies offset correction, and returns values."""
#         raw_distances = []
#         for sensor in self.sensors:
#             raw_distances.append(sensor.read())
#             time.sleep(0.05)
#         calibrated_distances = [max(0, raw - offset) for raw, offset in zip(raw_distances, self.offsets)]
#         binary_values = [1 if dist <= 80 else 0 for dist in calibrated_distances]

#         return calibrated_distances, binary_values


# from machine import Pin, I2C
# import utime
# from vl53l0x import VL53L0X  # type: ignore

# class DistanceSensors:
#     def __init__(self, i2c_bus=1, sda_pin=14, scl_pin=15, xshut_pins=[13, 11, 9]):
#         self.i2c = I2C(i2c_bus, sda=Pin(sda_pin), scl=Pin(scl_pin))
#         self.xshut_pins = [Pin(pin, Pin.OUT) for pin in xshut_pins]
#         self.num_sensors = len(xshut_pins)
#         self.shutdown_all_sensors()  # Ensure all sensors are initially off
#         self.offsets = [0,0,0]

#     def shutdown_all_sensors(self):
#         """Turn off all sensors by pulling their XSHUT pins low."""
#         for pin in self.xshut_pins:
#             pin.value(0)
#         utime.sleep_ms(100)

#     def read_distances(self):
#         """
#         Reads distances from all sensors one by one by activating them individually
#         using their XSHUT pins. Returns a list of distances.
#         """
#         distances = []
        
#         for i, xshut_pin in enumerate(self.xshut_pins):
#             # Activate the current sensor by setting its XSHUT pin high
#             xshut_pin.value(1)
#             utime.sleep_ms(10)  # Allow the sensor to power up
            
#             # Initialize the sensor with the default address (0x29)
#             try:
#                 sensor = VL53L0X(i2c=self.i2c)
#                 distance = sensor.read()  # Read the distance
#                 reading = (distance-self.offsets[i])
#                 if reading<0: 
#                     reading = 0
#                 # if reading<10:
#                 #     distances[i] = 1
                
#                 distances.append(reading)
#             except Exception as e:
#                 print(f"Error reading sensor {i}: {e}")
#                 distances.append(None)  # Append None if reading fails
            
#             # Deactivate the current sensor by setting its XSHUT pin low
#             xshut_pin.value(0)
#             utime.sleep_ms(10)  # Allow the sensor to shut down
        
#         return distances
