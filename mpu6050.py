import utime

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        
        # Wake up the sensor
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        
        # Configure the sensor for better accuracy
        self._configure_sensor()
        
        # Calibration variables
        self.ax_offset = 0
        self.ay_offset = 0
        self.az_offset = 0
        self.gx_offset = 0
        self.gy_offset = 0
        self.gz_offset = 0
        
        # Perform calibration
        self.calibrate()

    def _configure_sensor(self):
        """
        Configure the MPU6050 for optimal performance:
        - Set accelerometer range to ±2g
        - Set gyroscope range to ±250°/s
        - Enable DLPF (Digital Low-Pass Filter) for smoothing
        """
        # Accelerometer: ±2g (0x00), Gyroscope: ±250°/s (0x00)
        self.i2c.writeto_mem(self.addr, 0x1C, b'\x00')  # Accel config
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x00')  # Gyro config
        
        # Enable DLPF with a bandwidth of 42Hz (for smoother readings)
        self.i2c.writeto_mem(self.addr, 0x1A, b'\x03')  # DLPF_CFG = 3

    def read_raw_data(self, reg):
        """
        Read 16-bit signed data from the specified register.
        """
        high = self.i2c.readfrom_mem(self.addr, reg, 1)
        low = self.i2c.readfrom_mem(self.addr, reg + 1, 1)
        value = (high[0] << 8) | low[0]
        if value > 32767:
            value -= 65536
        return value

    def calibrate(self, samples=500):
        """
        Calibrate the sensor by averaging readings at rest.
        This helps remove bias in accelerometer and gyroscope readings.
        """
        print("Calibrating MPU6050...")
        ax_sum, ay_sum, az_sum = 0, 0, 0
        gx_sum, gy_sum, gz_sum = 0, 0, 0
        
        for _ in range(samples):
            raw_accel = self.get_raw_accel()
            raw_gyro = self.get_raw_gyro()
            
            ax_sum += raw_accel["ax"]
            ay_sum += raw_accel["ay"]
            az_sum += raw_accel["az"]
            
            gx_sum += raw_gyro["gx"]
            gy_sum += raw_gyro["gy"]
            gz_sum += raw_gyro["gz"]
            
            utime.sleep_ms(1)  # Small delay between readings
        
        # Calculate offsets
        self.ax_offset = ax_sum / samples
        self.ay_offset = ay_sum / samples
        self.az_offset = (az_sum / samples) - 1.0  # Gravity compensation
        
        self.gx_offset = gx_sum / samples
        self.gy_offset = gy_sum / samples
        self.gz_offset = gz_sum / samples
        
        print("Calibration complete.")

    def get_raw_accel(self):
        """
        Get raw accelerometer readings without applying offsets.
        """
        ax = self.read_raw_data(0x3B) / 16384.0  # Convert to g
        ay = self.read_raw_data(0x3D) / 16384.0
        az = self.read_raw_data(0x3F) / 16384.0
        return {"ax": ax, "ay": ay, "az": az}

    def get_raw_gyro(self):
        """
        Get raw gyroscope readings without applying offsets.
        """
        gx = self.read_raw_data(0x43) / 131.0  # Convert to deg/s
        gy = self.read_raw_data(0x45) / 131.0
        gz = self.read_raw_data(0x47) / 131.0
        return {"gx": gx, "gy": gy, "gz": gz}

    def get_accel(self):
        """
        Get calibrated accelerometer readings.
        """
        raw = self.get_raw_accel()
        ax = raw["ax"] - self.ax_offset
        ay = raw["ay"] - self.ay_offset
        az = raw["az"] - self.az_offset
        return {"ax": ax, "ay": ay, "az": az}

    def get_gyro(self):
        """
        Get calibrated gyroscope readings.
        """
        raw = self.get_raw_gyro()
        gx = raw["gx"] - self.gx_offset
        gy = raw["gy"] - self.gy_offset
        gz = raw["gz"] - self.gz_offset
        return {"gx": gx, "gy": gy, "gz": gz}

    def get_temp(self):
        """
        Get temperature reading in °C.
        """
        temp = self.read_raw_data(0x41) / 340.0 + 36.53
        return temp