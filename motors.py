from machine import Pin, PWM  # type: ignore
import utime  # type: ignore


class MotorControl:
    def __init__(self, speed=80):
        # Motor 1 Pins
        self.EncoderPinA1 = Pin(2, Pin.IN)
        self.EncoderPinB1 = Pin(3, Pin.IN)
        self.ForwardPin1 = Pin(22, Pin.OUT)
        self.BackwardPin1 = Pin(17, Pin.OUT)
        self.PWM1 = PWM(Pin(16))

        # Motor 2 Pins
        self.EncoderPinA2 = Pin(4, Pin.IN)
        self.EncoderPinB2 = Pin(5, Pin.IN)
        self.ForwardPin2 = Pin(20, Pin.OUT)
        self.BackwardPin2 = Pin(19, Pin.OUT)
        self.PWM2 = PWM(Pin(21))

        self.STBY = Pin(18, Pin.OUT)  # Standby pin
        self.STBY.value(1)

        # Set PWM frequency (10 kHz for motor control)
        self.PWM1.freq(1000)
        self.PWM2.freq(1000)

        # Encoder values
        self.Encodervalue1 = 0
        self.Encodervalue2 = 0
        self.speed = speed

        # Attach interrupts for encoders
        self.EncoderPinA1.irq(trigger=Pin.IRQ_RISING, handler=self.update_encoder1)
        self.EncoderPinA2.irq(trigger=Pin.IRQ_RISING, handler=self.update_encoder2)

        print("Motor system initialized.")

    # Encoder interrupt function for Motor 1
    def update_encoder1(self,Pin):
        self.Encodervalue1 += 1 if self.EncoderPinA1.value() == self.EncoderPinB1.value() else -1

    # Encoder interrupt function for Motor 2
    def update_encoder2(self,Pin):
        self.Encodervalue2 += 1 if self.EncoderPinA2.value() != self.EncoderPinB2.value() else -1

    # Unified move function
    def move(self, dir1, dir2):
        
        self.ForwardPin1.value(1 if dir1 == 1 else 0)
        self.BackwardPin1.value(1 if dir1 == -1 else 0)
        self.PWM1.duty_u16(int(self.speed * 65535 / 100) if dir1 else 0)

        self.ForwardPin2.value(1 if dir2 == 1 else 0)
        self.BackwardPin2.value(1 if dir2 == -1 else 0)
        self.PWM2.duty_u16(int(self.speed * 65535 / 100) if dir2 else 0)

    # Stop motors
    def stop_motors(self):
        self.PWM1.duty_u16(0)
        self.PWM2.duty_u16(0)
        print("Motors stopped.")
