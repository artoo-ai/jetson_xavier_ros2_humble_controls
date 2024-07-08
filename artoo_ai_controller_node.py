import rclpy
from rclpy.node import Node
import smbus2

class PwmControllerNode(Node):
    """
    2048 is 50% duty cycle which is half speed
    4095 is 100% duty cycle which is full speed
    """
    
    PCA9685_ADDRESS = 0x40
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_OFF_L = 0x08

    def __init__(self, i2c_bus_number):
        super().__init__('pwm_controller')
        # PCA9685 is Address 0x40 on Bus 8 on the Jetson Xavier.

        # Create an smbus2 I2C object for bus 8
        self.i2c_bus = smbus2.SMBus(8)

        # Define the I2C address of the PCA9685
        self.PCA9685_ADDRESS = 0x40
        
        # Account for motor not responding the same
        self.FORWARD_LEFT_OFFSET = 0
        self.FORWARD_RIGHT_OFFSET = 0
        self.REVERSE_LEFT_OFFSET = 0
        self.REVERSE_RIGHT_OFFSET = 0
        
        self.FORWARD_LEFT_CH = 0            # Foward Left Channel 
        self.FORWARD_RIGHT_CH = 2           # Foward Right Channel
        self.REVERSE_LEFT_CH = 1            # Reverse Left Channel
        self.REVERSE_RIGHT_CH = 3           # Reverse Right Channel
        
        # 2048 is 50% duty cycle which is half speed
        # 4095 is 100% duty cycle which is full speed
        self.MAX_SPEED = 4095
        self.MIN_SPEED = 0 
        
        # Init Settings
        # Perform the equivalent i2cset commands
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0x00)  # i2cset -y 8 0x40 0x00 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0xFE, 0x65)  # i2cset -y 8 0x40 0xFE 0x65
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0xA0)  # i2cset -y 8 0x40 0x00 0xA0

        # Channel 0
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x06, 0x00)  # i2cset -y 8 0x40 0x06 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x07, 0x00)  # i2cset -y 8 0x40 0x07 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x08, 0x00)  # i2cset -y 8 0x40 0x08 0xFA
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x09, 0x00)  # i2cset -y 8 0x40 0x09 0x00

        # Channel 1
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x0A, 0x00)  # i2cset -y 8 0x40 0x06 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x0B, 0x00)  # i2cset -y 8 0x40 0x07 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x0C, 0x00)  # i2cset -y 8 0x40 0x08 0xFA
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x0D, 0x00)  # i2cset -y 8 0x40 0x09 0x00

        # Channel 2
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x0E, 0x00) # i2cset -y 8 0x40 0x0E 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x0F, 0x00) # i2cset -y 8 0x40 0x0F 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x10, 0x00) # i2cset -y 8 0x40 0x10 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x11, 0x00) # i2cset -y 8 0x40 0x11 0x00

        # Channel 3
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x12, 0x00) # i2cset -y 8 0x40 0x12 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x13, 0x00) # i2cset -y 8 0x40 0x13 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x14, 0x00) # i2cset -y 8 0x40 0x14 0x00
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, 0x15, 0x00) # i2cset -y 8 0x40 0x15 0x00
        
    def set_pwm(self, channel, on, off):
        """
        Function to set PWM for a specific channel
        On is typically always 0.
        Off is 0 - 4095.  
        
        0    is  0% duty cycle which is stop
        2048 is 50% duty cycle which is half speed
        4095 is 100% duty cycle which is full speed
        
        Monitor motor temperatures.  Even at 50% the 12V
        temperatures were getting high in a short period of time.
        6V seems like a good temperature
        """
        
        # Calculate the base register address for the specified channel
        base_address = 0x06 + 4 * channel
        # Write the ON and OFF times to the appropriate registers
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, base_address, on & 0xFF)         # LEDn_ON_L
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, base_address + 1, on >> 8)       # LEDn_ON_H
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, base_address + 2, off & 0xFF)    # LEDn_OFF_L
        self.i2c_bus.write_byte_data(self.PCA9685_ADDRESS, base_address + 3, off >> 8)      # LEDn_OFF_H
        
    def drive_forward(self, speed: int):
        """
        Drive Forward.  First stop reverse.
        Use any offsets for the motors difference
        in speed.
        """
        # First set Reverse speed to 0
        self.set_pwm(self.REVERSE_LEFT_CH, 0, 0)
        self.set_pwm(self.REVERSE_RIGHT_CH, 0, 0)
        
        # Verify min/max speed
        if (speed < self.MIN_SPEED):
            speed = self.MIN_SPEED
        elif (speed > self.MAX_SPEED):
            speed = self.MAX_SPEED
        
        # Set speed for forward
        # Use offset to equilize the motors
        on_time = 0
        off_time_left = speed + self.FORWARD_LEFT_OFFSET
        off_time_right = speed + self.FORWARD_RIGHT_OFFSET
        
        print(f'Forward Speed: {speed} {off_time_left} {off_time_right}')
        self.set_pwm(self.FORWARD_LEFT_CH, on_time, off_time_left)
        self.set_pwm(self.FORWARD_RIGHT_CH, on_time, off_time_right)
        
    def drive_reverse(self, speed: int):
        """
        Drive Reverse.  First stop forward.
        Use any offsets for the motors difference
        in speed.
        """
        # First set Forward speed to 0
        self.set_pwm(self.FORWARD_LEFT_CH, 0, 0)
        self.set_pwm(self.FORWARD_RIGHT_CH, 0, 0)
        
        # Verify min/max speed
        if (speed < self.MIN_SPEED):
            speed = self.MIN_SPEED
        elif (speed > self.MAX_SPEED):
            speed = self.MAX_SPEED
        
        # Set speed for reverse
        # Use offset to equilize the motors
        on_time = 0
        off_time_left = speed + self.FORWARD_LEFT_OFFSET
        off_time_right = speed + self.FORWARD_RIGHT_OFFSET
        
        print(f'Reverse Speed: {speed} {off_time_left} {off_time_right}')
        self.set_pwm(self.REVERSE_LEFT_CH, on_time, off_time_left)
        self.set_pwm(self.REVERSE_RIGHT_CH, on_time, off_time_right)
        
    def drive_stop(self):
        """
        Stop driving.
        
        self.drive_forward() automatically calls self.drive_reverse(0)
        so you do not need to call it also
        """
        # First set Forward speed to 0
        self.drive_forward(0)
        