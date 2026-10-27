from machine import Pin, I2C
import utime
import ustruct

#Register addresses
CHIP_ADDR = 0x34    #motor driver I2C address
ADC_BAT_ADDR = 0x00 #ADC battery sampling
MOTOR_TYPE_ADDR = 0x14  #Encoder motor type settings
MOTOR_ENCODER_POLARITY_ADDR = 0x15  #Encoder direction polarity settings
MOTOR_FIXED_SPEED_MOTOR_ADDR = 0x33  #Fixed speed control. It is closed-loop control. Unit: pulse count per 10ms. Range is usually [-50, 50]
MOTOR_ENCODER_TOTAL_ADDR = 0x3C #Total pulse values for the four encoded motors. For one encoder, distance = nb_pulses/nb_pulses_per_rev *pi*wheel_diameter


#Motor type values
MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT = 1
MOTOR_TYPE_N20 = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3


class MotorDriverI2C:
    def __init__(self):
        self.i2c = I2C(id=0, sda=Pin(4), scl=Pin(5))
        self.i2c.writeto_mem(CHIP_ADDR, MOTOR_TYPE_ADDR, bytearray([MOTOR_TYPE_JGB37_520_12V_110RPM]))
        utime.sleep(0.5)
        self.i2c.writeto_mem(CHIP_ADDR, MOTOR_ENCODER_POLARITY_ADDR, bytearray([0]))
        self.set_speed(0)
    
    
    def __del__(self):
        self.set_speed(0)
    
    
    def set_speed(self, speed):
        """ Set desired speed for motors M1 and M3 """
        speed = int(self.clamp(speed))
        self.i2c.writeto_mem(CHIP_ADDR, MOTOR_FIXED_SPEED_MOTOR_ADDR, bytearray([-speed, 0, speed, 0]))


    def read_encoders_values(self):
        """ Get total pulse values for motors M1 and M3. They are expressed in 32 bits. Around 1260 ticks per revolution """
        data = ustruct.unpack("iiii", bytes(self.i2c.readfrom_mem(CHIP_ADDR, MOTOR_ENCODER_TOTAL_ADDR, 16)))
        return data[0], data[2]   
    
    
    def read_battery_level(self):
        """ Get current battery voltage in mV """
        try:
            data = self.i2c.readfrom_mem(CHIP_ADDR, ADC_BAT_ADDR, 2)
            voltage = data[0] | (data[1] << 8)
            return voltage
        except Exception:
            return None
    
    
    def clamp(self, x, min_val=-10, max_val=50):
        return min(max(min_val, x), max_val)    
