from machine import Pin, I2C
import utime
import ustruct

#Register addresses
CHIP_ADDR = 0x34    #motor driver I2C address
ADC_BAT_ADDR = 0x00 #ADC battery sampling
MOTOR_TYPE_ADDR = 0x14  #Encoder motor type settings
MOTOR_ENCODER_POLARITY_ADDR = 0x15  #Encoder direction polarity settings
MOTOR_FIXED_SPEED_MOTOR_ADDR = 0x33  #Fixed speed control. It is closed-loop control. Unit: pulse count per 10ms. Range is usually [-50, 50]

#Motor type values
MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT = 1
MOTOR_TYPE_N20 = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3