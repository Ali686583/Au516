from machine import Pin, PWM


def map(x, min_i, max_i, min_o, max_o):
    """ Re-maps a number from one range to another """    
    return int( (x - min_i) * (max_o - min_o)/(max_i - min_i) + min_o )


def clamp(val, min_val=1000, max_val=2000):
    """ Return a value within the range [min_val, max_val] """
    return min(max(min_val, val), max_val)


def pulse_to_duty(pulse):
    """ Return the corresponding duty cycle (expressed in 16bits) of a pulse """
    pulse = clamp(pulse)    #make sure pulse is within [1000, 2000]
    return map(pulse, 0, 20000, 0, 65536)
    

SERVO_PIN = 18

pwm = PWM(Pin(SERVO_PIN)) #Construct PWM object
pwm.freq(50) #Set the PWM frequency to 50Hz
pwm.duty_u16(pulse_to_duty(1750)) #Set the target pulse