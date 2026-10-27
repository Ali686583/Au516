from motor_driver import MotorDriverI2C
import utime

motor_driver = MotorDriverI2C()

current_battery_level = motor_driver.read_battery_level()
print(f"Battery level: {current_battery_level/1000}V")

enc_L, enc_R = motor_driver.read_encoders_values()
print(f"enc_l: {enc_L}, enc_r: {enc_R}")
                
motor_driver.set_speed(20) #move forward
utime.sleep(2.5)    #pause the program for 2.5s
motor_driver.set_speed(0)    #stop the motors

utime.sleep_ms(1)	#sleep is important to let the driver process the previous command
enc_L, enc_R = motor_driver.read_encoders_values()
print(f"enc_l: {enc_L}, enc_r: {enc_R}")


