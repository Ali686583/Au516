from motor_driver import MotorDriverI2C
import utime

motor_driver = MotorDriverI2C()

current_battery_level = motor_driver.read_battery_level()
print(f"Battery level: {current_battery_level/1000}V")
                
#code ajouté
motor_driver.set_speed(20) #move forward
utime.sleep(2.5)    #pause the program for 2.5s
motor_driver.set_speed(0)    #stop the motors  