from machine import Pin, I2C

sda = Pin(4)
scl = Pin(5)
bus_id = 0

i2c = I2C(id=bus_id, sda=sda, scl=scl)

detected_devices = i2c.scan()

for device_addr in detected_devices:
    print(f"Device with addr: {hex(device_addr)}")
