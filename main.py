# Importing Libraries
# 921 counts = 20cm
import serial
import time
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.5)
    data = arduino.readline()
    return data
while True:
    num = input("Enter Setpoint in Cm(0 to 20): ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value