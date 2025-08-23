import time

import serial
PORT = 'COM3'
BAUD = 9600

try :
    ser = serial.Serial(port=PORT , baudrate= BAUD , timeout= 1)
    time.sleep(4)
    print(f"WARNING :connection to the {PORT} was clear")

except Exception as e :
    print(f"there was an error to the {PORT} : {e}")

print("----CONNECTED-----")
