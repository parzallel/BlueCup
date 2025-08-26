import threading

import serial
import time

PORT = 'COM3'
BAUD = 9600

try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=1)
    time.sleep(2)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Error connecting to {PORT}: {e}")
    exit(1)
lock = threading.Lock()
latest_data = None

def mpu_data():
    try:
        line = ser.readline().decode('utf-8').strip().split(",")

        mpu = {"roll": line[0], "pitch": line[1], "yaw": line[2]}
        print(mpu)
    except Exception as e:
        print(f"ERROR: receiving data from MPU as {e}")


def serial_cycle():
    global latest_data
    while True:
        with lock:
            if latest_data is not None:
                ser.write((latest_data + "\n").encode())
                print(f"Sent: {latest_data}")
                mpu_data()
        time.sleep(0.11)


def start_serial_thread():
    """Start background thread for serial communication."""
    writer_thread = threading.Thread(target=serial_cycle, daemon=True)
    writer_thread.start()


