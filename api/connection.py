import threading
from . import sensors
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


def sensor_reader():
    """handles the reading from arduino"""
    try:
        # DONE : merge  all the sensors codes in arduino
        # TODO : merge all the rp code in rp
        data_from_sensors = ser.readline().decode('utf-8').strip().split(",")
        data_cleaned = sensors.SensorFormatter(ardu_data=data_from_sensors , ras_data=None)
        sensors.MPU(data_cleaned.mpu_formatter())
        return None # TODO : should return in the same format as controller
    except Exception as e:

        print(f"ERROR: receiving data from MPU as {e}")


def serial_cycle():
    """handles the writing on arduino"""
    global latest_data
    while True:
        with lock:
            print(f"Sent: {latest_data}")
            if latest_data is not None:
                # sensor_reader()
                ser.write((latest_data + "\n").encode())

        time.sleep(0.11)


def start_serial_thread():
    """Start background thread for serial communication."""
    writer_thread = threading.Thread(target=serial_cycle, daemon=True)
    writer_thread.start()
