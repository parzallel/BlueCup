import threading

from pefile import retrieve_flags

from . import sensors, stabilize
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

ras_data_sample = [23]
def sensor_reader(data):
    """handles the reading from arduino"""
    try:
        # TODO : merge all the rp code in rp
        # data_from_sensors = ser.readline().decode('utf-8').strip().split(",")  # returned data from sensors
        # formatted_data = sensors.SensorFormatter(ras_data_sample    , data_from_sensors) # formatted data from sensors
        # # mpu = sensors.MPU(formatted_data)
        # # return stabilize.Stabilize(mpu).make_stable()
        # TODO : should return in the same format as controller
        print(data)

    except Exception as e:

        print(f"ERROR: receiving data from MPU as {e}")

def sensor_handler():
    return "m2=for FUCKS sake"
def serial_cycle():
    """handles the writing on arduino"""
    global latest_data
    while True:
        with lock:
            print(f"Sent: {latest_data}")

            data_from_sensors = ser.readline().decode('utf-8').strip().split(",")
            sensor_reader(data_from_sensors )
            print(data_from_sensors )
            if latest_data is not None:
                ser.write((latest_data +  "\n").encode())

        time.sleep(0.11)


def start_serial_thread():
    """Start background thread for serial communication."""
    writer_thread = threading.Thread(target=serial_cycle, daemon=True)
    writer_thread.start()
