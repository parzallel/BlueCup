import threading
import time
import serial

PORT = 'COM3'
BAUD = 9600

try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=1)
    time.sleep(4)
    print(f"WARNING: connection to {PORT} was successful")
except Exception as e:
    print(f"Error connecting to {PORT}: {e}")
    exit(1)

print("----CONNECTED-----")
lock = threading.Lock()

def mpu_data():
    """Read a single line of MPU data after writing a command"""
    try:
        with lock:
            line = ser.readline().decode('utf-8').strip().split(",")
        if len(line) >= 3:
            mpu = {"roll": line[0], "pitch": line[1], "yaw": line[2]}
            print(mpu)
    except Exception as e:
        print(f"ERROR: receiving data from MPU - {e}")

def serial_cycle(command):
    """Continuously: write command → read response → wait"""
    while True:
        with lock:
            if command:
                ser.write((command + "\n").encode())
        mpu_data()  # only read AFTER writing
        time.sleep(0.11)  # pacing

# Start the thread
writer_thread = threading.Thread(target=serial_cycle, args=("TEST_CMD",), daemon=True)
writer_thread.start()
