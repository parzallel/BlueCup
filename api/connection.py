import threading
import log
import serial
import time
from . import sensors, stabilize
# Serial configuration
PORT = "COM5"
BAUD = 9600

# Configure logger

logger = log.getLogger(__name__)

# Shared state
lock = threading.Lock()
latest_data = None
data_from_sensors = None

# Initialize serial connection
try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=1)
    ser.readline()  # flush initial garbage
    logger.info("Warming up the motors...")
    time.sleep(5)
    logger.info(f"Connected to {PORT}")
except Exception as e:
    logger.error(f"Error connecting to {PORT}: {e}")
    exit(1)


def serial_cycle():
    """Continuously handle reading from sensors and writing to Arduino."""
    global latest_data, data_from_sensors

    while True:
        try:
            with lock:
                # Read sensor data
                raw_line = ser.readline().decode("utf-8").strip()
                if raw_line:
                    data_from_sensors = raw_line.split(",")

                # Write control data if available
                if latest_data is not None:
                    ser.write((latest_data + "\n").encode())
                    logger.debug(f"Sent: {latest_data}")

        except Exception as e:
            logger.warning(f"Serial cycle error: {e}")

        time.sleep(0.11)  # prevent flooding the serial line


def start_serial_thread():
    """Start background thread for serial communication."""
    writer_thread = threading.Thread(target=serial_cycle, daemon=True)
    writer_thread.start()
    logger.info("Serial communication thread started.")


def save_yaw():
    """Return yaw value from sensor data (if available)."""
    global data_from_sensors

    try:
        return data_from_sensors[2]
    except (IndexError, TypeError):
        logger.warning("Yaw not available yet (sensors warming up).")
        return None


def sensor_handler(saved_yaw_int):
    """Format sensor data and stabilize robot."""
    global data_from_sensors

    try:
        formatted_data = sensors.SensorFormatter(data_from_sensors)
        mpu = sensors.MPU(formatted_data.mpu_formatter())
        return stabilize.Stabilize(mpu, saved_yaw=saved_yaw_int).make_stable()
    except Exception as e:
        logger.error(f"Error receiving data from MPU: {e}")
        return None
