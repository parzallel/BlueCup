# hardware_interface/sensors.py
from . import communication
from config import ROV_SERIAL_PORT, ROV_BAUD_RATE
import time  # For potential delays or retries
import log

logger = log.getLogger(__name__)


def _request_sensor_data(sensor_command: str, retries=2, delay=0.1) -> str | None:
    """
    Helper function to send a sensor command and get a response.
    Args:
        sensor_command (str): The command string to request sensor data.
        retries (int): Number of times to retry sending the command.
        delay (float): Delay in seconds between retries.
    Returns:
        str or None: The response string from the sensor, or None if failed.
    """
    for attempt in range(retries + 1):
        if communication.send_command(sensor_command):
            response = communication.read_line(
                timeout_override=0.5)  # Adjust timeout as needed
            if response:
                return response
            else:
                logger.warning(
                    f"No response for sensor command '{sensor_command}' on attempt {attempt + 1}")
        else:
            logger.warning(
                f"Failed to send sensor command '{sensor_command}' on attempt {attempt + 1}")

        if attempt < retries:
            time.sleep(delay)
    logger.error(
        f"Failed to get response for '{sensor_command}' after {retries + 1} attempts.")
    return None


def get_temperature() -> float | None:
    """
    Reads temperature from the ROV.
    Returns:
        float or None: Temperature in Celsius, or None if reading fails.
    """
    # **IMPORTANT**: Replace "S:TEMP?" with your hardware's actual command.
    # And parse the response accordingly.
    # Example response format: "TEMP:25.5"
    command = "S:TEMP?"
    response = _request_sensor_data(command)

    if response and response.startswith("TEMP:"):
        try:
            temp_str = response.split(":")[1]
            temperature = float(temp_str)
            logger.info(f"Temperature reading: {temperature}°C")
            return temperature
        except (IndexError, ValueError) as e:
            logger.error(
                f"Error parsing temperature from response '{response}': {e}")
    else:
        logger.warning(
            f"Unexpected or no response for temperature: {response}")
    return None


def get_depth() -> float | None:
    """
    Reads depth from the ROV.
    Returns:
        float or None: Depth in meters, or None if reading fails.
    """
    # **IMPORTANT**: Replace "S:DEPTH?" with your hardware's actual command.
    # Example response format: "DEPTH:10.2"
    command = "S:DEPTH?"
    response = _request_sensor_data(command)

    if response and response.startswith("DEPTH:"):
        try:
            depth_str = response.split(":")[1]
            depth = float(depth_str)
            logger.info(f"Depth reading: {depth}m")
            return depth
        except (IndexError, ValueError) as e:
            logger.error(
                f"Error parsing depth from response '{response}': {e}")
    else:
        logger.warning(f"Unexpected or no response for depth: {response}")
    return None


def get_imu_data() -> dict | None:
    """
    Reads IMU data (e.g., accelerometer, gyroscope, magnetometer).
    Returns:
        dict or None: A dictionary with IMU readings, or None if fails.
                        Example: {'ax': 0.1, 'ay': 0.0, 'az': 9.8, 'gx': 0, ...}
    """
    # **IMPORTANT**: Replace "S:IMU?" with your hardware's command.
    # Example response: "IMU:AX=0.1,AY=0.0,AZ=9.81,GX=0.1,GY=-0.2,GZ=0.05"
    command = "S:IMU?"
    response = _request_sensor_data(command)

    if response and response.startswith("IMU:"):
        try:
            data_str = response.split(":", 1)[1]
            imu_data = {}
            parts = data_str.split(',')
            for part in parts:
                key, value = part.split('=')
                imu_data[key.lower()] = float(value)
            logger.info(f"IMU data: {imu_data}")
            return imu_data
        except Exception as e:
            logger.error(
                f"Error parsing IMU data from response '{response}': {e}")
    else:
        logger.warning(f"Unexpected or no response for IMU: {response}")
    return None


def get_battery_voltage() -> float | None:
    """
    Reads battery voltage.
    Returns:
        float or None: Voltage, or None if reading fails.
    """
    command = "S:BATT?"  # **IMPORTANT**: Replace with actual command
    response = _request_sensor_data(command)
    # Example response: "BATT:12.35"
    if response and response.startswith("BATT:"):
        try:
            voltage_str = response.replace("BATT:", "")
            voltage = float(voltage_str)
            logger.info(f"Battery voltage: {voltage}V")
            return voltage
        except ValueError as e:
            logger.error(
                f"Error parsing battery voltage from '{response}': {e}")
    else:
        logger.warning(
            f"Unexpected or no response for battery voltage: {response}")
    return None


# Example Usage
if __name__ == "__main__":
    # from communication import SerialCommunicator # For standalone test

    if communication.is_connected():
        print("Requesting Temperature...")
        temp = get_temperature()
        if temp is not None:
            print(f"  Temperature: {temp}°C")
        else:
            print("  Failed to get temperature.")

        print("\nRequesting Depth...")
        depth = get_depth()
        if depth is not None:
            print(f"  Depth: {depth}m")
        else:
            print("  Failed to get depth.")

        print("\nRequesting IMU Data...")
        imu = get_imu_data()
        if imu:
            print(f"  IMU: {imu}")
        else:
            print("  Failed to get IMU data.")

        print("\nRequesting Battery Voltage...")
        batt = get_battery_voltage()
        if batt:
            print(f"  Battery: {batt}V")
        else:
            print("  Failed to get battery voltage.")

        communication.disconnect()
    else:
        print(f"Failed to connect to {ROV_SERIAL_PORT} for sensor readings.")
