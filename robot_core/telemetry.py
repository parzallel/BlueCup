from typing import Any, Dict
from hardware_interface import communication
import time
import log

logger = log.getLogger(__name__)
logger.info("TelemetryCollector initialized.")


def collect_all_data(timeout=0.1, max_messages=10) -> dict:
    """Parses a raw sensor message and updates the global state dictionary."""
    start_time = time.monotonic()
    received_messages = 0
    _last_sensor_data: Dict[str, Any] = {
        'temperature': None,
        'depth': None,
        'imu': None,
        'battery': None,
        'rangefinder': None,
    }

    if not communication.is_connected():
        logger.warning(
            "Cannot collect telemetry: Sensor interface not available or not connected.")
        return _last_sensor_data

    while time.monotonic() - start_time < timeout and received_messages < max_messages:
        message = communication.read_line(timeout_override=0.01)

        if not message:
            break  # No more messages
        received_messages += 1

        try:
            prefix, payload = message.split(":", 1)

            if prefix == "TEMP":
                _last_sensor_data['temperature'] = float(payload)
                logger.debug(
                    f"Updated temperature: {_last_sensor_data['temperature']}°C")
            elif prefix == "DEPTH":
                _last_sensor_data['depth'] = float(payload)
                logger.debug(f"Updated depth: {_last_sensor_data['depth']}m")
            elif prefix == "BATT":
                _last_sensor_data['battery'] = float(payload)
                logger.debug(
                    f"Updated battery: {_last_sensor_data['battery']}V")
            elif prefix == "IMU":
                logger.debug(payload)
                imu_data = {}
                parts = payload.split(',')
                for part in parts:
                    key, value = part.split('=')
                    imu_data[key.lower()] = float(value)
                _last_sensor_data['imu'] = imu_data
                logger.debug(f"Updated IMU data: {imu_data}")
            elif prefix == "rf":
                _last_sensor_data['rangefinder'] = float(payload)
                logger.debug(
                    f"Updated rangefinder: {_last_sensor_data['rangefinder']}m")
            else:
                logger.warning(
                    f"Received message with unknown prefix: {prefix}")
                communication.flush_input()

        except (ValueError, IndexError) as e:
            pass
            # logger.error(f"Error parsing message '{message}': {e}")

    return _last_sensor_data


# Example Usage (requires a mock or real SensorInterface and SerialCommunicator)
if __name__ == "__main__":
    from hardware_interface import communication
    from config import ROV_SERIAL_PORT

    if communication.is_connected():
        print(f"Mock communicator connected to {ROV_SERIAL_PORT}")

        print("\nCollecting telemetry data (attempt 1):")
        telemetry = collect_all_data()
        for key, value in telemetry.items():
            print(f"  {key}: {value}")

        print("\nCollecting telemetry data (attempt 2):")
        telemetry = collect_all_data()
        for key, value in telemetry.items():
            print(f"  {key}: {value}")

        communication.disconnect()
    else:
        print(
            f"Failed to connect communicator to {ROV_SERIAL_PORT}. Ensure serial port is running.")
