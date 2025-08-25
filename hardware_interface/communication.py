import serial
import time
import log
from config import ROV_SERIAL_PORT, ROV_BAUD_RATE, ROV_TIMEOUT, EOL

logger = log.getLogger(__name__)


"""
Handles serial communication with a TTY device.
"""
port = ROV_SERIAL_PORT
baudrate = ROV_BAUD_RATE
timeout = ROV_TIMEOUT
eol = EOL.encode('utf-8')  # Encode EOL to bytes
serial_connection = None


def connect():
    """
    Establishes the serial connection.
    """
    print("here")
    global serial_connection
    try:
        serial_connection = serial.Serial(
            port, baudrate, timeout=timeout
        )
        # Allow some time for the connection to establish
        time.sleep(2)
        if serial_connection.is_open:
            logger.info(
                f"Successfully connected to {port} at {baudrate} baud.")
        else:
            logger.error(f"Failed to open serial port {port}.")
    except serial.SerialException as e:
        logger.error(f"Error connecting to serial port {port}: {e}")
        serial_connection = None  # Ensure it's None if connection failed


def disconnect():
    """
    Closes the serial connection.
    """
    if serial_connection and serial_connection.is_open:
        try:
            serial_connection.close()
            logger.info(f"Disconnected from {port}.")
        except Exception as e:
            logger.error(f"Error disconnecting from {port}: {e}")
    serial_connection = None


def send_command(command_str):
    """
    Sends a command string over the serial port.
    Args:
        command_str (str): The command to send.
    Returns:
        bool: True if the command was sent successfully, False otherwise.
    """
    if not is_connected():
        logger.warning(
            f"Not connected to {port}. Cannot send command: {command_str}")
        # Optionally, try to reconnect:
        # logger.info("Attempting to reconnect...")
        # connect()
        # if not is_connected():
        #     return False
        return False

    try:
        full_command = command_str.encode('utf-8') + eol
        serial_connection.write(full_command)
        logger.debug(
            f"Sent to {port}: {full_command.decode('utf-8').strip()}")
        return True
    except serial.SerialTimeoutException:
        logger.error(
            f"Timeout writing to {port} for command: {command_str}")
    except serial.SerialException as e:
        logger.error(
            f"Serial error writing command '{command_str}' to {port}: {e}")
    except Exception as e:
        logger.error(
            f"Unexpected error writing command '{command_str}' to {port}: {e}")
    return False


def read_line(timeout_override=None):
    """
    Reads a line of text from the serial port until EOL or timeout.
    Args:
        timeout_override (float, optional): Specific timeout for this read operation.
    Returns:
        str or None: The line read (without EOL), or None if timeout or error.
    """
    if not is_connected():
        logger.warning(f"Not connected to {port}. Cannot read line.")
        return None

    original_timeout = serial_connection.timeout
    if timeout_override is not None:
        serial_connection.timeout = timeout_override

    try:
        line_bytes = serial_connection.readline()
        if line_bytes:
            decoded_line = line_bytes.decode(
                'utf-8', errors='ignore').strip()
            logger.debug(f"Read from {port}: {decoded_line}")
            return decoded_line
        else:
            # This can happen on timeout if no data is received
            logger.debug(f"Read timeout or no data from {port}.")
            return None
    except serial.SerialException as e:
        logger.error(f"Serial error reading line from {port}: {e}")
    except Exception as e:
        logger.error(
            f"Unexpected error reading line from {port}: {e}")
    finally:
        if timeout_override is not None:
            serial_connection.timeout = original_timeout  # Restore original
    return None


def read_bytes(num_bytes):
    """
    Reads a specific number of bytes from the serial port.
    Args:
        num_bytes (int): The number of bytes to read.
    Returns:
        bytes or None: The bytes read, or None if timeout or error.
    """
    if not is_connected():
        logger.warning(
            f"Not connected to {port}. Cannot read bytes.")
        return None
    try:
        data_bytes = serial_connection.read(num_bytes)
        if data_bytes:
            logger.debug(
                f"Read {len(data_bytes)} bytes from {port}.")
            return data_bytes
        else:
            logger.debug(
                f"Read timeout or no data (expected {num_bytes} bytes) from {port}.")
            return None
    except serial.SerialException as e:
        logger.error(
            f"Serial error reading {num_bytes} bytes from {port}: {e}")
    except Exception as e:
        logger.error(
            f"Unexpected error reading {num_bytes} bytes from {port}: {e}")
    return None


def flush_input():
    """ Clears input buffer. """
    if is_connected():
        serial_connection.reset_input_buffer()
        logger.debug(f"Input buffer flushed for {port}")


def flush_output():
    """ Clears output buffer. """
    if is_connected():
        serial_connection.reset_output_buffer()
        logger.debug(f"Output buffer flushed for {port}")


def is_connected():
    """
    Checks if the serial port is connected.
    """
    return serial_connection is not None and serial_connection.is_open


# connect()


# Example usage (for testing this module directly)
if __name__ == "__main__":
    # Replace with your ROV's actual serial port and baud rate
    # On Linux, it might be /dev/ttyUSB0, /dev/ttyACM0, etc.
    # On Windows, it might be COM3, COM4, etc.
    # Example, use a virtual serial port for testing if no hardware
    ROV_PORT = "/dev/ttyS10"
    # To create virtual serial ports for testing on Linux:
    # sudo apt-get install socat
    # socat -d -d pty,raw,echo=0 pty,raw,echo=0
    # This will output two port names like /dev/pts/X and /dev/pts/Y

    if is_connected():
        # Example: Send a command and try to read a response
        # THIS IS A PLACEHOLDER - use actual commands for your hardware
        test_command = "STATUS?"
        if send_command(test_command):
            print(f"Sent command: {test_command}")
            response = read_line()
            if response:
                print(f"Received response: {response}")
            else:
                print("No response or timeout.")
        else:
            print(f"Failed to send command: {test_command}")

        disconnect()
    else:
        print(f"Could not connect to {ROV_PORT}")
