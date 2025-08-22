import cv2
import time
import log
from config import CAMERA_ID_OR_PIPELINE
"""
Manages a camera (USB or CSI via GStreamer on Raspberry Pi) using OpenCV.
"""

logger = log.getLogger(__name__)

# --- Example GStreamer Pipelines for Raspberry Pi CSI Cameras ---
# These can vary based on Pi version, camera module (v1, v2, HQ), and installed libraries.
# You might need to adjust `sensor-id` or other parameters.

# For libcamera stack (newer Raspberry Pi OS - Bullseye and later)
# This uses the `libcamerasrc` GStreamer element.
# `! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1`
RPI_CSI_LIBCAMERA_PIPELINE = (
    "libcamerasrc ! "
    "video/x-raw,width={width},height={height},framerate={fps}/1 ! "
    "videoconvert ! "
    "appsink drop=true max-buffers=1"  # drop=true and max-buffers=1 for latest frame
)
# A more complete example with BGR format for OpenCV:
RPI_CSI_LIBCAMERA_PIPELINE_BGR = (
    "libcamerasrc ! "
    # NV12 is a common CSI output
    "video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
)


# For older MMAL stack (legacy Raspberry Pi OS - Buster or older with legacy camera enabled)
# This uses `nvarguscamerasrc` (NVIDIA on Jetson often) or `v4l2src` (sometimes for CSI on Pi if configured that way)
# The more direct approach for MMAL was often through picamera library, but OpenCV can use GStreamer.
# A common GStreamer pipeline for CSI using V4L2 (if the CSI camera appears as /dev/videoX):
# `v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! appsink`
# However, for CSI direct access on older Pi OSes, `picamera` library was often easier than crafting a GStreamer pipeline for OpenCV.
# If using `libcamera` is an option (newer OS), it's generally preferred for CSI.

"""
Initializes the camera.

Args:
    camera_id_or_pipeline (int | str):
        - For USB cameras: Integer ID (e.g., 0, 1).
        - For CSI cameras (Raspberry Pi): A GStreamer pipeline string.
            You can use RPI_CSI_LIBCAMERA_PIPELINE_BGR.format(width=..., height=..., fps=...)
            or provide your own.
    width (int): Desired capture width.
    height (int): Desired capture height.
    fps (int): Desired capture frames per second.
    api_preference: OpenCV backend to prefer (e.g., cv2.CAP_V4L2, cv2.CAP_GSTREAMER).
                    cv2.CAP_ANY lets OpenCV decide.
"""
camera_id_or_pipeline = CAMERA_ID_OR_PIPELINE
cap = None
width = None
height = None
fps = None
api_preference = cv2.CAP_ANY  # Use OpenCV's default or let it decide


def _get_gst_pipeline() -> str | None:
    """
    Returns a GStreamer pipeline string if camera_id_or_pipeline is a known key
    or if it's already a GStreamer pipeline string.
    """
    if isinstance(camera_id_or_pipeline, str):
        if "!" in camera_id_or_pipeline:  # Likely already a GStreamer pipeline
            return camera_id_or_pipeline
        # Add known keys for predefined pipelines if desired
        # elif camera_id_or_pipeline.upper() == "CSI_LIBCAMERA":
        #     return RPI_CSI_LIBCAMERA_PIPELINE_BGR.format(width=width, height=height, fps=fps)
    return None  # Not a GStreamer pipeline string or known key


def connect():
    """
    Establishes the camera connection.
    """
    logger.info(
        f"Attempting to open camera: {camera_id_or_pipeline}")

    gst_pipeline = _get_gst_pipeline()

    try:
        if gst_pipeline:
            logger.info(f"Using GStreamer pipeline: {gst_pipeline}")
            cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        elif isinstance(camera_id_or_pipeline, int):
            logger.info(
                f"Using camera index: {camera_id_or_pipeline} with API: {api_preference}")
            cap = cv2.VideoCapture(
                camera_id_or_pipeline, api_preference)
        else:
            logger.error(
                f"Invalid camera_id_or_pipeline type: {type(camera_id_or_pipeline)}. Must be int or GStreamer string.")
            return

        if not cap.isOpened():
            logger.error(
                f"Failed to open camera: {camera_id_or_pipeline}")
            cap = None
            return

        # Try to set camera properties
        # Note: Not all cameras/backends support setting all properties,
        # and some properties must be set before opening via the GStreamer pipeline it
        if cap:
            if width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if fps:
                cap.set(cv2.CAP_PROP_FPS, fps)

            # Verify what was actually set (optional, reading can be slow)
            actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            logger.info(f"Camera opened. Requested: {width}x{height} @ {fps}fps. "
                        f"Actual: {int(actual_width)}x{int(actual_height)} @ {actual_fps:.2f}fps.")
            width = int(actual_width)  # Update with actual values
            height = int(actual_height)

    except Exception as e:
        logger.error(f"Exception during camera connection: {e}")
        if cap:
            cap.release()
        cap = None


def is_opened() -> bool:
    """Checks if the camera is opened and ready."""
    return cap is not None and cap.isOpened()


def read_frame() -> tuple[bool, cv2.typing.MatLike | None]:
    """
    Reads a frame from the camera.

    Returns:
        tuple[bool, MatLike | None]: (success, frame).
        'success' is True if a frame was read, False otherwise.
        'frame' is the captured OpenCV frame (NumPy array) if successful, else None.
    """
    if not is_opened():
        # logger.warning("Attempted to read frame, but camera is not open.")
        return False, None

    assert cap is not None
    ret, frame = cap.read()
    if not ret:
        # logger.warning("Failed to retrieve frame from camera.")
        return False, None
    return True, frame


def get_properties() -> dict:
    """
    Gets current camera properties.
    Note: Reading properties frequently can sometimes be slow or unreliable
            depending on the OpenCV backend and camera.
    """
    if not is_opened():
        return {
            "width": width,  # Return desired/last known if not open
            "height": height,
            "fps": fps,
            "is_opened": False
        }

    assert cap is not None
    return {
        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": cap.get(cv2.CAP_PROP_FPS),
        "brightness": cap.get(cv2.CAP_PROP_BRIGHTNESS),
        "contrast": cap.get(cv2.CAP_PROP_CONTRAST),
        "saturation": cap.get(cv2.CAP_PROP_SATURATION),
        "hue": cap.get(cv2.CAP_PROP_HUE),
        "gain": cap.get(cv2.CAP_PROP_GAIN),
        "exposure": cap.get(cv2.CAP_PROP_EXPOSURE),
        "is_opened": True
    }


def release():
    """
    Releases the camera resource.
    """
    if cap is not None:
        logger.info("Releasing camera.")
        cap.release()
        cap = None
    logger.info("Camera released.")


connect()

# Example Usage
if __name__ == "__main__":
    # --- Option 1: Try to use a generic USB camera (e.g., /dev/video0) ---
    print("Attempting to open USB camera (index 0)...")
    # For USB, you can specify desired resolution, though it might not always be respected perfectly.

    if is_opened():
        print("USB Camera opened successfully.")
        print(f"Properties: {get_properties()}")
        try:
            for i in range(60):  # Read 60 frames
                success, frame = read_frame()
                if success and frame is not None:
                    # cv2.imshow("USB Camera Frame", frame) # Requires a GUI environment
                    if i % 10 == 0:  # Log every 10 frames
                        print(f"Read USB frame {i+1}, shape: {frame.shape}")
                    # if cv2.waitKey(1) & 0xFF == ord('q'): # Allows q to quit if imshow is active
                    #     break
                    time.sleep(1/30)  # Simulate processing delay
                else:
                    print("Failed to read frame from USB camera.")
                    break
        finally:
            # cv2.destroyAllWindows() # If using imshow
            release()
            print("USB Camera released.")
    else:
        print("Failed to open USB camera.")

    print("\n" + "="*30 + "\n")

    # --- Option 2: Try to use a Raspberry Pi CSI camera via libcamera GStreamer pipeline ---
    # This part will likely only work on a Raspberry Pi with a CSI camera and libcamera correctly configured.
    print("Attempting to open Raspberry Pi CSI camera (using example libcamera pipeline)...")
    csi_width = 1280
    csi_height = 720
    csi_fps = 30

    # Note: platform.machine() can return 'aarch64' or 'armv7l' on Raspberry Pi
    # This is just an example; robust platform detection might be more involved.
    # if "arm" in platform.machine().lower() or "aarch64" in platform.machine().lower():
    # This example pipeline should be modified if your setup requires different parameters or elements.
    csi_pipeline = RPI_CSI_LIBCAMERA_PIPELINE_BGR.format(
        width=csi_width, height=csi_height, fps=csi_fps)

    # To test this part, you'd uncomment it and run on a Pi.
    # For non-Pi systems, this will likely fail to open.
    # csi_cam = CameraControllerCV(camera_id_or_pipeline=csi_pipeline,
    #                              width=csi_width, height=csi_height, fps=csi_fps,
    #                              api_preference=cv2.CAP_GSTREAMER)

    # if csi_cam.is_opened():
    #     print("CSI Camera opened successfully.")
    #     print(f"Properties: {csi_cam.get_properties()}")
    #     try:
    #         for i in range(60): # Read 60 frames
    #             success, frame = csi_cam.read_frame()
    #             if success:
    #                 # cv2.imshow("CSI Camera Frame", frame)
    #                 if i % 10 == 0:
    #                     print(f"Read CSI frame {i+1}, shape: {frame.shape}")
    #                 # if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 #     break
    #                 time.sleep(1/30)
    #             else:
    #                 print("Failed to read frame from CSI camera.")
    #                 break
    #     finally:
    #         # cv2.destroyAllWindows()
    #         csi_cam.release()
    #         print("CSI Camera released.")
    # else:
    #     print("Failed to open CSI camera (or not on a compatible Raspberry Pi setup).")
    print("Skipping CSI camera test for this example run unless explicitly uncommented and on a Pi.")
