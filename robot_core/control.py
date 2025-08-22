# robot_core/control.py
import log
import math

# --- Thruster Configuration ---
# This is a conceptual example. You'll need to define this based on your ROV.
# For a typical 6-DOF ROV, you might have 6 or 8 thrusters.
# This example assumes a simple setup:
# - 2 thrusters for forward/backward (surge)
# - 2 thrusters for up/down (heave)
# - 2 thrusters for yaw (rotation)
# - Strafing (sway) might be achieved by differential thrust or dedicated thrusters.

# For simplicity, let's assume thruster outputs are scaled from -255 to 255.
# This would be defined by your MotorController's expected input.
THRUSTER_MAX_OUTPUT = 255


logger = log.getLogger(__name__)
num_thrusters = 6
# You might initialize PID controllers here if you plan to use them for stability.
# e.g., depth_pid = PIDController(kp=..., ki=..., kd=...)
logger.info(
    f"ROVControlSystem initialized for {num_thrusters} thrusters.")


def calculate_thruster_outputs(x: float, y: float, z: float, yaw: float) -> list[int]:
    """
    Calculates individual thruster outputs based on desired movement.
    Args:
        x (float): Desired forward/backward thrust (-1.0 to 1.0).
        y (float): Desired strafe left/right thrust (-1.0 to 1.0).
        z (float): Desired up/down thrust (-1.0 to 1.0).
        yaw (float): Desired rotational thrust/rate (-1.0 to 1.0).
    Returns:
        list[int]: A list of thruster output values (e.g., scaled -255 to 255).
                    The order depends on your MotorController's thruster indexing.
    """
    # This is a VERY simplified mixing algorithm.
    # Real ROV control requires a proper thruster allocation matrix (TAM)
    # based on the geometry and orientation of each thruster.

    # --- Conceptual Thruster Mapping (Example for 6 thrusters) ---
    # Thruster IDs (indices for the output list):
    # 0: Forward Port (Front Left)
    # 1: Forward Starboard (Front Right)
    # 2: Vertical Port (Mid Left)
    # 3: Vertical Starboard (Mid Right)
    # 4: Yaw/Strafe Port (Rear Left or dedicated strafe)
    # 5: Yaw/Strafe Starboard (Rear Right or dedicated strafe)

    # Use float for intermediate calculations
    thrusters = [0.0] * num_thrusters

    # 1. Surge (Forward/Backward - X)
    # Assuming thrusters 0 and 1 contribute to surge
    surge_thrust_component = x * THRUSTER_MAX_OUTPUT
    thrusters[0] += surge_thrust_component
    thrusters[1] += surge_thrust_component

    # 2. Heave (Up/Down - Z)
    # Assuming thrusters 2 and 3 contribute to heave
    heave_thrust_component = z * THRUSTER_MAX_OUTPUT
    thrusters[2] += heave_thrust_component
    thrusters[3] += heave_thrust_component

    # 3. Yaw (Rotation - Yaw)
    # Assuming thrusters 0 & 1 (differentially) or 4 & 5 contribute to yaw
    yaw_thrust_component = yaw * THRUSTER_MAX_OUTPUT
    # Example: Differential thrust on forward thrusters for yaw
    # Port thruster more forward for right turn (positive yaw)
    thrusters[0] += yaw_thrust_component
    # Starboard thruster more reverse for right turn
    thrusters[1] -= yaw_thrust_component

    # (Alternative or additional yaw using dedicated thrusters 4 and 5)
    # If thrusters 4 and 5 are horizontal and opposing for yaw:
    # thrusters[4] += yaw_thrust_component
    # thrusters[5] -= yaw_thrust_component

    # 4. Sway (Strafe Left/Right - Y)
    # This often involves vectoring main thrusters or dedicated side thrusters.
    # For this simplified example, we might use thrusters 4 and 5 if available and oriented for sway.
    if num_thrusters >= 6:  # Assuming thrusters 4 and 5 are for sway
        sway_thrust_component = y * THRUSTER_MAX_OUTPUT
        # Example: Thrusters 4 (port side) and 5 (starboard side) push in the same direction for sway.
        # To strafe right (positive y), both might push right.
        thrusters[4] += sway_thrust_component
        thrusters[5] += sway_thrust_component

    # Normalize/Clamp thruster outputs to be within [-THRUSTER_MAX_OUTPUT, THRUSTER_MAX_OUTPUT]
    # A more sophisticated approach involves scaling down all thruster values proportionally
    # if any single thruster command exceeds the maximum, to maintain the desired maneuver shape.

    # Simple clamping:
    scaled_thrusters_int = []
    for t_val in thrusters:
        clamped_val = max(-THRUSTER_MAX_OUTPUT,
                          min(THRUSTER_MAX_OUTPUT, t_val))
        # Round before converting to int
        scaled_thrusters_int.append(int(round(clamped_val)))

    logger.debug(
        f"Input: x={x:.2f},y={y:.2f},z={z:.2f},yaw={yaw:.2f} -> Raw Thrusters: {[f'{t:.2f}' for t in thrusters]} -> Scaled Int: {scaled_thrusters_int}")
    return scaled_thrusters_int


def maintain_depth(current_depth: float, target_depth: float, dt: float) -> float:
    """
    Calculates Z-axis thrust correction to maintain a target depth.
    (This is a placeholder for where PID logic would go)
    Args:
        current_depth (float): Current depth reading from sensors.
        target_depth (float): Desired depth set by operator or AI.
        dt (float): Time step (delta time) since the last update, for PID derivative and integral terms.
    Returns:
        float: Required Z-axis thrust correction, normalized (-1.0 to 1.0).
    """
    # Example PID usage (you'd need a PIDController class)
    # if not hasattr('depth_pid'):
    #     # Initialize PID controller if it doesn't exist (e.g., with Kp, Ki, Kd gains from config)
    #     # depth_pid = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=target_depth, output_limits=(-1.0, 1.0))
    #     logger.warning("Depth PID controller not initialized.")
    #     return 0.0
    #
    # depth_pid.setpoint = target_depth # Update setpoint if it can change
    # error = target_depth - current_depth # Or pid.update(current_value, dt) if pid class handles error internally
    # correction = depth_pid.update(error, dt) # PID update method
    #
    # logger.debug(f"Depth Hold: Target={target_depth:.2f}, Current={current_depth:.2f}, Error={error:.2f}, Correction={correction:.2f}")
    # return max(-1.0, min(1.0, correction)) # Ensure output is clamped

    logger.debug(
        f"maintain_depth() called (PID not fully implemented). Target: {target_depth}, Current: {current_depth}")
    # Simple proportional control for demonstration:
    error = target_depth - current_depth
    kp = 0.5  # Proportional gain (needs tuning)
    correction = kp * error
    return max(-1.0, min(1.0, correction))  # Clamp output


# Example Usage
if __name__ == "__main__":
    # Enable debug logging for this test
    logging.basicConfig(level=logging.DEBUG)

    print("--- Testing ROV Control System ---")

    test_cases: list[dict] = [
        {"desc": "Forward (0.5)", "inputs": {
            "x": 0.5, "y": 0.0, "z": 0.0, "yaw": 0.0}},
        {"desc": "Backward (-0.3)", "inputs": {"x": -0.3,
                                               "y": 0.0, "z": 0.0, "yaw": 0.0}},
        {"desc": "Heave Up (0.7)", "inputs": {
            "x": 0.0, "y": 0.0, "z": 0.7, "yaw": 0.0}},
        {"desc": "Heave Down (-0.4)", "inputs": {"x": 0.0,
                                                 "y": 0.0, "z": -0.4, "yaw": 0.0}},
        {"desc": "Yaw Right (0.3)", "inputs": {
            "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.3}},
        {"desc": "Yaw Left (-0.6)", "inputs": {"x": 0.0,
                                               "y": 0.0, "z": 0.0, "yaw": -0.6}},
        {"desc": "Strafe Right (0.6) (6 thrusters)", "inputs": {
            "x": 0.0, "y": 0.6, "z": 0.0, "yaw": 0.0}},
        {"desc": "Strafe Left (-0.2) (6 thrusters)",
         "inputs": {"x": 0.0, "y": -0.2, "z": 0.0, "yaw": 0.0}},
        {"desc": "Combined: Fwd(0.5) & YawR(0.3)", "inputs": {
            "x": 0.5, "y": 0.0, "z": 0.0, "yaw": 0.3}},
        {"desc": "Combined: Fwd(0.8),StrafeR(0.4),HeaveU(0.2),YawL(-0.1)",
         "inputs": {"x": 0.8, "y": 0.4, "z": 0.2, "yaw": -0.1}},
        {"desc": "Exceed limits (x=1.0, yaw=1.0)", "inputs": {
            "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 1.0}},  # Should clamp
    ]

    for case in test_cases:
        print(f"\n{case['desc']}:")
        outputs = calculate_thruster_outputs(**case['inputs'])
        # The expected outputs below are based on the VERY simple mixing logic.
        # Your actual thruster configuration will yield different results.
        # E.g., for Forward (0.5), x=0.5*255=127.5. thrusters[0]=127.5, thrusters[1]=127.5
        # For Yaw Right (0.3), yaw=0.3*255=76.5. thrusters[0]+=76.5, thrusters[1]-=76.5
        # So Fwd(0.5)&YawR(0.3) -> t0=127.5+76.5=204, t1=127.5-76.5=51
        print(f"  Inputs: {case['inputs']}")
        print(f"  Outputs: {outputs}")

    print("\n--- Testing Depth Hold (Simple Proportional) ---")
    depth_correction1 = maintain_depth(
        current_depth=10.0, target_depth=12.0, dt=0.1)
    print(
        f"Maintain depth (10m -> 12m): Z-correction = {depth_correction1:.2f} (Should be positive for going down)")

    depth_correction2 = maintain_depth(
        current_depth=15.0, target_depth=12.0, dt=0.1)
    print(
        f"Maintain depth (15m -> 12m): Z-correction = {depth_correction2:.2f} (Should be negative for going up)")

    depth_correction3 = maintain_depth(
        current_depth=12.0, target_depth=12.0, dt=0.1)
    print(
        f"Maintain depth (12m -> 12m): Z-correction = {depth_correction3:.2f} (Should be zero)")
