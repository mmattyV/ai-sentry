"""
Servo Controller Module for Laser Turret

Implements a proportional (P) controller to convert pixel error to servo angle adjustments.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class ControllerOutput:
    """Output from the servo controller."""
    pan_angle: int
    tilt_angle: int
    pan_error: float  # Pixel error in X
    tilt_error: float  # Pixel error in Y
    error_magnitude: float  # Combined error magnitude


class ServoController:
    """
    PD (Proportional-Derivative) controller for pan-tilt servo control.

    Converts pixel error (target position - frame center) to servo angle adjustments.
    The derivative term helps prevent overshoot caused by camera latency.

    Coordinate system:
    - Pan servo: increase angle = move RIGHT (positive X direction)
    - Tilt servo: increase angle = move DOWN (positive Y direction)

    So if target is to the RIGHT of center (positive X error), we INCREASE pan angle.
    If target is BELOW center (positive Y error), we INCREASE tilt angle.
    """

    def __init__(
        self,
        frame_size: Tuple[int, int] = (320, 240),
        fov: Tuple[float, float] = (70.0, 55.0),  # Horizontal and vertical FOV in degrees
        kp: float = 0.3,
        kd: float = 0.5,  # Derivative gain to dampen oscillation
        deadzone: float = 10.0,
        pan_limits: Tuple[int, int] = (0, 180),
        tilt_limits: Tuple[int, int] = (0, 180),
        initial_pan: int = 90,
        initial_tilt: int = 90
    ):
        """
        Initialize the servo controller.

        Args:
            frame_size: Camera frame dimensions (width, height).
            fov: Camera field of view (horizontal, vertical) in degrees.
            kp: Proportional gain (0.1-1.0, higher = faster but may oscillate).
            kd: Derivative gain (0.1-1.0, higher = more damping, prevents overshoot).
            deadzone: Pixel error below which we consider "on target".
            pan_limits: (min, max) angles for pan servo.
            tilt_limits: (min, max) angles for tilt servo.
            initial_pan: Starting pan angle.
            initial_tilt: Starting tilt angle.
        """
        self.frame_width, self.frame_height = frame_size
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)

        self.fov_h, self.fov_v = fov
        self.kp = kp
        self.kd = kd
        self.deadzone = deadzone

        self.pan_min, self.pan_max = pan_limits
        self.tilt_min, self.tilt_max = tilt_limits

        self.current_pan = initial_pan
        self.current_tilt = initial_tilt

        # Previous errors for derivative calculation
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        # Precompute degrees per pixel
        self.deg_per_pixel_x = self.fov_h / self.frame_width
        self.deg_per_pixel_y = self.fov_v / self.frame_height

    def compute(self, target_position: Tuple[int, int]) -> ControllerOutput:
        """
        Compute servo angles to move toward target using PD control.

        Args:
            target_position: (x, y) pixel coordinates of target in frame.

        Returns:
            ControllerOutput with new servo angles and error information.
        """
        target_x, target_y = target_position
        center_x, center_y = self.frame_center

        # Calculate pixel error (positive = target is right/below center)
        error_x = target_x - center_x
        error_y = target_y - center_y

        # Calculate error magnitude
        error_magnitude = (error_x**2 + error_y**2) ** 0.5

        # Calculate derivative (rate of change of error)
        # Negative derivative means error is decreasing (approaching target)
        d_error_x = error_x - self.prev_error_x
        d_error_y = error_y - self.prev_error_y

        # Store current error for next iteration
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # PD control: output = Kp * error + Kd * derivative
        # The derivative term dampens movement when approaching target
        pan_output = self.kp * error_x + self.kd * d_error_x
        tilt_output = self.kp * error_y + self.kd * d_error_y

        # Convert to angle delta
        pan_delta = pan_output * self.deg_per_pixel_x
        tilt_delta = tilt_output * self.deg_per_pixel_y

        # Apply deadzone - don't adjust if error is small enough
        if abs(error_x) < self.deadzone:
            pan_delta = 0
        if abs(error_y) < self.deadzone:
            tilt_delta = 0

        # Update servo angles
        # Pan: positive error (target on right) -> increase angle (move right)
        # Tilt: positive error (target below) -> increase angle (move down)
        new_pan = self.current_pan + pan_delta
        new_tilt = self.current_tilt + tilt_delta

        # Clamp to limits
        new_pan = max(self.pan_min, min(self.pan_max, new_pan))
        new_tilt = max(self.tilt_min, min(self.tilt_max, new_tilt))

        # Update current positions
        self.current_pan = new_pan
        self.current_tilt = new_tilt

        return ControllerOutput(
            pan_angle=int(round(new_pan)),
            tilt_angle=int(round(new_tilt)),
            pan_error=error_x,
            tilt_error=error_y,
            error_magnitude=error_magnitude
        )

    def set_position(self, pan: int, tilt: int):
        """Directly set servo positions."""
        self.current_pan = max(self.pan_min, min(self.pan_max, pan))
        self.current_tilt = max(self.tilt_min, min(self.tilt_max, tilt))

    def get_position(self) -> Tuple[int, int]:
        """Get current servo positions."""
        return int(round(self.current_pan)), int(round(self.current_tilt))

    def center(self):
        """Move to center position."""
        self.current_pan = (self.pan_min + self.pan_max) // 2
        self.current_tilt = (self.tilt_min + self.tilt_max) // 2

    def set_kp(self, kp: float):
        """Update proportional gain."""
        self.kp = max(0.01, min(2.0, kp))

    def set_deadzone(self, deadzone: float):
        """Update deadzone threshold."""
        self.deadzone = max(0, deadzone)

    def update_frame_size(self, width: int, height: int):
        """Update frame dimensions (call if camera resolution changes)."""
        self.frame_width = width
        self.frame_height = height
        self.frame_center = (width // 2, height // 2)
        self.deg_per_pixel_x = self.fov_h / width
        self.deg_per_pixel_y = self.fov_v / height

    def is_on_target(self, threshold: float = None) -> bool:
        """Check if currently on target (within deadzone)."""
        if threshold is None:
            threshold = self.deadzone
        # This requires the last computed error - store it
        return hasattr(self, '_last_error') and self._last_error < threshold


class PIDController:
    """
    PID controller for smoother servo control (optional upgrade).

    Adds integral and derivative terms for:
    - I: Eliminates steady-state error
    - D: Dampens oscillations for smoother movement
    """

    def __init__(
        self,
        kp: float = 0.3,
        ki: float = 0.01,
        kd: float = 0.1,
        integral_limit: float = 50.0
    ):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            integral_limit: Maximum integral accumulator value.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error: float, dt: float = 1/30) -> float:
        """
        Compute PID output.

        Args:
            error: Current error value.
            dt: Time delta since last computation.

        Returns:
            Control output value.
        """
        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        # Derivative
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        self.last_error = error

        return p_term + i_term + d_term

    def reset(self):
        """Reset integral and derivative state."""
        self.integral = 0.0
        self.last_error = 0.0


if __name__ == "__main__":
    # Test controller
    controller = ServoController(
        frame_size=(320, 240),
        kp=0.5,
        deadzone=10
    )

    print("Testing servo controller...")
    print(f"Frame center: {controller.frame_center}")
    print(f"Initial position: {controller.get_position()}")
    print()

    # Simulate target moving from right to center
    test_positions = [
        (280, 120),  # Far right, center height
        (240, 120),  # Right of center
        (180, 120),  # Slightly right
        (160, 120),  # Near center (in deadzone)
        (100, 80),   # Left and up
        (50, 200),   # Far left and down
    ]

    for pos in test_positions:
        output = controller.compute(pos)
        print(f"Target: {pos}")
        print(f"  Error: ({output.pan_error:.1f}, {output.tilt_error:.1f})")
        print(f"  Error magnitude: {output.error_magnitude:.1f}")
        print(f"  Servo angles: Pan={output.pan_angle}, Tilt={output.tilt_angle}")
        print()

