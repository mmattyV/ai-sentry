"""
Visualization Module for Laser Turret

Provides real-time OpenCV window displaying:
- Camera feed with bounding boxes
- Current target highlighted
- Crosshairs and debug information
"""

import cv2
import numpy as np
from typing import List, Optional, Tuple
from detector import Detection
from targeting import TargetState


class Visualizer:
    """
    Real-time visualization of turret tracking.

    Displays an OpenCV window with the camera feed, detection overlays,
    and debug information.
    """

    # Color scheme (BGR format)
    COLOR_DETECTION = (0, 255, 0)      # Green - detected persons
    COLOR_TARGET = (0, 0, 255)         # Red - current target
    COLOR_LOCKED = (0, 255, 255)       # Yellow - locked on
    COLOR_CROSSHAIR = (255, 255, 255)  # White - center crosshairs
    COLOR_TEXT = (255, 255, 255)       # White - text
    COLOR_TEXT_BG = (0, 0, 0)          # Black - text background

    def __init__(self, window_name: str = "Laser Turret"):
        """
        Initialize the visualizer.

        Args:
            window_name: Name of the OpenCV window.
        """
        self.window_name = window_name
        self.window_created = False

    def _create_window(self):
        """Create the OpenCV window if not already created."""
        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 640, 480)
            self.window_created = True

    def draw_frame(
        self,
        frame: np.ndarray,
        detections: List[Detection],
        target_state: Optional[TargetState],
        servo_angles: Tuple[int, int],
        error: Tuple[float, float],
        laser_on: bool,
        fps: float = 0.0
    ) -> np.ndarray:
        """
        Draw all overlays on the frame.

        Args:
            frame: BGR image to draw on.
            detections: List of all detected persons.
            target_state: Current targeting state.
            servo_angles: Current (pan, tilt) servo angles.
            error: Current (x, y) pixel error.
            laser_on: Whether laser is currently firing.
            fps: Current frames per second.

        Returns:
            Frame with overlays drawn.
        """
        # Make a copy to avoid modifying original
        display = frame.copy()
        h, w = display.shape[:2]

        # Draw all detection bounding boxes
        for i, det in enumerate(detections):
            is_current_target = (
                target_state is not None and
                target_state.current_target is not None and
                det.center == target_state.current_target.center
            )

            self._draw_detection(display, det, i, is_current_target, target_state)

        # Draw center crosshairs
        self._draw_crosshairs(display, w, h)

        # Draw debug info panel
        self._draw_info_panel(
            display, target_state, servo_angles, error, laser_on, fps, len(detections)
        )

        # Draw laser indicator if firing
        if laser_on:
            self._draw_laser_indicator(display, w, h)

        return display

    def _draw_detection(
        self,
        frame: np.ndarray,
        detection: Detection,
        index: int,
        is_target: bool,
        target_state: Optional[TargetState]
    ):
        """Draw a single detection with bounding box and label."""
        x1, y1, x2, y2 = detection.bbox
        cx, cy = detection.center

        # Choose color based on whether this is the current target
        if is_target:
            if target_state and target_state.is_locked:
                color = self.COLOR_LOCKED
                thickness = 3
            else:
                color = self.COLOR_TARGET
                thickness = 3
        else:
            color = self.COLOR_DETECTION
            thickness = 2

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        # Draw center point
        cv2.circle(frame, (cx, cy), 5, color, -1)

        # Draw corners for current target (emphasize)
        if is_target:
            corner_len = 20
            # Top-left
            cv2.line(frame, (x1, y1), (x1 + corner_len, y1), color, 3)
            cv2.line(frame, (x1, y1), (x1, y1 + corner_len), color, 3)
            # Top-right
            cv2.line(frame, (x2, y1), (x2 - corner_len, y1), color, 3)
            cv2.line(frame, (x2, y1), (x2, y1 + corner_len), color, 3)
            # Bottom-left
            cv2.line(frame, (x1, y2), (x1 + corner_len, y2), color, 3)
            cv2.line(frame, (x1, y2), (x1, y2 - corner_len), color, 3)
            # Bottom-right
            cv2.line(frame, (x2, y2), (x2 - corner_len, y2), color, 3)
            cv2.line(frame, (x2, y2), (x2, y2 - corner_len), color, 3)

        # Draw label
        label = f"#{index + 1} ({detection.confidence:.0%})"
        if is_target:
            label = "TARGET " + label

        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        cv2.rectangle(
            frame,
            (x1, y1 - label_size[1] - 8),
            (x1 + label_size[0] + 4, y1),
            color,
            -1
        )
        cv2.putText(
            frame, label,
            (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 0, 0), 1
        )

    def _draw_crosshairs(self, frame: np.ndarray, width: int, height: int):
        """Draw center crosshairs."""
        cx, cy = width // 2, height // 2
        size = 20
        gap = 5

        # Horizontal lines
        cv2.line(frame, (cx - size, cy), (cx - gap, cy), self.COLOR_CROSSHAIR, 1)
        cv2.line(frame, (cx + gap, cy), (cx + size, cy), self.COLOR_CROSSHAIR, 1)

        # Vertical lines
        cv2.line(frame, (cx, cy - size), (cx, cy - gap), self.COLOR_CROSSHAIR, 1)
        cv2.line(frame, (cx, cy + gap), (cx, cy + size), self.COLOR_CROSSHAIR, 1)

        # Center dot
        cv2.circle(frame, (cx, cy), 2, self.COLOR_CROSSHAIR, -1)

    def _draw_info_panel(
        self,
        frame: np.ndarray,
        target_state: Optional[TargetState],
        servo_angles: Tuple[int, int],
        error: Tuple[float, float],
        laser_on: bool,
        fps: float,
        num_detections: int
    ):
        """Draw debug information panel."""
        h, w = frame.shape[:2]
        panel_width = 200
        panel_height = 140
        margin = 10

        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (margin, margin),
            (margin + panel_width, margin + panel_height),
            self.COLOR_TEXT_BG,
            -1
        )
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        # Draw text
        y = margin + 20
        line_height = 18

        def draw_text(text: str, color=self.COLOR_TEXT):
            nonlocal y
            cv2.putText(
                frame, text,
                (margin + 10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                color, 1
            )
            y += line_height

        draw_text(f"FPS: {fps:.1f}")
        draw_text(f"Targets: {num_detections}")

        pan, tilt = servo_angles
        draw_text(f"Pan: {pan}  Tilt: {tilt}")

        ex, ey = error
        draw_text(f"Error: ({ex:.0f}, {ey:.0f})")

        if target_state:
            draw_text(f"Target: {target_state.target_index + 1}/{target_state.total_targets}")
            draw_text(f"Switch in: {target_state.time_until_switch:.1f}s")

            if target_state.is_locked:
                draw_text("STATUS: LOCKED", self.COLOR_LOCKED)
            elif target_state.current_target:
                draw_text("STATUS: TRACKING", self.COLOR_TARGET)
            else:
                draw_text("STATUS: SEARCHING", self.COLOR_DETECTION)
        else:
            draw_text("STATUS: NO TARGET", (128, 128, 128))

        # Laser status
        if laser_on:
            draw_text("LASER: FIRING", (0, 0, 255))
        else:
            draw_text("LASER: OFF", (128, 128, 128))

    def _draw_laser_indicator(self, frame: np.ndarray, width: int, height: int):
        """Draw visual indicator when laser is firing."""
        # Red border flash
        cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (0, 0, 255), 4)

        # "FIRING" text at bottom
        text = "LASER ACTIVE"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
        text_x = (width - text_size[0]) // 2
        text_y = height - 20

        cv2.putText(
            frame, text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (0, 0, 255), 2
        )

    def show(self, frame: np.ndarray) -> bool:
        """
        Display frame and handle window events.

        Args:
            frame: Frame to display.

        Returns:
            False if window was closed (user pressed 'q' or closed window),
            True otherwise.
        """
        self._create_window()
        cv2.imshow(self.window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # 'q' or ESC
            return False

        # Check if window was closed
        try:
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except cv2.error:
            return False

        return True

    def close(self):
        """Close the visualization window."""
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            self.window_created = False

    def __del__(self):
        """Cleanup on destruction."""
        self.close()


def test_visualizer():
    """Test the visualizer with synthetic data."""
    import time

    vis = Visualizer("Test Visualization")

    # Create a test frame
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[:] = (50, 50, 50)  # Dark gray background

    # Create mock detections
    detections = [
        Detection.from_bbox(50, 50, 100, 150, 0.95),
        Detection.from_bbox(200, 60, 260, 180, 0.85),
    ]

    # Create mock target state
    target_state = TargetState(
        current_target=detections[0],
        target_index=0,
        total_targets=2,
        time_until_switch=2.5,
        is_locked=True
    )

    print("Press 'q' to quit")

    frame_count = 0
    start_time = time.time()

    while True:
        # Animate the frame a bit
        test_frame = frame.copy()

        # Calculate FPS
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0

        # Draw frame with overlays
        display = vis.draw_frame(
            test_frame,
            detections,
            target_state,
            servo_angles=(90, 90),
            error=(15.0, -8.0),
            laser_on=(frame_count % 60 < 30),  # Blink laser
            fps=fps
        )

        if not vis.show(display):
            break

        frame_count += 1
        time.sleep(1 / 30)  # ~30 FPS

    vis.close()


if __name__ == "__main__":
    test_visualizer()

