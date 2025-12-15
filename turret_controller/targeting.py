"""
Target Management Module for Laser Turret

Handles multi-target tracking with automatic cycling between detected persons.
"""

import time
from typing import List, Optional, Tuple
from dataclasses import dataclass
from detector import Detection


@dataclass
class TargetState:
    """Current targeting state information."""
    current_target: Optional[Detection]
    target_index: int
    total_targets: int
    time_until_switch: float
    is_locked: bool


class TargetManager:
    """
    Manages target selection and cycling between multiple detected persons.

    Features:
    - Automatic cycling between targets at configurable intervals
    - Immediate switch when current target is lost
    - Returns to neutral when no targets are available
    - Tracks lock-on state based on aim error
    """

    def __init__(
        self,
        cycle_interval: float = 3.0,
        lock_threshold: float = 20.0,
        neutral_position: Tuple[int, int] = (160, 120)
    ):
        """
        Initialize the target manager.

        Args:
            cycle_interval: Time in seconds between target switches.
            lock_threshold: Maximum pixel error to consider "locked on".
            neutral_position: Default aim position when no targets (frame center).
        """
        self.cycle_interval = cycle_interval
        self.lock_threshold = lock_threshold
        self.neutral_position = neutral_position

        self.current_index = 0
        self.last_switch_time = time.time()
        self.current_target: Optional[Detection] = None
        self.is_locked = False

    def update(self, detections: List[Detection], aim_error: Tuple[float, float]) -> TargetState:
        """
        Update target selection based on current detections.

        Args:
            detections: List of detected persons from the detector.
            aim_error: Current (x_error, y_error) in pixels.

        Returns:
            TargetState with current targeting information.
        """
        current_time = time.time()
        time_since_switch = current_time - self.last_switch_time

        num_targets = len(detections)

        if num_targets == 0:
            # No targets available
            self.current_target = None
            self.current_index = 0
            self.is_locked = False
            return TargetState(
                current_target=None,
                target_index=-1,
                total_targets=0,
                time_until_switch=self.cycle_interval,
                is_locked=False
            )

        # Check if we should switch targets
        should_switch = False

        # Check if enough time has passed for automatic cycling
        if time_since_switch >= self.cycle_interval and num_targets > 1:
            should_switch = True

        # Check if current target index is out of bounds
        if self.current_index >= num_targets:
            should_switch = True
            self.current_index = 0

        # Check if current target was lost (using IoU or position comparison)
        # For simplicity, we just use index bounds checking above

        if should_switch:
            self.current_index = (self.current_index + 1) % num_targets
            self.last_switch_time = current_time
            time_since_switch = 0

        # Get current target
        self.current_target = detections[self.current_index]

        # Check lock-on state
        error_magnitude = (aim_error[0]**2 + aim_error[1]**2) ** 0.5
        self.is_locked = error_magnitude < self.lock_threshold

        time_until_switch = max(0, self.cycle_interval - time_since_switch)

        return TargetState(
            current_target=self.current_target,
            target_index=self.current_index,
            total_targets=num_targets,
            time_until_switch=time_until_switch,
            is_locked=self.is_locked
        )

    def get_target_position(self) -> Tuple[int, int]:
        """
        Get the position to aim at.

        Returns:
            (x, y) pixel coordinates of target center, or neutral position if no target.
        """
        if self.current_target is not None:
            return self.current_target.center
        return self.neutral_position

    def force_switch(self):
        """Force an immediate switch to the next target."""
        self.current_index += 1
        self.last_switch_time = time.time()

    def reset(self):
        """Reset targeting state."""
        self.current_index = 0
        self.last_switch_time = time.time()
        self.current_target = None
        self.is_locked = False

    def set_cycle_interval(self, interval: float):
        """Update the cycle interval."""
        self.cycle_interval = max(0.5, interval)  # Minimum 0.5 seconds

    def set_lock_threshold(self, threshold: float):
        """Update the lock-on threshold."""
        self.lock_threshold = max(1.0, threshold)  # Minimum 1 pixel


class TargetTracker:
    """
    Advanced target tracker that maintains identity across frames.

    Uses IoU (Intersection over Union) to match detections across frames,
    providing more stable tracking when targets move.
    """

    def __init__(self, iou_threshold: float = 0.3):
        """
        Initialize the target tracker.

        Args:
            iou_threshold: Minimum IoU to consider same target.
        """
        self.iou_threshold = iou_threshold
        self.tracked_targets: List[Detection] = []
        self.target_ids: List[int] = []
        self.next_id = 0

    @staticmethod
    def compute_iou(box1: Tuple[int, int, int, int], box2: Tuple[int, int, int, int]) -> float:
        """Compute Intersection over Union between two bounding boxes."""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2

        # Compute intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)

        if x2_i < x1_i or y2_i < y1_i:
            return 0.0

        intersection = (x2_i - x1_i) * (y2_i - y1_i)

        # Compute union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection

        if union == 0:
            return 0.0

        return intersection / union

    def update(self, detections: List[Detection]) -> List[Tuple[int, Detection]]:
        """
        Update tracked targets with new detections.

        Args:
            detections: New detections from current frame.

        Returns:
            List of (id, detection) tuples with stable IDs.
        """
        if not self.tracked_targets:
            # First frame - assign new IDs to all
            self.tracked_targets = detections.copy()
            self.target_ids = list(range(self.next_id, self.next_id + len(detections)))
            self.next_id += len(detections)
            return list(zip(self.target_ids, self.tracked_targets))

        # Match detections to tracked targets using IoU
        matched_detections = []
        matched_indices = set()
        new_target_ids = []
        new_tracked = []

        for det in detections:
            best_iou = 0
            best_idx = -1

            for idx, tracked in enumerate(self.tracked_targets):
                if idx in matched_indices:
                    continue

                iou = self.compute_iou(det.bbox, tracked.bbox)
                if iou > best_iou:
                    best_iou = iou
                    best_idx = idx

            if best_iou >= self.iou_threshold:
                # Matched to existing target
                matched_indices.add(best_idx)
                new_target_ids.append(self.target_ids[best_idx])
                new_tracked.append(det)
            else:
                # New target
                new_target_ids.append(self.next_id)
                new_tracked.append(det)
                self.next_id += 1

        self.tracked_targets = new_tracked
        self.target_ids = new_target_ids

        return list(zip(self.target_ids, self.tracked_targets))


if __name__ == "__main__":
    # Test target manager
    manager = TargetManager(cycle_interval=2.0)

    # Create mock detections
    mock_detections = [
        Detection.from_bbox(50, 50, 100, 150, 0.9),
        Detection.from_bbox(200, 60, 260, 180, 0.85),
        Detection.from_bbox(100, 100, 180, 220, 0.75),
    ]

    print("Testing target cycling...")
    print(f"Cycle interval: {manager.cycle_interval}s")
    print(f"Number of targets: {len(mock_detections)}")
    print()

    for i in range(10):
        state = manager.update(mock_detections, (10, 5))

        print(f"Frame {i+1}:")
        print(f"  Target: {state.target_index + 1}/{state.total_targets}")
        print(f"  Position: {manager.get_target_position()}")
        print(f"  Locked: {state.is_locked}")
        print(f"  Time until switch: {state.time_until_switch:.1f}s")
        print()

        time.sleep(0.5)


