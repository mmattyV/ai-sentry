"""
Person Detection Module for Laser Turret

Uses YOLOv8 for real-time person detection with full-body bounding boxes.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Run: pip install ultralytics")


@dataclass
class Detection:
    """Represents a detected person."""
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2) - top-left and bottom-right
    confidence: float
    center: Tuple[int, int]  # (cx, cy) - center of bounding box
    area: int  # Area of bounding box in pixels

    @classmethod
    def from_bbox(cls, x1: int, y1: int, x2: int, y2: int, confidence: float) -> "Detection":
        """Create a Detection from bounding box coordinates."""
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        area = (x2 - x1) * (y2 - y1)
        return cls(
            bbox=(x1, y1, x2, y2),
            confidence=confidence,
            center=(cx, cy),
            area=area
        )


class PersonDetector:
    """YOLOv8-based person detector."""

    # COCO class ID for person
    PERSON_CLASS_ID = 0

    def __init__(self, model_name: str = "yolov8n.pt", confidence_threshold: float = 0.5):
        """
        Initialize the person detector.

        Args:
            model_name: YOLOv8 model to use. Options:
                - 'yolov8n.pt' (nano - fastest, least accurate)
                - 'yolov8s.pt' (small)
                - 'yolov8m.pt' (medium)
                - 'yolov8l.pt' (large)
                - 'yolov8x.pt' (extra large - slowest, most accurate)
            confidence_threshold: Minimum confidence score for detections.
        """
        self.confidence_threshold = confidence_threshold
        self.model: Optional[YOLO] = None
        self.device = 'cpu'

        if not YOLO_AVAILABLE:
            raise RuntimeError("ultralytics package not installed")

        # Try to use MPS (Apple Silicon GPU) for faster inference
        try:
            import torch
            if torch.backends.mps.is_available():
                self.device = 'mps'
                print("Using MPS (Apple Silicon GPU) for inference")
            else:
                print("MPS not available, using CPU")
        except Exception:
            print("Could not check MPS availability, using CPU")

        print(f"Loading YOLOv8 model: {model_name}")
        self.model = YOLO(model_name)
        print("Model loaded successfully")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Detect persons in a frame.

        Args:
            frame: BGR image as numpy array (from OpenCV).

        Returns:
            List of Detection objects for each person found.
        """
        if self.model is None:
            return []

        # Run inference on GPU (MPS) if available
        results = self.model(
            frame,
            conf=self.confidence_threshold,
            classes=[self.PERSON_CLASS_ID],  # Only detect persons
            device=self.device,
            verbose=False
        )

        detections = []

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                # Get bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0].cpu().numpy())

                detection = Detection.from_bbox(
                    int(x1), int(y1), int(x2), int(y2), confidence
                )
                detections.append(detection)

        # Sort by area (largest first - typically closest person)
        detections.sort(key=lambda d: d.area, reverse=True)

        return detections

    def detect_from_jpeg(self, jpeg_data: bytes) -> Tuple[Optional[np.ndarray], List[Detection]]:
        """
        Detect persons from JPEG data.

        Args:
            jpeg_data: Raw JPEG image data.

        Returns:
            Tuple of (decoded frame, list of detections).
            Frame may be None if decoding fails.
        """
        import cv2

        # Decode JPEG to numpy array
        nparr = np.frombuffer(jpeg_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is None:
            return None, []

        detections = self.detect(frame)
        return frame, detections


class MockDetector:
    """Mock detector for testing without YOLO."""

    def __init__(self):
        print("Using mock detector (no actual detection)")
        self.frame_count = 0

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Return mock detections for testing."""
        self.frame_count += 1

        # Create a fake detection that moves around
        import math
        t = self.frame_count / 30.0  # Time in "seconds"

        h, w = frame.shape[:2]
        cx = int(w/2 + w/4 * math.sin(t))
        cy = int(h/2 + h/4 * math.cos(t * 0.7))

        # Create a bounding box around the center
        box_w, box_h = 60, 120
        x1 = max(0, cx - box_w // 2)
        y1 = max(0, cy - box_h // 2)
        x2 = min(w, cx + box_w // 2)
        y2 = min(h, cy + box_h // 2)

        return [Detection.from_bbox(x1, y1, x2, y2, 0.95)]

    def detect_from_jpeg(self, jpeg_data: bytes) -> Tuple[Optional[np.ndarray], List[Detection]]:
        """Decode and run mock detection."""
        import cv2
        nparr = np.frombuffer(jpeg_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is None:
            return None, []

        detections = self.detect(frame)
        return frame, detections


def create_detector(use_mock: bool = False, **kwargs) -> PersonDetector:
    """
    Factory function to create a detector.

    Args:
        use_mock: If True, return a mock detector for testing.
        **kwargs: Arguments passed to PersonDetector.

    Returns:
        A detector instance.
    """
    if use_mock or not YOLO_AVAILABLE:
        return MockDetector()
    return PersonDetector(**kwargs)


if __name__ == "__main__":
    # Test detection on webcam
    import cv2

    detector = create_detector()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open webcam")
        exit(1)

    print("Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.detect(frame)

        # Draw detections
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cx, cy = det.center

            # Bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Center point
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Label
            label = f"Person: {det.confidence:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Person Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

