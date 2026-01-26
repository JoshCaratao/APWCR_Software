import time
import cv2


class ComputerVision:
    """
    Perception module that owns:
      - Camera
      - Detector

    Responsibilities (inside this class):
      - Run detection when tick() is called
      - Apply anti-flicker (min consecutive detections + hold time)
      - Draw overlay text + show OpenCV window (optional)

    Not responsible for:
      - Scheduling / inference Hz (do that in main with utils.Rate)
      - State machine logic
      - Motor control
    """

    def __init__(
        self,
        camera,
        detector,
        min_consecutive_detections: int = 2,
        hold_seconds: float = 0.4,
        window_name: str = "Pet Waste Detection - Live",
        show_window: bool = True,
    ):
        self.camera = camera
        self.detector = detector

        self.min_streak = int(min_consecutive_detections)
        self.hold_s = float(hold_seconds)

        self.window_name = window_name
        self.show_window = bool(show_window)

        # Anti-flicker state
        self.streak = 0
        self.stable_detected = False
        self._last_stable_t = 0.0
        self.stable_center = None  # (cx, cy, conf)

        # For display: keep last annotated frame (so window still updates if you want)
        self._last_display = None

        self._started = False

    def start(self) -> bool:
        """
        Open camera once.
        """
        if self._started:
            return True
        ok = self.camera.open()
        self._started = ok
        return ok

    def tick(self):
        """
        One perception update:
          - read a frame
          - run detector on that frame
          - update anti-flicker state
          - draw overlay and optionally show window

        Returns:
          obs dict, or None if frame read failed.

        obs keys:
          - frame
          - display_frame
          - r0 (latest YOLO Results[0] or None)
          - best (latest best detection center (cx,cy,conf) or None)
          - stable_detected (bool)
          - stable_center (cx,cy,conf) or None
          - streak
          - timestamp
        """
        if not self._started:
            raise RuntimeError("ComputerVision not started. Call start() first.")

        ret, frame = self.camera.read()
        if not ret or frame is None:
            return None

        now = time.perf_counter()

        # Run inference (main controls how often tick() is called)
        r0, annotated, best = self.detector.detect(frame)
        self._last_display = annotated

        detected_now = self.detector.has_valid_detection(r0)

        # Update streak counter
        if detected_now:
            self.streak += 1
        else:
            self.streak = 0

        # Update stable detection state
        if self.streak >= self.min_streak:
            self.stable_detected = True
            self._last_stable_t = now
            self.stable_center = best
        else:
            # Hold stable state briefly to prevent rapid on/off
            if self.stable_detected and self.hold_s > 0.0:
                if (now - self._last_stable_t) > self.hold_s:
                    self.stable_detected = False
                    self.stable_center = None
            else:
                self.stable_detected = False
                self.stable_center = None

        display_frame = self._last_display if self._last_display is not None else frame

        # Overlay status (no hz shown here because scheduling is done in main)
        status = "STABLE DETECTION" if self.stable_detected else "searching"
        cv2.putText(
            display_frame,
            f"{status} | streak={self.streak}/{self.min_streak} | imgsz={self.detector.imgsz}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if self.stable_detected else (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Show center coords when stable (handy for control debugging)
        if self.stable_detected and self.stable_center is not None:
            cx, cy, conf = self.stable_center
            cv2.putText(
                display_frame,
                f"center=({cx},{cy}) conf={conf:.2f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        if self.show_window:
            cv2.imshow(self.window_name, display_frame)

        return {
            "frame": frame,
            "display_frame": display_frame,
            "r0": r0,
            "best": best,
            "stable_detected": self.stable_detected,
            "stable_center": self.stable_center,
            "streak": self.streak,
            "timestamp": now,
        }

    def should_quit(self) -> bool:
        """
        True if user pressed 'q' (only relevant if show_window is True).
        """
        if not self.show_window:
            return False
        return (cv2.waitKey(1) & 0xFF) == ord("q")

    def stop(self):
        """
        Release resources.
        """
        self.camera.release()
        if self.show_window:
            cv2.destroyAllWindows()
        self._started = False
