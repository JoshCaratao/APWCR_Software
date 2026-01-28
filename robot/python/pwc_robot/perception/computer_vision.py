import time
import cv2
import threading


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
        self._latest_annotated_frame = None

        # Threading lock for latest annotated frame protection
        self._cv_lock = threading.Lock()

        # Latest Observation data
        self._latest_obs = None



        self._started = False

    def start(self) -> bool:
        """
        Open camera once.
        """
        if self._started:
            return True
        cam_started = self.camera.start()
        self._started = cam_started
        return cam_started

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

        # Get latest available camera frame 
        frame = self.camera.get_latest_frame()
        if frame is None:
            return None

        now = time.perf_counter()

        # Run inference (main controls how often tick() is called)
        r0, annotated, best = self.detector.detect(frame)

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

        display_frame = annotated if annotated is not None else frame


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
        
        # Update latest annotated frame after final annotations are done with display frame
        with self._cv_lock:
            self._latest_annotated_frame = display_frame.copy()


        if self.show_window:
            cv2.imshow(self.window_name, display_frame)

        # Return obs (full) for main loop use
        obs = {
            "frame": frame,
            "display_frame": display_frame,
            "r0": r0,
            "best": best,
            "stable_detected": self.stable_detected,
            "stable_center": self.stable_center,
            "streak": self.streak,
            "timestamp": now,
        }

        # Store lightweight obs for Flask/UI (no big numpy arrays)
        latest_obs = {
                "best": list(best) if best is not None else None,
                "stable_detected": self.stable_detected,
                "stable_center": list(self.stable_center) if self.stable_center is not None else None,
                "streak": self.streak,
                "timestamp": now,
        }


        with self._cv_lock:
            self._latest_obs = latest_obs
        
        return obs
        
        
    def should_quit(self) -> bool:
        """
        True if user pressed 'q' (only relevant if show_window is True).
        """
        if not self.show_window:
            return False
        return (cv2.waitKey(1) & 0xFF) == ord("q")
    
    def get_latest_annotated_frame(self):
        with self._cv_lock:
            if self._latest_annotated_frame is None:
                return None
            return self._latest_annotated_frame.copy()
        
    def get_latest_obs(self):
        with self._cv_lock:
            return None if self._latest_obs is None else dict(self._latest_obs)


    def stop(self):
        """
        Release resources.
        """
        self.camera.stop()
        if self.show_window:
            cv2.destroyAllWindows()
        self._started = False
