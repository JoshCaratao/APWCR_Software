import time
import cv2
import threading


class ComputerVision:
    """
    Perception module that owns:
      - Camera
      - Detector

    Responsibilities:
      - Read latest camera frame
      - Run detector when tick() is called
      - Select a target from detections (based on configured targetting policy)
      - Determine stable detection based on target presence for N consecutive ticks
      - Draw overlay text + show OpenCV window (optional

    Not responsible for:
      - Scheduling / inference Hz (do that in main with utils.Rate)
      - State machine logic
      - Motor control
    """

    def __init__(
        self,
        camera,
        detector,
        window_name: str = "Pet Waste Detection - Live",
        show_window: bool = True,
        target_infer_hz: float | None = None,
        targeting_mode: str = "area", # Options are "area", "conf", "conf_area", but this should be set in config file
        targeting_conf_w: float = 0.5,
        targeting_area_w: float = 0.5,
        stable_window: int = 5
    ):
        self.camera = camera
        self.detector = detector

        self.window_name = window_name
        self.show_window = bool(show_window)


        # For gui display: keep last annotated frame (so window still updates if you want)
        self._latest_annotated_frame = None
        self._latest_obs = None
        self._cv_lock = threading.Lock()


        # Target + measured inference rate (Hz)
        self.target_infer_hz = None if target_infer_hz is None else float(target_infer_hz)
        self._last_tick_t = None
        self._measured_infer_hz_ema = None

        # Targetting Mode/Policy
        self.targeting_mode = str(targeting_mode)
        self.targeting_conf_w = float(targeting_conf_w)
        self.targeting_area_w = float(targeting_area_w)

        # Confirm chosen target mode is valid
        allowed = {"area", "conf", "conf_area"}
        if self.targeting_mode not in allowed:
            raise ValueError(f"INVALID TARGETING MODE ='{self.targeting_mode}'. ALLOWED: {sorted(allowed)}")

        # Stability
        self.stable_window = int(stable_window)
        if self.stable_window < 1:
            raise ValueError("stable_window must be >= 1")
        self._stable_count = 0

        # Variable to say whether camera has started
        self._started = False


    def start(self) -> bool:
        """
        Open camera once.
        """
        if self._started:
            return True
        self._started = self.camera.start()
        return self._started
    
    
    def select_target(self, candidates):
        """
        candidates: list of dicts with keys cx, cy, conf, area, xyxy
        returns: target dict or None
        """

        if not candidates:
            return None
        
        # Return candidates with Max confidence level if confidence targeting mode
        if self.targeting_mode == "conf":
            return max(candidates, key = lambda d: d["conf"])
        
        # Return candidates with Max area if area targeting mode
        elif self.targeting_mode == "area":
            return max(candidates, key = lambda d: d["area"])
        
        # Return Max conf and area score if conf+area is used for targeting.
        elif self.targeting_mode == "conf_area":
            # Normalize area
            max_area = max(d["area"] for d in candidates) or 1.0
            def score(d):
                area_norm = d["area"] / max_area
                return (self.targeting_conf_w * d["conf"]) + (self.targeting_area_w * area_norm)

            return max(candidates, key = score)
        
        else:
            return None
    

    def tick(self):
        
        if not self._started:
            raise RuntimeError("ComputerVision not started. Call start() first.")

        # Get latest available camera frame 
        frame = self.camera.get_latest_frame()
        if frame is None:
            return None

        now = time.perf_counter()

        # --- Measure actual inference rate (real achieved tick rate) ---
        if self._last_tick_t is not None:
            dt = now - self._last_tick_t
            if dt > 0:
                inst_hz = 1.0 / dt
                ema = self._measured_infer_hz_ema
                self._measured_infer_hz_ema = inst_hz if ema is None else (0.8 * ema + 0.2 * inst_hz)
        self._last_tick_t = now

        # Run YOLO Inference
        r0, annotated_frame, candidates, num_detections = self.detector.detect(frame)

        # Obtain Target first
        target = self.select_target(candidates)

        # Then evaluate stability of selected target
        if target is not None:
            self._stable_count +=1
        else:
            self._stable_count = 0

        stable_detected = self._stable_count >= self.stable_window
        stable_target = target if stable_detected else None
        
        # Display Frame
        display_frame = annotated_frame if annotated_frame is not None else frame

        # Draw Crosshair on chosen target (even if not stable yet, helps debugging)
        if target is not None:
            self.detector.draw_crosshair(display_frame, target["cx"], target["cy"])
        
        # Draw Red Box on chosen target if stable
        if stable_target is not None and "xyxy" in stable_target:
            x1,y1,x2,y2 = target["xyxy"]

            # Ensure ints for cv2
            x1,y1,x2,y2 = int(x1), int(y1), int(x2), int(y2)

            # Red outline for the selected target
            cv2.rectangle(display_frame, (x1,y1), (x2,y2), (0,0,255), 3)


        # Create Overlay status + metrics
        status_str = "STABLE" if stable_detected else ("DETECTED" if num_detections > 0 else "SEARCHING")
        hz_str = "Inf Hz=N/A" if self._measured_infer_hz_ema is None else f"Inf Hz={self._measured_infer_hz_ema:.1f}Hz"

        # Overlay frame with status text
        cv2.putText(
            display_frame,
            f"{status_str} | N={num_detections} | imgsz={self.detector.imgsz} | {hz_str}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if stable_detected else (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Show center coords when stable (handy for control debugging)
        if target is not None:
            cv2.putText(
                display_frame,
                f"Target=({target['cx']},{target['cy']}) conf={target['conf']:.2f} area={target['area']:.0f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        
        # Save latest annotated frame for GUI
        with self._cv_lock:
            self._latest_annotated_frame = display_frame.copy()


        if self.show_window:
            cv2.imshow(self.window_name, display_frame)

        # Full observation data for main loop (includes candidates list)
        obs = {
            "frame": frame,
            "display_frame": display_frame,
            "r0": r0,
            "candidates": candidates,
            "num_detections": num_detections,
            "target": target,
            "stable_detected": stable_detected,
            "stable_target": stable_target,
            "stable_count": self._stable_count,
            "stable_window": self.stable_window,
            "timestamp": now,
            "target_infer_hz": self.target_infer_hz,
            "measured_infer_hz": self._measured_infer_hz_ema,
        }

        # Lightweight obs for Flask/UI
        target_status = "Stable Detection" if stable_detected else ("Detected" if target is not None else "Searching")
        target_label = "Selected" if target is not None else "N/A"

        latest_obs = {
            # Speeds
            "target_infer_hz": self.target_infer_hz,                 # what you requested / configured
            "measured_infer_hz": self._measured_infer_hz_ema,        # what you are actually achieving

            # High-level detection info
            "num_detections": num_detections,
            "target_policy": self.targeting_mode,                    # "area", "conf", "conf_area"
            "target": target_label,                                  # "Selected" or "N/A"
            "target_status": target_status,                          # "Detected", "Stable Detection", "Searching"

            # Target details (only present if target exists)
            "target_data": None if target is None else {
                "conf": float(target["conf"]),
                "area": float(target["area"]),
                "cx": int(target["cx"]),
                "cy": int(target["cy"]),
                "xyxy": [int(v) for v in target["xyxy"]],
            },

            # Optional but often useful in UI
            "stable_count": self._stable_count,
            "stable_window": self.stable_window,
            "timestamp": now,
        }

        # Update latest_obs with thread locking to prevent corrupted data
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
