import cv2
import platform
import threading
import time


class Camera:
    def __init__(
        self,
        index: int = 0,
        width: int | None = None,
        height: int | None = None,
        capture_hz: float | None = None,   # None means "run as fast as camera gives"
        copy_on_get: bool = True,          # safest default for multi-thread use
    ):
        """
        Threaded camera wrapper.

        index: which camera to open (0 = default)
        width, height: optional resolution hints
        capture_hz: optional loop rate limit for the capture thread
        copy_on_get: if True, get_latest_frame returns a copy to avoid shared-memory issues
        """
        self.index = index
        self.width = width
        self.height = height
        self.capture_hz = capture_hz
        self.copy_on_get = copy_on_get

        self.cap = None

        # Threading state
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # Latest frame buffer
        self._latest_frame = None
        self._latest_ts = 0.0
        self._started = False

    def open(self) -> bool:
        """
        Open the camera device.
        Kept for compatibility, but typically you will call start().
        """
        if platform.system() == "Windows":
            self.cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(self.index)

        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.index}.")
            return False
        
        # request fps + size
        self.cap.set(cv2.CAP_PROP_FPS, self.capture_hz)
        if self.width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        if self.height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))

        # Verify what we actually got
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[Camera] requested={self.width}x{self.height}  actual={actual_w}x{actual_h}")

        return True

    def start(self) -> bool:
        """
        Open the camera (if needed) and start the capture thread.
        """
        if self._started:
            return True

        ok = self.open()
        if not ok:
            return False

        # Clear thread even _stop_event to "allow" thread to run
        self._stop_event.clear()

        # Create threading object. Function that threading object will run is _run() function
        self._thread = threading.Thread(target=self._run, daemon=True)

        # Begin Thread Function and update camera thread state
        self._thread.start()
        self._started = True
        return True

    def _run(self) -> None:
        """
        Capture loop running in a background thread.
        """

        # Optional rate limiting. If capture_hz is absent or if its 0, then period is 0
        period = (1.0 / self.capture_hz) if (self.capture_hz and self.capture_hz > 0) else 0.0

        while not self._stop_event.is_set():
            if self.cap is None:
                break
            t0 = time.perf_counter()
            ret, frame = self.cap.read()
            now = time.perf_counter()

            # Update latest_frame data with capture frame
            if ret and frame is not None:
                # Lock thread to prevent cv object from accessing mid write
                with self._lock:
                    self._latest_frame = frame
                    self._latest_ts = now

            if period > 0.0:
                dt = time.perf_counter() - t0
                sleep_s = period - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)

    def get_latest_frame(self):
        """
        Returns the latest frame (or None if none captured yet).
        """
        with self._lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy() if self.copy_on_get else self._latest_frame

    def get_latest_timestamp(self) -> float:
        """
        Timestamp of the latest captured frame (perf_counter seconds).
        """
        with self._lock:
            return float(self._latest_ts)

    @property
    def is_open(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def get_resolution(self) -> tuple[int, int]:
        if self.cap is None:
            raise RuntimeError("Camera not opened.")
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def stop(self) -> None:
        """
        Stop capture thread and release device.
        """
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        self.release()
        self._started = False

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        # Clear buffers
        with self._lock:
            self._latest_frame = None
            self._latest_ts = 0.0
