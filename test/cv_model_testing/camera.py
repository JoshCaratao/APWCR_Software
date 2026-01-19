import cv2

class Camera:
    def __init__(self, index: int = 0, width: int | None = None, height: int | None = None):
        """
        Simple camera wrapper.
        index: which camera to open (0 = default)
        width, height: optional resolution hints
        """
        self.index = index
        self.width = width
        self.height = height
        self.cap = None

    # Open Camera Function
    def open(self) -> bool:
        # Use DirectShow backend on Windows to avoid long hangs
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)

        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.index}.")
            return False

        if self.width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        return True
    
    # Read Camera Frames
    def read(self):
        """
        Returns (ret, frame) like cv2.VideoCapture.read().
        """
        if self.cap is None:
            raise RuntimeError("Camera not opened. Call open() first.")
        return self.cap.read()

    # Release Camera
    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
